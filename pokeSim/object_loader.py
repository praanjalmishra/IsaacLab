# object_loader.py

import omni
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

def load_objects(stage):
    # Ensure PhysX is enabled for deformable simulation
    physx_interface = omni.physx.acquire_physx_interface()
    physx_interface.create_physx_scene()

    # Load the deformable cube from assets
    asset_path = "/Isaac/Props/Deformable/Cube/cube.usd"  # Update with the correct asset path
    cube_prim_path = "/World/DeformableCube"

    # Add the cube to the stage
    result = omni.usd.get_context().get_stage().DefinePrim(cube_prim_path, "Xform")
    cube_prim = stage.DefinePrim(cube_prim_path)
    omni.usd.get_context().get_stage().GetRootLayer().Reload()

    # Reference the asset
    cube_prim.GetReferences().AddReference(asset_path)

    # Apply necessary physics APIs
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    PhysxSchema.PhysxDeformableBodyAPI.Apply(cube_prim)

    # Set deformable physics properties
    deformable_api = PhysxSchema.PhysxDeformableBodyAPI(cube_prim)
    deformable_api.CreateDeformableRestOffsetAttr(0.0)
    deformable_api.CreateDeformableDampingAttr(0.1)
    deformable_api.CreateSelfCollisionAttr(True)
    # Adjust other properties like stiffness and damping as needed
    deformable_api.CreateYoungsModulusAttr(5000.0)  # Adjust for desired softness
    deformable_api.CreatePoissonsRatioAttr(0.3)

    # Position the cube in the scene
    cube_prim.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.05))  # Adjust Z for placement above ground

    # Load the rigid cylinder (poking tool)
    cylinder_prim_path = "/World/PokingTool"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_prim_path)
    cylinder.CreateHeightAttr(0.2)
    cylinder.CreateRadiusAttr(0.005)
    cylinder.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.15))
    cylinder.AddRotateYOp().Set(90)  # Orient the cylinder if necessary

    # Apply rigid body physics to the cylinder
    cylinder_prim = stage.GetPrimAtPath(cylinder_prim_path)
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    # Optionally set mass and other properties
    mass_api = UsdPhysics.MassAPI.Apply(cylinder_prim)
    mass_api.CreateMassAttr(0.1)

    return cube_prim, cylinder_prim

if __name__ == "__main__":
    from scene_setup import setup_scene
    simulation_app = omni.isaac.kit.SimulationApp({"headless": False})
    sim_context, stage = setup_scene()
    cube_prim, cylinder_prim = load_objects(stage)

    # Update and run the simulation for a few steps to see the objects
    for _ in range(100):
        sim_context.step()
        simulation_app.update()

    # Close the simulation app if this script is run standalone
    simulation_app.close()
