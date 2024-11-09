# object_loader.py

from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni
import omni.physx

def load_objects(stage):
    # Ensure PhysX is enabled for deformable simulation
    physx_interface = omni.physx.acquire_physx_interface()
    physx_interface.create_physx_scene()

    # Create a deformable cube
    cube_path = "/World/DeformableCube"
    cube_size = 0.1  # 10 cm cube
    cube = UsdGeom.Mesh.Define(stage, cube_path)
    cube.CreatePointsAttr([
        (-cube_size, -cube_size, -cube_size),
        (-cube_size, -cube_size, cube_size),
        (-cube_size, cube_size, -cube_size),
        (-cube_size, cube_size, cube_size),
        (cube_size, -cube_size, -cube_size),
        (cube_size, -cube_size, cube_size),
        (cube_size, cube_size, -cube_size),
        (cube_size, cube_size, cube_size),
    ])
    # Define faces, normals, and other attributes as needed
    # ...

    # Set deformable physics properties
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    PhysxSchema.PhysxDeformableAPI.Apply(cube.GetPrim())
    deformable_api = PhysxSchema.PhysxDeformableAPI(cube.GetPrim())
    deformable_api.CreateDeformableRestOffsetAttr(0.001)
    deformable_api.CreateDeformableDampingAttr(0.1)
    deformable_api.CreateDeformableSolverPositionIterationCountAttr(30)
    deformable_api.CreateDeformableSolverVelocityIterationCountAttr(0)
    deformable_api.CreateSelfCollisionAttr(True)
    # Set material properties
    # ...

    # Position the cube
    cube.GetPrim().SetTranslate((0, 0, cube_size + 0.01))  # Slightly above the ground

    # Create a rigid cylinder (poking tool)
    cylinder_path = "/World/PokingTool"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.CreateHeightAttr(0.2)
    cylinder.CreateRadiusAttr(0.01)
    cylinder.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.2))
    # Apply rigid body physics
    UsdPhysics.RigidBodyAPI.Apply(cylinder.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    # Set mass and other physical properties if needed
    # ...

    return cube, cylinder

if __name__ == "__main__":
    from scene_setup import setup_scene
    sim_context, stage = setup_scene()
    load_objects(stage)
    # Update and run the simulation for a few steps to see the objects
    for _ in range(100):
        sim_context.step()
    simulation_app.update()
    # Close the simulation app if this script is run standalone
    simulation_app.close()
