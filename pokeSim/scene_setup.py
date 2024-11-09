# scene_setup.py

import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core import SimulationContext
from pxr import UsdGeom, Gf

# Initialize the simulation app
simulation_app = SimulationApp({"headless": False})  # Set headless=True if running without GUI

def setup_scene():
    # Create a simulation context
    sim_context = SimulationContext()
    
    # Set up the stage
    stage = omni.usd.get_context().get_stage()
    
    # Set up lighting
    UsdGeom.Xform.Define(stage, "/World/Light")
    distant_light = UsdGeom.DistantLight.Define(stage, "/World/Light/DistantLight")
    distant_light.CreateIntensityAttr(500)
    distant_light.CreateAngleAttr(0.53)
    
    # Set up the ground plane
    plane_prim = UsdGeom.Mesh.Define(stage, "/World/GroundPlane")
    plane_prim.CreatePointsAttr([
        (-10, -10, 0),
        (-10, 10, 0),
        (10, 10, 0),
        (10, -10, 0)
    ])
    plane_prim.CreateExtentAttr([(-10, -10, 0), (10, 10, 0)])
    plane_prim.CreateFaceVertexCountsAttr([4])
    plane_prim.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane_prim.CreateDisplayColorAttr([(0.5, 0.5, 0.5)])
    
    # Set physics parameters if needed
    # ...

    return sim_context, stage

if __name__ == "__main__":
    sim_context, stage = setup_scene()
    simulation_app.update()
    # Close the simulation app if this script is run standalone
    simulation_app.close()
