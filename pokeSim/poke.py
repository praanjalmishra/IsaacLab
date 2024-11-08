# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to spawn a deformable cube and a thin cylinder into the scene.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/tutorials/01_assets/spawn_deformable_and_cylinder.py
"""

"""Launch Isaac Sim Simulator first."""

import argparse
from omni.isaac.lab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(description="Spawn a deformable cube and a thin cylinder.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import DeformableObject, DeformableObjectCfg
from omni.isaac.core.objects import DynamicCylinder
from omni.isaac.lab.sim import SimulationContext
#from omni.isaac.core.prims import MassPropertiesCfg 

import torch

def design_scene():
    """Design the scene with a deformable cube and a thin cylinder."""
    # Ground plane
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)

    # Lighting
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    # Deformable Cube
    deformable_cfg = DeformableObjectCfg(
        prim_path="/World/DeformableCube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.2, 0.2, 0.2),
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1e5),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.1)),
        debug_vis=True,
    )
    deformable_cube = DeformableObject(cfg=deformable_cfg)

    # Thin Cylinder
    cylinder_cfg = {
        "prim_path": "/World/ThinCylinder",
        "position": (0.0, 0.0, 3),
        "radius": 0.01,
        "height": 0.4,
        "color": torch.tensor([0.0, 0.0, 1.0]),
        "mass": 1.0,
    }
    thin_cylinder = DynamicCylinder(**cylinder_cfg)

    # Return the scene entities
    scene_entities = {"deformable_cube": deformable_cube, "thin_cylinder": thin_cylinder}
    return scene_entities

def main():
    """Main function to set up and visualize the scene."""
    # Load simulation configuration
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([1.5, 0.0, 1.0], [0.0, 0.0, 0.5])

    # Design the scene
    scene_entities = design_scene()

    # Reset simulation to apply changes
    sim.reset()
    print("[INFO]: Setup complete. Deformable cube and thin cylinder have been spawned in the scene.")

    # Run simulation without interaction
    while simulation_app.is_running():
        sim.step()

if __name__ == "__main__":
    # Run the main function
    main()
    # Close simulation application
    simulation_app.close()
