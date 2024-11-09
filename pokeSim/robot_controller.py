# robot_controller.py

from pxr import Gf
import time

def perform_poking_action(sim_context, cylinder_prim, cube_prim):
    # Define poking motion parameters
    initial_position = Gf.Vec3f(0, 0, 0.2)
    poke_depth = 0.05  # How deep to poke into the cube
    poke_speed = 0.01  # Units per simulation step

    # Move the cylinder towards the cube
    current_position = initial_position
    target_position = Gf.Vec3f(0, 0, 0.1)  # Adjust based on cube position

    # Calculate the number of steps needed
    distance = current_position[2] - target_position[2]
    steps = int(distance / poke_speed)

    for _ in range(steps):
        current_position[2] -= poke_speed
        cylinder_prim.GetAttribute("xformOp:translate").Set(current_position)
        sim_context.step()
        simulation_app.update()

    # Hold the position for a few steps to simulate contact
    for _ in range(50):
        sim_context.step()
        simulation_app.update()

    # Retract the cylinder
    for _ in range(steps):
        current_position[2] += poke_speed
        cylinder_prim.GetAttribute("xformOp:translate").Set(current_position)
        sim_context.step()
        simulation_app.update()

    # Finalize simulation
    for _ in range(100):
        sim_context.step()
        simulation_app.update()

if __name__ == "__main__":
    from scene_setup import setup_scene
    from object_loader import load_objects

    sim_context, stage = setup_scene()
    cube, cylinder = load_objects(stage)
    cube_prim = cube.GetPrim()
    cylinder_prim = cylinder.GetPrim()

    # Perform the poking action
    perform_poking_action(sim_context, cylinder_prim, cube_prim)

    # Close the simulation app if this script is run standalone
    simulation_app.close()
