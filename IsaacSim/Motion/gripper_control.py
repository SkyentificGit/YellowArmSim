from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

my_world = World(stage_units_in_meters=1.0)
#TODO: change this to your own path
asset_path = "/home/skyentific/Documents/YellowArm/URDF/yellowarm_description/robots/yellow_arm/yellow_arm.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/yellowarm")
#define the gripper
gripper = ParallelGripper(
    #We chose the following values while inspecting the articulation
    end_effector_prim_path="/World/yellowarm/yellowarm_link7",
    joint_prim_names=["yellowarm_joint8", "yellowarm_joint9"],
    joint_opened_positions=np.array([0.022, 0.022]),
    joint_closed_positions=np.array([-0.015, -0.015]),
    action_deltas=np.array([0.037, 0.037]),
)
#define the manipulator
my_yellowarm = my_world.scene.add(SingleManipulator(prim_path="/World/yellowarm", name="yellowarm_robot",
                                                end_effector_prim_path="/World/yellowarm/yellowarm_link7",
                                                gripper=gripper))
#set the default positions of the other gripper joints to be opened so
#that its out of the way of the joints we want to control when gripping an object for instance.
joints_default_positions = np.zeros(9)
joints_default_positions[7] = -0.022
joints_default_positions[8] =  0.022
my_yellowarm.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
        gripper_positions = my_yellowarm.gripper.get_joint_positions()
        if i < 37*8:
            #close the gripper slowly
            my_yellowarm.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.001/8, gripper_positions[1] + 0.001/8]))
        if i > 37*8:
            #open the gripper slowly
            my_yellowarm.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.001/8, gripper_positions[1] - 0.001/8]))
        if i == 37*8*2:
            i = 0

simulation_app.close()