import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # TODO: change the follow paths
        self.rmpflow = mg.lula.motion_policies.RmpFlow(robot_description_path="/home/skyentific/Documents/YellowArm/Motion/rmpflow/robot_descriptor.yaml",
                                                        rmpflow_config_path="/home/skyentific/Documents/YellowArm/Motion/rmpflow/skyentific_rmpflow_common.yaml",
                                                        urdf_path="/home/skyentific/Documents/YellowArm/URDF/yellowarm_description/robots/yellow_arm.urdf",
                                                        end_effector_frame_name="yellowarm_link7",
                                                        maximum_substep_size=0.00334)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
