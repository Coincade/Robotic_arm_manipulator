import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import numpy as np

def move_robot():
    arduinobot = MoveItPy(node_name='moveit_py')
    arduinobot_arm = arduinobot.get_planning_component("arm")
    arduinobot_gripper = arduinobot.get_planning_component("gripper")

    # Getting current state of the robot
    arm_state = RobotState(arduinobot.get_robot_model())
    gripper_state = RobotState(arduinobot.get_robot_model())

    arm_state.set_joint_group_positions("arm", np.array([1.57,0.0,0.0])) # rotate the base joint 90 degrees
    gripper_state.set_joint_group_positions("gripper", np.array([-0.7, 0.7])) # open the gripper

    #setting the state of the robot to current state
    arduinobot_arm.set_start_state_to_current_state()
    arduinobot_gripper.set_start_state_to_current_state()

    #setting the goal state of the robot
    arduinobot_arm.set_goal_state(robot_state=arm_state)
    arduinobot_gripper.set_goal_state(robot_state=gripper_state)

    #planning the trajectory
    arm_plan_result = arduinobot_arm.plan()
    gripper_plan_result = arduinobot_gripper.plan()

    #executing the trajectory
    if arm_plan_result and gripper_plan_result:
        arduinobot.execute(arm_plan_result.trajectory, controllers=[])
        arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("One or more trajectories failed to plan")
    
    
    
def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()

if __name__ == '__main__':
    main()