#!/usr/bin/env python3


import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import franka_gripper.msg 
import actionlib

try:
    from math import pi, tau, dist, fabs, cos
except:  
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class Cycle_action(object):

    def __init__(self):
        super(Cycle_action, self).__init__()

        ##Инициализация основных нод moveit_commander и rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("Controll_panda_bot", anonymous=True)

        ##Инициализация что робот представляет собой кинематическую модель
        robot = moveit_commander.RobotCommander()

        ##Наверное нужно для обновления мира
        scene = moveit_commander.PlanningSceneInterface()

        ## Группа должна точно быть такая же как в moveit
        group_name = "panda_arm" 
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Группа 2 для захвата
        group_name2 = "panda_hand" 
        move_group2 = moveit_commander.MoveGroupCommander(group_name2)

        ##  Создание издателя который публикует траекторию робота         
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            franka_gripper.msg.GraspAction
        )
        planning_frame = move_group.get_planning_frame()
        rospy.logwarn("Reference frame для данного робота")
        print(planning_frame)

        eef_link = move_group.get_end_effector_link()
        rospy.logwarn("Название звена эндефектора")
        print(eef_link)
        print("="*60 )

        group_names = robot.get_group_names()
        rospy.logwarn("Звенья доступные для планирования")
        print(robot.get_group_names())
        print("_"*60 )
        rospy.logwarn("Положение звеньев робота")
        print(robot.get_current_state())
        print("_"*60 )
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group2 = move_group2
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def go2pose1(self):
        ref_pose = [-0.04135178872069645, 0.01914370021458467, -0.7525707168216158,\
                   -1.817295192826208, 0.014709362448320995, 1.8329280274586583,\
                   -0.012179420302052435, 0.021573767801331942, 0.021573767801331942]
        move_group = self.move_group
        rospy.logwarn("Переход в положение 1_1")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go2pose1_1(self):
                  
        ref_pose = [-0.31785746950210836, 0.6837097784355386, -0.42647660662143494,\
                    -2.0815769472842067, 0.5873535560140439, 2.649049841545665,\
                    -0.40115174607225157, 0.02157072440218778, 0.02157072440218778]

        move_group = self.move_group
        rospy.logwarn("Переход в положение 1_1")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    
    def go2pose2_1(self):
        ref_pose = [0.48037182112545196, 0.06765164427359682, 0.34515078853517345,\
                   -1.755724783120959, -0.02495915767168455, 1.8206413883719303,\
                    1.6162684619631165, 0.02147690192928137, 0.02147690192928137]

        move_group = self.move_group
        rospy.logwarn("Переход в положение 2_1")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go2pose2_2(self):
        ref_pose = [0.5519921002538091, 0.615316741744742, 0.20418916044005186,\
             -2.1196291082998346, -0.28636215719706737, 2.7108373639238987,\
                 1.765956624533617, 0.021420678760804664, 0.021420678760804664]

        move_group = self.move_group
        rospy.logwarn("Переход в положение 2_2")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    



    def open_gripper(self):
        rospy.logwarn("Открытие захвата")
        move_group2 = self.move_group2
        joint_goal = move_group2.get_current_joint_values()
        joint_goal[0] = 0.03
        move_group2.go(joint_goal, wait=True)
        move_group2.stop()
        current_joints = move_group2.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def close_gripper(self):
        rospy.logwarn("Закрытие захвата")
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', \
        franka_gripper.msg.GraspAction)
        client.wait_for_server()
        gripper_move = franka_gripper.msg.GraspGoal()
        gripper_move.width = 0.02
        gripper_move.epsilon.inner = 0.008
        gripper_move.epsilon.outer = 0.008
        gripper_move.speed = 0.1
        gripper_move.force = 5
        client.send_goal(gripper_move)
        client.wait_for_result()

    def display_trajectory(self, plan):
        rospy.logwarn("Отображение траектории")
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        rospy.logwarn("Выполнение траектории")
        move_group.execute(plan, wait=True)

def main():
    #Реализация конечного автомата
    time_wait =0
    try:
        
        robot_actions  = Cycle_action()
        input(
            "Для старта симуляции нажмите -> Enter <-"
        )
        robot_actions.open_gripper()
        while not rospy.is_shutdown():
            time.sleep(time_wait) 
            robot_actions.go2pose1()
            time.sleep(time_wait) 
            robot_actions.go2pose1_1()
            time.sleep(time_wait)
            robot_actions.open_gripper()
            time.sleep(time_wait)
            robot_actions.close_gripper()
            time.sleep(time_wait)
            robot_actions.go2pose1()
            time.sleep(time_wait)
            robot_actions.go2pose2_1()
            time.sleep(time_wait)
            robot_actions.go2pose2_2()
            time.sleep(time_wait)
            robot_actions.open_gripper()
            time.sleep(time_wait)
            robot_actions.close_gripper()
            time.sleep(time_wait)
            robot_actions.go2pose2_1()
            time.sleep(time_wait)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return



if __name__ == "__main__":
    main()
