#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

import tf

from object_msgs.msg import Object


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('go_to_pose', anonymous=True)

        self._planning_group = "ur5_arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class Ur5Moveit2:

    # Constructor
    def __init__(self):

        # rospy.init_node('follow_joints2', anonymous=True)

        self._planning_group = "ur5_hand_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    ur5_2 = Ur5Moveit2()
    listener = tf.TransformListener()


    # joint angles for taking the picture
    picture_2 = [0.132353824652, 0.0432870992458, -0.0255333584981, -0.541252398569, 0.132409681033, 5.89499764159e-05]

    coke_grip = [0.25]
    coke_drop = [0.0]
    battery_grip = [0.35]
    battery_drop = [0.0]
    glue_grip = [0.36]
    glue_drop = [0.0]
    pre_drop = [-0.334794541127, 0.558720221856, -1.26614329414, 0.459547751942, -0.467115319843, 0.15246066248]
    # distance between wrist_3_link and gripper = 0.239m

    # pose of wrist_3 joint
    # - Translation: [0.257, 0.263, 0.794]
    # - Rotation: in Quaternion [0.083, 0.996, -0.008, -0.011]
    #         in RPY (radian) [-3.125, -0.022, 2.976]
    #         in RPY (degree) [-179.031, -1.238, 170.523]

    # change only x, y, yaw

    grip_dis_coke = 0.250
    grip_dis_glue = 0.210
    grip_dis_battery = 0.190
    fixed_height = 0.8

    ur5.set_joint_angles(picture_2)

    coke = ''
    battery = ''
    glue = ''

    while(battery == '' or glue == '' or coke == ''):
        if(listener.frameExists('object_28')):
            coke = 'object_28'
        elif(listener.frameExists('object_29')):
            coke = 'object_29'
        elif(listener.frameExists('object_30')):
            coke = 'object_30'
        elif(listener.frameExists('object_31')):
            coke = 'object_31'

        if(listener.frameExists('object_33')):
            battery = 'object_33'
        elif(listener.frameExists('object_36')):
            battery = 'object_36'

        if(listener.frameExists('object_32')):
            glue = 'object_32'
        elif(listener.frameExists('object_34')):
            glue = 'object_34'
        elif(listener.frameExists('object_35')):
            glue = 'object_35'



    try:
        (coke_trans, coke_rot) = listener.lookupTransform('/ebot_base', coke, rospy.Time(0))
        (battery_trans, battery_rot) = listener.lookupTransform('/ebot_base', battery, rospy.Time(0))
        (glue_trans, glue_rot) = listener.lookupTransform('/ebot_base', glue, rospy.Time(0))
        
        # base_link position wrt ebot_base is [0, 0, 0.521]

        coke_pose = geometry_msgs.msg.Pose()
        if(coke_trans[0]>0.378):
            coke_pose.position.x = 0.447 - 0.004
        elif(coke_trans[0]<0.378 and coke_trans[0]>0.2435):
            coke_pose.position.x = 0.309 - 0.003
        elif(coke_trans[0]<0.2435):
            coke_pose.position.x = 0.178 - 0.004
        coke_pose.position.y = coke_trans[1] - 0.19
        coke_pose.position.z = 1.0
        coke_pose.orientation.x = 0
        coke_pose.orientation.y = 1
        coke_pose.orientation.z = 0
        coke_pose.orientation.w = 0
        ur5.go_to_pose(coke_pose)
        coke_pose.position.z = 0.9
        coke_pose.orientation.x = 0.005
        coke_pose.orientation.y = 0.971
        coke_pose.orientation.z = -0.239
        coke_pose.orientation.w = -0.005            
        ur5.go_to_pose(coke_pose)
        ur5_2.set_joint_angles(coke_grip)
        coke_pose.position.z = 1.1
        coke_pose.orientation.x = 0
        coke_pose.orientation.y = 1
        coke_pose.orientation.z = 0
        coke_pose.orientation.w = 0
        ur5.go_to_pose(coke_pose)
        ur5.set_joint_angles(pre_drop)
        ur5_2.set_joint_angles(coke_drop)
        ur5.set_joint_angles(picture_2)

        battery_pose = geometry_msgs.msg.Pose()
        battery_pose.position.x = battery_trans[0]
        battery_pose.position.y = battery_trans[1] - 0.19
        battery_pose.position.z = 1.0
        battery_pose.orientation.x = 0
        battery_pose.orientation.y = 1
        battery_pose.orientation.z = 0
        battery_pose.orientation.w = 0
        ur5.go_to_pose(battery_pose)
        battery_pose.position.z = 0.9
        battery_pose.orientation.x = 0.005
        battery_pose.orientation.y = 0.971
        battery_pose.orientation.z = -0.239
        battery_pose.orientation.w = -0.005
        ur5.go_to_pose(battery_pose)
        ur5_2.set_joint_angles(battery_grip)
        battery_pose.position.z = 1.0
        battery_pose.orientation.x = 0
        battery_pose.orientation.y = 1
        battery_pose.orientation.z = 0
        battery_pose.orientation.w = 0
        ur5.go_to_pose(battery_pose)
        ur5.set_joint_angles(pre_drop)
        ur5_2.set_joint_angles(battery_drop)
        ur5.set_joint_angles(picture_2)

        glue_pose = geometry_msgs.msg.Pose()
        glue_pose.position.x = glue_trans[0] - 0.02
        glue_pose.position.y = glue_trans[1] - 0.19
        glue_pose.position.z = 1.0
        glue_pose.orientation.x = 0
        glue_pose.orientation.y = 1
        glue_pose.orientation.z = 0
        glue_pose.orientation.w = 0
        ur5.go_to_pose(glue_pose)
        glue_pose.position.z = 0.9
        glue_pose.orientation.x = 0.005
        glue_pose.orientation.y = 0.971
        glue_pose.orientation.z = -0.239
        glue_pose.orientation.w = -0.005
        ur5.go_to_pose(glue_pose)
        ur5_2.set_joint_angles(glue_grip)
        ur5.set_joint_angles(pre_drop)
        ur5_2.set_joint_angles(glue_drop)

        coke_detected.origin.position.x = coke_pose.position.x
        coke_detected.origin.position.y = coke_trans[1] + 0.03
        coke_detected.origin.position.z = 0.214
        coke_detected.origin.orientation.x = 0
        coke_detected.origin.orientation.y = 0
        coke_detected.origin.orientation.z = 0
        coke_detected.origin.orientation.w = 0
        pub.publish(coke_detected)

        battery_detected.origin.position.x = battery_trans[0]
        battery_detected.origin.position.y = battery_trans[1] + 0.03
        battery_detected.origin.position.z = 0.214
        battery_detected.origin.orientation.x = 0
        battery_detected.origin.orientation.y = 0
        battery_detected.origin.orientation.z = 0
        battery_detected.origin.orientation.w = 0
        pub.publish(battery_detected)

        glue_detected.origin.position.x = glue_pose.position.x
        glue_detected.origin.position.y = glue_trans[1] + 0.03
        glue_detected.origin.position.z = 0.214
        glue_detected.origin.orientation.x = 0
        glue_detected.origin.orientation.y = 0
        glue_detected.origin.orientation.z = 0
        glue_detected.origin.orientation.w = 0
        pub.publish(glue_detected)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.spin()

    rospy.spin()
    moveit_commander.roscpp_shutdown()

    del ur5
    del ur5_2


if __name__ == '__main__':
    pub = rospy.Publisher('/detection_info', Object, queue_size=10)
    coke_detected = Object()
    coke_detected.name = 'Coke Can'
    battery_detected = Object()
    battery_detected.name = 'Battery'
    glue_detected = Object()
    glue_detected.name = 'Glue'

    main()
