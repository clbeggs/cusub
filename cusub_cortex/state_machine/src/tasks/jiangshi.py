#!/usr/bin/env python
from __future__ import division
"""
Jiangshi Buoy Task, attempts to slay Jiangshi
Objectives:
- Search
- Slay
---> Go to approach point
---> Slay vampire
---> Backup
"""
from tasks.task import Task, Objective
from tasks.search import Search
import tf
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import smach
import smach_ros
from sensor_msgs.msg import Imu
import copy
from perception_control.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback
import actionlib

class Jiangshi(Task):
    name = "jiangshi"

    def __init__(self):
        super(Jiangshi, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search(self.get_prior_param(), "cusub_cortex/mapper_out/jiangshi")
        self.slay = Slay()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Slay', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Slay', self.slay, transitions={'success':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Slay(Objective):
    """
    Go to a point in front of the bouy, slay jiangshi backwards, backup
    """
    outcomes=['success', 'timed_out']

    def __init__(self):
        super(Slay, self).__init__(self.outcomes, "Slay")
        self.jiangshi_pose = None
        rospy.Subscriber("cusub_cortex/mapper_out/jiangshi", PoseStamped, self.jiangshi_callback)
        self.approach_dist = rospy.get_param('tasks/jiangshi/approach_dist', 2.0)
        self.print_configuration()

        # APPROACH: SOLVEPNP
        self.replan_threshold = rospy.get_param('tasks/jiangshi/solvepnp_approach/replan_threshold', 0.5)
        self.use_buoys_yaw = rospy.get_param('tasks/jiangshi/solvepnp_approach/use_buoys_yaw', True)

        # SLAY: SETPOINT
        self.slay_dist = rospy.get_param('tasks/jiangshi/setpoint_slay/slay_dist', 0.5) 

        # APPROACH/SLAY VISUAL SERVO
        self.feedback = None
        self.target_pixel_box = rospy.get_param("tasks/jiangshi/visual_servo/target_pixel_threshold")
        self.vs_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        rospy.loginfo("...waiting for visual_servo server")
        self.vs_client.wait_for_server()
        rospy.loginfo("\tfound visual servo server")

        # IMU
        self.monitor_imu = False
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)
        self.imu_axis = rospy.get_param("tasks/jiangshi/imu_axis")
        self.jump_thresh = rospy.get_param("tasks/jiangshi/jump_thresh")

    def print_configuration(self):
        if rospy.get_param("tasks/jiangshi/method") == "solvepnp":
            rospy.loginfo("...method: SOLVEPNP")
        else:
            rospy.loginfo("...method: VISUAL_SERVO")

    def imu_callback(self, msg):
        if ( self.monitor_imu == False ) or self.replan_requested():
            return

        hit_detected = False

        if self.imu_axis == 'x':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.x < self.jump_thresh ):
                hit_detected = True
                self.request_replan()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.x > self.jump_thresh ):
                hit_detected = True
                self.request_replan()
        elif self.imu_axis == 'y':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.y < self.jump_thresh ):
                hit_detected = True
                self.request_replan()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.y > self.jump_thresh ):
                hit_detected = True
                self.request_replan()
        elif self.imu_axis == 'z':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.z < self.jump_thresh ):
                hit_detected = True
                self.request_replan()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.z > self.jump_thresh ):
                hit_detected = True
                self.request_replan()
        else:
            rospy.logerr("Unrecognized imu axis: " + str(self.imu_axis))

        if hit_detected:
            rospy.loginfo_throttle(1, "Detected a buoy hit!")

    def jiangshi_callback(self, msg):
        # Set the first pose and don't replan
        if self.jiangshi_pose == None:
            self.jiangshi_pose = msg
            return
        change_in_pose = self.get_distance_xy(self.jiangshi_pose.pose.position, msg.pose.position)
        if (change_in_pose > self.replan_threshold) and not self.replan_requested():
            self.jiangshi_pose = msg
            self.request_replan()

    def vs_feedback_callback(self, feedback):
        self.feedback = feedback.centered
    
    def get_approach_pose(self):
        approach_pose = Pose()
        if self.use_buoys_yaw:    
            quat = [self.jiangshi_pose.pose.orientation.x, self.jiangshi_pose.pose.orientation.y,self.jiangshi_pose.pose.orientation.z,self.jiangshi_pose.pose.orientation.w]
            jiangshi_roll, jiangshi_pitch, jiangshi_yaw = tf.transformations.euler_from_quaternion(quat)
            approach_pose.position.x = self.jiangshi_pose.pose.position.x + self.approach_dist * np.cos(jiangshi_yaw)
            approach_pose.position.y = self.jiangshi_pose.pose.position.y + self.approach_dist * np.sin(jiangshi_yaw)
            approach_pose.position.z = self.jiangshi_pose.pose.position.z
            goal_quat = tf.transformations.quaternion_from_euler(jiangshi_roll, jiangshi_pitch, jiangshi_yaw)
            approach_pose.orientation.x = goal_quat[0]
            approach_pose.orientation.y = goal_quat[1]
            approach_pose.orientation.z = goal_quat[2]
            approach_pose.orientation.w = goal_quat[3]
        else: # just draw a line from sub to buoy and hit it
            approach_pose = self.get_pose_between(self.cur_pose, self.jiangshi_pose.pose, self.approach_dist) # inherited
        return approach_pose

    def get_slay_pose(self):
        """
        Calculate slay pose for the buoy setpoint method

        Parameters
        ----------
        self.jiangshi_pose : PoseStamped
            Jiangshi's pose
        self.use_buoys_yaw : bool
            Whether to calculate slay path and approach pose based on the buoys yaw
        self.slay_dist : float
            Meters to go behind jiangshi in order to slay him

        Returns
        -------
        slay_pose : Pose
        """
        slay_pose = Pose()
        if self.use_buoys_yaw:    
            quat = [self.jiangshi_pose.pose.orientation.x, self.jiangshi_pose.pose.orientation.y,self.jiangshi_pose.pose.orientation.z,self.jiangshi_pose.pose.orientation.w]
            jiangshi_roll, jiangshi_pitch, jiangshi_yaw = tf.transformations.euler_from_quaternion(quat)
            slay_pose.position.x = self.jiangshi_pose.pose.position.x - self.slay_dist * np.cos(jiangshi_yaw)
            slay_pose.position.y = self.jiangshi_pose.pose.position.y - self.slay_dist * np.sin(jiangshi_yaw)
            slay_pose.position.z = self.jiangshi_pose.pose.position.z
            slay_pose.orientation = approach_pose.orientation
        else: # just draw a line from sub to buoy and hit it
            slay_pose = self.get_pose_behind(self.cur_pose, self.jiangshi_pose.pose, self.slay_dist)
        return slay_pose
    
    def solvepnp_method(self, userdata):
        rospy.loginfo("...using solvepnp")
        self.configure_darknet_cameras([1,1,0,0,1,0])

        # Approach 
        approach_pose = self.get_approach_pose() 
        while not rospy.is_shutdown():          # Loop until we find a good task pose
            approach_pose = self.get_approach_pose() #only use the approach part
            if self.go_to_pose(approach_pose, userdata.timeout_obj):
                if userdata.timeout_obj.timed_out:
                    userdata.outcome = "timed_out"
                    return "timed_out"
                else: # Loop, replan_requested() happened
                    pass
            else: # we've reached our pose!
                break
        
        # Slay
        if rospy.get_param("tasks/jiangshi/slay_timeout"):
            userdata.timeout_obj.set_new_time(rospy.get_param("tasks/jiangshi/slay_timeout"))
        rospy.loginfo("...slaying jiangshi")
        slay_pose = self.get_slay_pose()
        self.monitor_imu = True
        self.go_to_pose(slay_pose, userdata.timeout_obj)
        rospy.loginfo("\tslayed")
        self.monitor_imu = False

        # backup to our previous pose
        userdata.timeout_obj.set_new_time(0)
        userdata.timeout_obj.timed_out = False
        rospy.loginfo("...disabling timeouts while backing up from slay")
        self.go_to_pose(approach_pose, userdata.timeout_obj, replan_enabled=False, move_mode="backup")
        userdata.outcome = "success"
        return "success"

    def visual_servo_method(self, userdata):
        rospy.loginfo("...using visual servoing approach")
        goal = VisualServoGoal()
        goal.target_class = "vampire_cute"
        goal.camera = goal.OCCAM
        goal.x_axis = goal.YAW_AXIS
        goal.y_axis = goal.DEPTH_AXIS
        goal.area_axis = goal.DRIVE_AXIS
        goal.target_frame = rospy.get_param("~robotname") +"/description/occam0_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        goal.target_box_area = goal.FIFTY_PERCENT_IMAGE
        goal.target_pixel_threshold = self.target_pixel_box
        rospy.loginfo("...centering")
        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)

        while not rospy.is_shutdown() :
            if userdata.timeout_obj.timed_out:
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            if self.feedback:
                break
            rospy.sleep(0.25)
        self.vs_client.cancel_goal()
        rospy.loginfo("\tcentered")
        self.vs_client.cancel_goal()
        
        # rospy.loginfo("...backing up")
        # userdata.timeout_obj.set_new_time(0)
        # userdata.timeout_obj.timed_out = False
        # rospy.loginfo("...disabling timeouts while backing up from slay")
        # self.vs_client.cancel_goal()

        userdata.outcome = "success"
        return "success"


    # def approach(self, userdata):
    #     # VISUAL SERVO METHOD
    #     else:
    #         
        
    #     return "lead free solder" # any str that's not "done" will indicate success here

    # def slay(self, userdata):
    #     rospy.loginfo("...slaying buoy")
    #     approach_pose = copy.deepcopy(self.cur_pose) # don't want this value to be changed as self.cur_pose gets updated
    #     if rospy.get_param("tasks/jiangshi/slay_timeout"):
    #         userdata.timeout_obj.set_new_time(rospy.get_param("tasks/jiangshi/slay_timeout"))
    #     if rospy.get_param("tasks/jiangshi/slay_method") == "setpoint":
    #         rospy.loginfo("...using setpoint slay")
    #         slay_pose = self.get_slay_pose() #only use the slay par
    #         self.monitor_imu = True
    #         self.go_to_pose(slay_pose, userdata.timeout_obj)
    #         self.monitor_imu = False
    #     else:
    #         rospy.loginfo("...using visual servoing slay")
    #         goal = VisualServoGoal()
    #         goal.target_class = "vampire_cute"
    #         goal.camera = goal.OCCAM
    #         goal.x_axis = goal.YAW_AXIS
    #         goal.y_axis = goal.DEPTH_AXIS
    #         goal.area_axis = goal.DRIVE_AXIS
    #         goal.target_frame = rospy.get_param("~robotname") +"/description/occam0_frame_optical"
    #         goal.visual_servo_type = goal.PROPORTIONAL
    #         goal.target_pixel_x = goal.CAMERAS_CENTER_X
    #         goal.target_pixel_y = goal.CAMERAS_CENTER_Y
    #         goal.target_box_area = goal.ONE_HUNDRED_PERCENT_IMAGE
    #         goal.target_pixel_threshold = self.target_pixel_box
    #         rospy.loginfo("...centering")
    #         self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)
 
    #         while not rospy.is_shutdown() :
    #             if userdata.timeout_obj.timed_out:
    #                 self.vs_client.cancel_goal()
    #                 userdata.outcome = "timed_out"
    #                 return "done"
    #             if self.feedback:
    #                 break
    #             rospy.sleep(0.25)
    #         self.vs_client.cancel_goal()
    #         rospy.loginfo("\tcentered")

        

    def execute(self, userdata):
        if rospy.get_param("tasks/jiangshi/method") == "vs": # Do visual servo
            rospy.loginfo("...enacting visual servoing")
            return self.visual_servo_method(userdata)
        else:
            rospy.loginfo("...using solvepnp")
            return self.solvepnp_method(userdata)

# WE SHOULD REHERSE OUR RUN ON ONE GREAT SIMULATOR, EVERYONE LAUNCHING EVERYTHING INDIVIDUALLY


"""
Exit cases for the VS is timeout OR success from the visual servoing b/c we reached our target box size
OR
setpoint exit case is reaching the setpoint OR timeout
"""