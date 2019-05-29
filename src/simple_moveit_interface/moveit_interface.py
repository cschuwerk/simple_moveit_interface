#!/usr/bin/env python

## @package simple_moveit_interface
#  Demo application script for bin picking demo

## @file
#  moveit_interface class

## @author Clemens Schuwerk
#  clemens.schuwerk@tum.de

import sys
import rospy
import moveit_commander
from tf import TransformListener
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from control_msgs.msg import FollowJointTrajectoryResult
from moveit_msgs.msg import MoveGroupActionResult, ExecuteTrajectoryActionResult
import copy

class moveit_interface_config:
    cfg = {
        "move_group_name": "manipulator",
        "goal_position_tolerance": 0.01,
        "goal_joint_tolerance": 0.01,
        "goal_orientation_tolerance": 0.01,
        "topicMoveGroupResult": "/move_group/result",
        "topicTrajectoryExecutionResult": "/execute_trajectory/result",
    }

    def __init__(self, ns="~"):
        ''' Checks for configuration parameters and writes corresponding values
		    Any of the keys in cfg can be used as a ROS parameter name. '''
        for name in self.cfg.keys():
            pn = ns + name
            if rospy.has_param(pn):
                val = rospy.get_param(pn)
                self.cfg[name] = val
                rospy.loginfo("Load config '%s' from parameter server: %s" % (name, str(self.cfg[name])))
            else:
                rospy.loginfo("Config '%s': %s" % (name, str(self.cfg[name])))


class moveit_interface:

    def __init__(self, config):
        rospy.loginfo("Start to intialize moveit_control_interface")

        # Config
        self.cfg = config.cfg

        # TF
        self.tf = TransformListener()
        
        rospy.Subscriber(self.cfg['topicMoveGroupResult'], MoveGroupActionResult, self.cb_move_group_result)
        rospy.Subscriber(self.cfg['topicTrajectoryExecutionResult'], ExecuteTrajectoryActionResult, self.cb_trajectory_execution_result)

        # Wait for Moveit action server
        moveit_interface_found = False;
        while not moveit_interface_found:
            try:
                self.robot = moveit_commander.RobotCommander()
                self.scene = moveit_commander.PlanningSceneInterface()
                self.group = moveit_commander.MoveGroupCommander(self.cfg['move_group_name'])
                moveit_interface_found = True
            except Exception as e:
                rospy.logerr(e.message)
                moveit_interface_found = False
                rospy.logerr("Retrying to connect to MoveIt action server ...")
                rospy.signal_shutdown('No MoveIt interface found')
                return

        # Create an inverse mapping of the joint names
        self.active_joints = self.group.get_active_joints()
        rospy.loginfo("============ Active joints: %d" % len(self.active_joints))
        self.active_joints_id = {}
        i=0
        for joint_name in self.active_joints:
            self.active_joints_id[i] = joint_name
            i+=1
        rospy.loginfo(self.active_joints_id)

        # Joint/Goal tolerances for motion planning:
        rospy.loginfo("============ Goal tolerances: joint, position, orientation")
        self.group.set_goal_position_tolerance(self.cfg['goal_position_tolerance'])
        self.group.set_goal_joint_tolerance(self.cfg['goal_joint_tolerance'])
        self.group.set_goal_orientation_tolerance(self.cfg['goal_orientation_tolerance'])
        rospy.loginfo("Joint-, position-, orientation tolerance: ")
        rospy.loginfo(self.group.get_goal_tolerance()) # Return a tuple of goal tolerances: joint, position and orientation.

        rospy.loginfo("============ Reference frame for poses of end-effector")
        rospy.loginfo(self.group.get_pose_reference_frame())

    ## Init the MoveIt planning scene
    def init_planning_scene(self):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.
        p.pose.position.y = 0.
        p.pose.position.z = 0.
        self.scene.add_box("table", p, (0.5, 1.5, 0.6))

    ## Execute the plan passed as parameter or, if the passed plan is None, execute the most recently planned trajectory.
    def execute(self,plan=None,wait=True):

        if plan is None:
            self.group.set_start_state_to_current_state()
            plan = self.group.plan()
        if len(plan.joint_trajectory.points) is 0:
            rospy.logerr("Empty trajectory, nothing to execute! Probably IK did not find a solution.")
            return
        self.group.execute(plan, wait)
        #self.group.go(wait=True)

    ## Change the current pose in the direction of the passed axis by the passed value.
    # This is a helper function for the move_x/y/z and rotate_y/y/z methods.
    # @param axis The axis to change (integer from 0 to 5 corresponding to the translation/rotation axis)
    # @param value The value to be added to the current pose value.
    def _shift_pose_target(self, axis, value, wait=True):
        try:
            self.group.shift_pose_target(axis,value)
            self.execute(wait=wait)
        except Exception as e:
            rospy.logerr(e.message)
        return

    ## Move to a new 6D endeffector pose (geometry_msgs/Pose)
    def move_pose(self, pose, wait=True):
        rospy.loginfo("Received pose command")
        try:
            self.group.set_pose_target(pose)
            self.execute(wait=wait)
        except Exception as e:
            rospy.logerr(e.message)
        return

    ## Move to a new 3D endeffector position (geometry_msgs/Point)
    def move_position(self, position, wait=True):
        try:
            rospy.loginfo("Received position command")
            self.group.set_position_target([position.x, position.y, position.z])
            self.execute(wait=wait)
        except Exception as e:
            rospy.logerr(e.message)
        return

    ## Move to a named (and pre-defined) fixed pose
    def move_fixed_pose(self, fixed_pose_name, wait=True):
        rospy.loginfo("Received fixed pose command: %s" % fixed_pose_name)
        try:
            self.group.set_named_target(fixed_pose_name)
            self.execute(wait=wait)
        except Exception as e:
            rospy.logerr(e.message)
        return

    ## Move to a given joint state
    def move_joint_state(self, joint_state, wait=True):
        rospy.loginfo("Received joint state command")
        desired_joint_pos = None

        if len(joint_state.name) is not 0 and len(joint_state.name) is len(joint_state.position):
            desired_joint_pos = dict(zip(joint_state.name,joint_state.position))
        elif len(joint_state.name) is 0 and len(joint_state.position) is len(self.active_joints):
            desired_joint_pos = joint_state.position
        else:
            rospy.logerr("Invalid joint state message received! %s" % joint_state)

        if desired_joint_pos is not None:
            try:
                self.group.set_joint_value_target(desired_joint_pos)
                self.execute(wait=wait)
            except Exception as e:
                rospy.logerr(e.message)
        return

    ## Move the endeffector relative in the x-direction
    def move_x(self, move_by_value, wait=True):
        rospy.loginfo("Received move x command: %f" % move_by_value)
        self._shift_pose_target(0, move_by_value, wait)
        return

    ## Move the endeffector relative in the y-direction
    def move_y(self, move_by_value, wait=True):
        rospy.loginfo("Received move y command: %f" % move_by_value)
        self._shift_pose_target(1, move_by_value, wait)
        return

    ## Move the endeffector relative in the z-direction
    def move_z(self, move_by_value, wait=True):
        rospy.loginfo("Received move z command: %f" % move_by_value)
        self._shift_pose_target(2, move_by_value, wait)
        return

    ## Callback function for a relative rotation of the endeffector around the x-axis
    def rotate_x(self, rotate_by_value):
        rospy.loginfo("Received rotate x command: %f" % rotate_by_value)
        self._shift_pose_target(3, rotate_by_value, wait)
        return

    ## Callback function for a relative rotation of the endeffector around the y-axis
    def rotate_y(self, rotate_by_value, wait=True):
        rospy.loginfo("Received rotate y command: %f" % rotate_by_value)
        self._shift_pose_target(4, rotate_by_value, wait)
        return

    ## Callback function for a relative rotation of the endeffector around the z-axis
    def rotate_z(self, rotate_by_value, wait=True):
        rospy.loginfo("Received rotate z command: %f" % rotate_by_value)
        self._shift_pose_target(5, rotate_by_value, wait)
        return

    ## Callback function to move the endeffector to a named TF pose
    def move_to_tf_pose(self, tf_frame_name, wait=True):
        rospy.loginfo("Received move to TF pose command: %s" % tf_frame_name.data)
        pose_reference_frame = self.group.get_pose_reference_frame()

        try:
            t = self.tf.getLatestCommonTime(pose_reference_frame, tf_frame_name.data)
            pos, rot = self.tf.lookupTransform(pose_reference_frame, tf_frame_name.data, t)
            self.group.set_pose_target(pos+rot)
            self.execute(wait=wait)
        except Exception as e:
            rospy.logerr(e.message)
        return

    ## Callback function to move the endeffector to the position of a named TF pose
    def move_to_tf_position(self, tf_frame_name, wait=True):
        rospy.loginfo("Received move to TF position command: %s" % tf_frame_name.data)
        pose_reference_frame = self.group.get_pose_reference_frame()

        try:
            t = self.tf.getLatestCommonTime(pose_reference_frame, tf_frame_name.data)
            pos, rot = self.tf.lookupTransform(pose_reference_frame, tf_frame_name.data, t)
            self.group.set_position_target(pos)
            self.execute(wait=wait)
        except Exception as e:
            rospy.logerr(e.message)
        return

    ## Callback function to move the endeffector on a cartesian path (straight line) to the goal pose
    def move_cartesian_path_to_pose(self, pose, wait=True):

        try:
            rospy.loginfo("Received move pose with cartesian path")

            waypoints = []
            current_pose = self.group.get_current_pose()
            waypoints.append(copy.deepcopy(current_pose.pose))
            waypoints.append(copy.deepcopy(pose))

            # waypoints to follow, eef_step, jump_threshold
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01,0.0)
            self.execute(plan,wait)
        except Exception as e:
            rospy.logerr(e.message)
        return
        
        
    def get_current_pose(self):
        try:
            return self.group.get_current_pose().pose
        except Exception as e:
            rospy.logerr(e.message)
        return None

    ## Callback function for move group planning action result feedback
    def cb_move_group_result(self, result):
        rospy.loginfo("Move group: %s (code %d)" % (result.status.text, result.status.status))

    ## Callback function for trajectory execution action result feedback
    def cb_trajectory_execution_result(self, result):
        rospy.loginfo("Trajectory execution: %s (code %d)" % (result.status.text, result.status.status))
