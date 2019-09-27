#!/usr/bin/env python

## @package simple_moveit_interface
#  Moveit (MoveIt commander) wrapper class
#  infos: http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
# http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
## @file
#  moveit_interface class

## @author Clemens Schuwerk
#  clemens.schuwerk@tum.de

import sys
import rospy
import moveit_commander
from tf import TransformListener, TransformBroadcaster
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from control_msgs.msg import FollowJointTrajectoryResult
from moveit_msgs.msg import MoveGroupActionResult, ExecuteTrajectoryActionResult
import copy
from geometry_msgs.msg import TransformStamped

class moveit_interface_config:
    cfg = {
        "move_group_name": "manipulator",
        "goal_position_tolerance": 0.005,
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
        self.br = TransformBroadcaster()


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


    ## Publish a goal to TF
    def _publish_tf(self, poseStamped, name="moveit_target"):

        transform = TransformStamped()
        transform.header = poseStamped.header
        transform.transform.translation = poseStamped.pose.position
        transform.transform.rotation = poseStamped.pose.orientation
        transform.child_frame_id = name
        self.br.sendTransformMessage(transform)

    ## Init the MoveIt planning scene
    # Important: the moveit_commander needs some time to come up!
    def init_planning_scene(self, remove_objects=False, addGround=False):
        rospy.loginfo("Initializing the planning scene")
        rospy.sleep(2.0)
        if remove_objects:
            rospy.loginfo("Removing all objects from the planning scene")
            self.scene.remove_world_object()
        if addGround:
            if not self.add_ground("ground", 0.0):
                rospy.logerr("Ground was not added to the scene")


    def _add_object_decorator(func):
            def wrapper(*args, **kwargs):
                self = args[0]
                rospy.loginfo("Adding object '" + args[1] + "' to the scene.")
                func(*args, **kwargs)
                res = self.wait_for_object(args[1])
                rospy.loginfo("Known objects in the scene: " + str(self.scene.get_known_object_names()))
                return res
            return wrapper


    ## Adds a horizontal ground plane to the planning scene
    # @param name Name of the ground plane
    # @param z z-Position of the ground plane
    @_add_object_decorator
    def add_ground(self, name, z=0.0, frame=""):
        p = geometry_msgs.msg.PoseStamped()
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        if frame is not "":
            p.header.frame_id = frame
        else:
            p.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_plane(name,p)

    ## Add a box to the planning scene
    # @param name Name of the Object
    # @param size Size of the object given as vector list (dx,dy,dz)
    # @param position Position of the object given as vector list (x,y,z)
    # @param orientation Orientation of the object given as quaternion list (x,y,z,w)
    # @param frame Attach to the given frame
    @_add_object_decorator
    def add_box(self, name, size, position, orientation, frame=""):
        p = geometry_msgs.msg.PoseStamped()
        p.pose.position = geometry_msgs.msg.Point(position[0],position[1],position[2])
        p.pose.orientation = geometry_msgs.msg.Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])
        if frame is not "":
            p.header.frame_id = frame
        else:
            p.header.frame_id = self.robot.get_planning_frame()

        self.scene.add_box(name,p,size)


    ## Wait for timeout s if an object appears in the planning scene
    # @param object_name Nome of the object to check/wait for
    # @param timeout Wait for timeout s
    def wait_for_object(self,object_name, timeout=3.0):
        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():

          # Test if we are in the expected state
          if object_name in self.scene.get_known_object_names():
            return True

          # Sleep so that we give other threads time on the processor
          rospy.loginfo("Waiting for scene to be updated")
          rospy.sleep(0.5)
          seconds = rospy.get_time()

        rospy.logerr("Object '" + object_name + "' was not found in the scene")
        return False


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
            self._publish_tf(pose)
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

    ## Move the endeffector relative in the x-direction in a straight line
    def move_x_straigt(self, move_by_value, wait=True):
        rospy.loginfo("Received move x straight command: %f" % move_by_value)
        pose = self.group.get_current_pose().pose
        pose.position.x += move_by_value;
        self.move_cartesian_path_to_pose(pose)
        return

    ## Move the endeffector relative in the y-direction
    def move_y(self, move_by_value, wait=True):
        rospy.loginfo("Received move y command: %f" % move_by_value)
        self._shift_pose_target(1, move_by_value, wait)
        return

    ## Move the endeffector relative in the y-direction in a straight line
    def move_y_straigt(self, move_by_value, wait=True):
        rospy.loginfo("Received move y straight command: %f" % move_by_value)
        pose = self.group.get_current_pose().pose
        pose.position.y += move_by_value;
        self.move_cartesian_path_to_pose(pose)
        return

    ## Move the endeffector relative in the z-direction
    def move_z(self, move_by_value, wait=True):
        rospy.loginfo("Received move z command: %f" % move_by_value)
        self._shift_pose_target(2, move_by_value, wait)
        return

    ## Move the endeffector relative in the z-direction in a straight line
    def move_z_straigt(self, move_by_value, wait=True):
        rospy.loginfo("Received move z straight command: %f" % move_by_value)
        pose = self.group.get_current_pose().pose
        pose.position.z += move_by_value;
        self.move_cartesian_path_to_pose(pose)
        return

    ## Relative rotation of the endeffector around the x-axis
    def rotate_x(self, rotate_by_value, wait=True):
        rospy.loginfo("Received rotate x command: %f" % rotate_by_value)
        self._shift_pose_target(3, rotate_by_value, wait)
        return

    ## Relative rotation of the endeffector around the y-axis
    def rotate_y(self, rotate_by_value, wait=True):
        rospy.loginfo("Received rotate y command: %f" % rotate_by_value)
        self._shift_pose_target(4, rotate_by_value, wait)
        return

    ## Relative rotation of the endeffector around the z-axis
    def rotate_z(self, rotate_by_value, wait=True):
        rospy.loginfo("Received rotate z command: %f" % rotate_by_value)
        self._shift_pose_target(5, rotate_by_value, wait)
        return

    ## Move the endeffector to a named TF pose
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

    ## Move the endeffector to the position of a named TF pose
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

    ## Move the endeffector on a cartesian path (straight line) to the goal pose
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

    ## Get the current pose of the robot    
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
