#!/usr/bin/env python

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import threading
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SrRobotCommander(object):
    """
    Base class for hand and arm commanders
    """
    __group_prefixes= {"right_arm": "ra_",
                       "left_arm": "la_",
                       "right_hand": "rh_",
                       "left_hand": "lh_"}

    def __init__(self, name):
        """
        Initialize MoveGroupCommander object
        @param name - name of the MoveIt group
        """
        self._move_group_commander = MoveGroupCommander(name)
        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = rospy.Subscriber("joint_states", JointState, self._joint_states_callback)
        self._joints_position = {}
        self._joints_velocity = {}
        self._joints_effort = {}
        
        # prefix of the trajectory controller
        self._prefix = self.__group_prefixes[name]
        # a trajectory goal that will contain a single trajectory point (for the move unsafe command)
        self._trajectory_goal = FollowJointTrajectoryGoal()
        self._trajectory_goal.trajectory = JointTrajectory()
        self._set_default_trajectory()        
        self._set_up_action_client()
        
        threading.Thread(None, rospy.spin)

    def move_to_joint_value_target(self, joint_states, wait_result=True):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can contain only joints values of which need to
        be changed.
        @param wait_result - should method wait for movement end or not
        """
        self._move_group_commander.set_joint_value_target(joint_states)
        self._move_group_commander.go(wait=wait_result)

    def move_to_named_target(self, name, wait_result=True):
        """
        Set target of the robot's links and moves to it
        @param name - name of the target pose defined in SRDF
        @param wait_result - should method wait for movement end or not
        """
        self._move_group_commander.set_named_target(name)
        self._move_group_commander.go(wait=wait_result)

    def get_joints_position(self):
        """
        Returns joints position
        @return - dictionary with joints positions
        """
        with self._joint_states_lock:
            return self._joints_position

    def get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities
        """
        with self._joint_states_lock:
            return self._joints_velocity

    def _get_joints_effort(self):
        """
        Returns joints effort
        @return - dictionary with joints efforts
        """
        with self._joint_states_lock:
            return self._joints_effort

    def run_joint_trajectory(self, joint_trajectory, wait_result=True):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents trajectory of the joints which would be
        executed.
        @param wait_result - should method wait for movement end or not
        """
        plan = RobotTrajectory()
        plan.joint_trajectory = joint_trajectory
        self._move_group_commander.execute(plan, wait_result)

    def _move_to_position_target(self, xyz, end_effector_link="", wait_result=True):
        """
        Specify a target position for the end-effector and moves to it
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait_result - should method wait for movement end or not
        """
        self._move_group_commander.set_position_target(xyz, end_effector_link)
        self._move_group_commander.go(wait=wait_result)

    def _joint_states_callback(self, joint_state):
        """
        The callback function for the topic joint_states.
        It will store the received joint position, velocity and efforts information into dictionaries
        @param joint_state - the message containing the joints data.
        """
        with self._joint_states_lock:
            self._joints_position = {n: p for n, p in zip(joint_state.name, joint_state.position)}
            self._joints_velocity = {n: v for n, v in zip(joint_state.name, joint_state.velocity)}
            self._joints_effort = {n: v for n, v in zip(joint_state.name, joint_state.effort)}

    def _set_up_action_client(self):
        """
        Sets up an action client to communicate with the trajectory controller
        """
        self._client = SimpleActionClient(
            self._prefix + "trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        if self._client.wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to action server in 4 sec")
            raise
    
    def _set_default_trajectory(self):
        """
        Builds a default trajectory for this group, With the correct joint names and containing one point
        with the positions set to 0
        """
        active_joints = self._move_group_commander.get_active_joints()
        print active_joints
        self._trajectory_goal.trajectory.joint_names = active_joints
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self._trajectory_goal.trajectory.joint_names)
        point.velocities = [0.0] * len(self._trajectory_goal.trajectory.joint_names)
        self._trajectory_goal.trajectory.points = []
        self._trajectory_goal.trajectory.points.append(point)
        
    def _update_default_trajectory(self):
        """
        Fill a trajectory point with the current position of the robot. It will serve as a base to move only the
        joints the user specifies in the move_to_joint_value_target_unsafe
        """
        current_joints_position = self.get_joints_position()
        if len(current_joints_position) >= len(self._trajectory_goal.trajectory.joint_names):
            for i, joint_name in enumerate(self._trajectory_goal.trajectory.joint_names):
                self._trajectory_goal.trajectory.points[0].positions[i] = current_joints_position[joint_name]
        else:
            rospy.logerr("Joints position length mismatch {} vs traj {}".format(len(current_joints_position), len(self._trajectory_goal.trajectory.joint_names)))
        
        print self._trajectory_goal.trajectory.points[0].positions
        
    def _set_targets_to_default_trajectory(self, joint_states):
        """
        Set the target values in joint_states to the default trajectory goal (leaving the others with their original value).
        @param joint_states - dictionary with joint name and value. It can contain only joints values of which need to
        be changed.
        """
        for name, pos in joint_states.items():
            i = self._trajectory_goal.trajectory.joint_names.index(name)
            self._trajectory_goal.trajectory.points[0].positions[i] = pos
        
    def move_to_joint_value_target_unsafe(self, joint_states, time = 0.002, wait_result=True):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can contain only joints values of which need to
        be changed.
        @param wait_result - should method wait for movement end or not
        @param time - time in s (counting from now) for the robot to reach the target (it needs to be greater than 0.0 for 
        it not to be rejected by the trajectory controller)
        """
        
        self._update_default_trajectory()
        self._set_targets_to_default_trajectory(joint_states)
        self._trajectory_goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(time)
        self._client.send_goal(self._trajectory_goal)
        
        if not wait_result:
            return
        
        if not self._client.wait_for_result():
            rospy.loginfo("Trajectory not completed")
    
    def run_joint_trajectory_unsafe(self, joint_trajectory, wait_result=True):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents trajectory of the joints which would be
        executed.
        @param wait_result - should method wait for movement end or not
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory
        self._client.send_goal(goal)
        
        if not wait_result:
            return
        
        if not self._client.wait_for_result():
            rospy.loginfo("Trajectory not completed")
        