#!/usr/bin/python3

## Debug script for easier fog_msgs.action.NavigationAction publishing

import rclpy
import math
import os

from abc import ABC, abstractmethod

from threading import Event

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import fog_msgs.srv
import fognav_msgs.action

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from action_msgs.msg import GoalStatus

# #{ def quaternion_from_euler(roll, pitch, yaw)

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Code below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

# #} end of def quaternion_from_euler(roll, pitch, yaw)

# #{ AbstractClass for Action - ActionClass

class ActionClass(ABC):

    def __init__(self, node_handler, service_type, service_topic, action_type, action_topic):

        self._action_done_event = Event()
        self._action_response_result = False
        self._goal_handle = None

        self.node_handler = node_handler

        self._action_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(node_handler, action_type, action_topic, callback_group = self._action_callback_group)

        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        self._service = node_handler.create_service(service_type, service_topic, self.service_callback, callback_group = self._service_callback_group) 
        

    @abstractmethod
    def send_goal(self, goal):
        pass

    def service_callback(self, request, response):
        self.node_handler.get_logger().info('Incoming request: \t{0}'.format(request.goal))
        self.send_goal(request.goal)

        # Wait for action to be done
        self._action_done_event.wait()
        response.success = self._action_response_result
        if response.success:
            response.message = "Goal accepted"
        else:
            response.message = "Goal rejected"
        return response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node_handler.get_logger().error('Goal rejected :(')
            # Signal that action is done
            self._action_done_event.set()
            return

        self.node_handler.get_logger().info('Goal accepted :)')
        self._action_response_result = True
        # Signal that action is done
        self._action_done_event.set()

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        self._goal_handle = goal_handle

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node_handler.get_logger().info('Goal succeeded! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_ABORTED:
            self.node_handler.get_logger().error('Goal aborted! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_CANCELED:
            self.node_handler.get_logger().error('Goal canceled! Result: {0}'.format(result.message))

        if self._goal_handle.goal_id == future.result().goal_id:
            self._goal_handle = None

    def feedback_callback(self, feedback_msg):
        self.node_handler.get_logger().info('Received feedback: {0}'.format(feedback_msg))

# #} end of AbstractClass for Action - ActionClass

# #{ GlobalServiceActionClass

class GlobalServiceActionClass(ActionClass):

    def __init__(self, node_handler, DRONE_DEVICE_ID):
        action_topic = "/" + DRONE_DEVICE_ID + "/navigate_to_pose"
        service_topic = "~/global_waypoint"
        super().__init__(node_handler, fog_msgs.srv.Vec4, service_topic, fognav_msgs.action.NavigateToPose, action_topic)
        self.node_handler = node_handler
        self.node_handler.get_logger().info('- GlobalService prepared : \'{}\''.format(service_topic))

    def send_goal(self, goal):
        self.node_handler.get_logger().info('Waiting for action server for GlobalService')
        self._action_client.wait_for_server()

        pose = PoseStamped()
        pose.header.stamp = rclpy.clock.Clock().now().to_msg()
        pose.header.frame_id = "geographic_world"
        point = Point()
        point.x = float(goal[0])
        point.y = float(goal[1])
        point.z = float(goal[2])
        pose.pose.position = point
        q = quaternion_from_euler(0,0,goal[3])
        pose.pose.orientation = q

        goal_msg = fognav_msgs.action.NavigateToPose.Goal()
        goal_msg.pose = pose
        # goal_msg.behavior_tree 
        goal_msg.not_auto_land = True

        self.node_handler.get_logger().info('Sending goal request...')
        self._action_done_event.clear()
        self._action_response_result = False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return

# #} end of class GlobalServiceActionClass

# #{ LocalServiceActionClass

class LocalServiceActionClass(ActionClass):

    def __init__(self, node_handler, DRONE_DEVICE_ID):
        action_topic = "/" + DRONE_DEVICE_ID + "/navigate_to_pose"
        service_topic = "~/local_waypoint"
        super().__init__(node_handler, fog_msgs.srv.Vec4, service_topic, fognav_msgs.action.NavigateToPose, action_topic)
        self.node_handler = node_handler
        self.node_handler.get_logger().info('- LocalService prepared : \'{}\''.format(service_topic))

    def send_goal(self, goal):
        self.node_handler.get_logger().info('Waiting for action server for GlobalService')
        self._action_client.wait_for_server()

        pose = PoseStamped()
        pose.header.stamp = rclpy.clock.Clock().now().to_msg()
        pose.header.frame_id = "world"
        point = Point()
        point.x = float(goal[0])
        point.y = float(goal[1])
        point.z = float(goal[2])
        pose.pose.position = point
        q = quaternion_from_euler(0,0,goal[3])
        pose.pose.orientation = q

        goal_msg = fognav_msgs.action.NavigateToPose.Goal()
        goal_msg.pose = pose
        # goal_msg.behavior_tree 
        goal_msg.not_auto_land = True

        self.node_handler.get_logger().info('Sending goal request...')
        self._action_done_event.clear()
        self._action_response_result = False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return

# #} end of class GlobalServiceActionClass

class NavigationActionClient(Node):

    def __init__(self):
        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')
        super().__init__(node_name="navigation_action_client", namespace=DRONE_DEVICE_ID)
        self.get_logger().info('| --------------------- Initialization --------------------- |')
        self.global_service_action = GlobalServiceActionClass(self, DRONE_DEVICE_ID)
        self.get_logger().info('| -------------------- Everything ready -------------------- |')

# #{ main 

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor)

    self.get_logger().info('=================================')

if __name__ == '__main__':
    main()

# #} end of 
