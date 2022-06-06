#!/usr/bin/python3

## Debug script for easier fog_msgs.action.NavigationAction publishing

import rclpy
import math
import os

from threading import Event

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fognav_msgs.action import NavigateToPose
from fognav_msgs.action import Takeoff
from fognav_msgs.action import Land

from std_srvs.srv import Trigger
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from action_msgs.msg import GoalStatus

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

class PoseClient(Node):

    def __init__(self):
        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

        super().__init__("navigation_action_client")
        self._action_done_event = Event()
        self._action_cancel_event = Event()
        self._action_response_result = False
        self._action_cancel_result = False
        self._goal_handle = None

        self._action_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(self, NavigateToPose, "/" + DRONE_DEVICE_ID + "/navigation", callback_group = self._action_callback_group)


    def local_waypoint_callback(self, request, response):
        self.send_goal(request.goal, True)
        
        # Wait for action to be done
        
    def gps_waypoint_callback(self, request, response):
        self.get_logger().info('Incoming gps waypoint request: \t{0}'.format(request.goal))
        self.send_goal(request.goal, False)
        
        # Wait for action to be done
        self._action_done_event.wait()
        response.success = self._action_response_result
        if response.success:
            response.message = "Goal accepted"
        else:
            response.message = "Goal rejected"
        return response

    def cancel_goal_callback(self, request, response):
        self.get_logger().info('Incoming request to cancel goal')
        if self._goal_handle is None: 
            response.success = False
            response.message = "No active goal"
            self.get_logger().error('{0}'.format(response.message))
            return response

        self.cancel_goal()
        # Wait for action to be done
        self._action_cancel_event.wait()
        response.success = self._action_cancel_result
        if response.success:
            response.message = "Goal canceled"
        else:
            response.message = "Goal failed to cancel"
        return response


    def send_goal(self, goal, is_local):
        self.get_logger().info('Incoming local waypoint request: \t{0}'.format(request.goal))

        self.get_logger().info('Waiting for action server')
        self._action_client.wait_for_server()

        path = NavPath()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "world"

        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = "world"
        point = Point()
        point.x = float(goal[0])
        point.y = float(goal[1])
        point.z = float(goal[2])
        pose.pose.position = point
        q = quaternion_from_euler(0,0,goal[3])
        pose.pose.orientation = q
        path.poses.append(pose)

        goal_msg = NavigationAction.Goal()
        goal_msg.path = path
        goal_msg.is_local = is_local
        
        self.get_logger().info('Sending goal request...')
        self._action_done_event.clear()
        self._action_response_result = False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            # Signal that action is done
            self._action_done_event.set()
            return

        self.get_logger().info('Goal accepted :)')
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
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal aborted! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error('Goal canceled! Result: {0}'.format(result.message))

        if self._goal_handle.goal_id == future.result().goal_id:
            self._goal_handle = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.mission_progress))

def init_arg_parser():
    parser = argparse.ArgumentParser(
        prog='mesh_bandwith_test.py',
        epilog="See '<command> --help' to read about a specific sub-command."
    )

    subparsers = parser.add_subparsers(dest='command', help='Sub-commands')

    server_parser = subparsers.add_parser('takeoff', help='Run server')

    client_parser = subparsers.add_parser('land', help='Run client')

    plot_parser = subparsers.add_parser('local', help='Plot the results')
    plot_parser.add_argument('-x', '--file', type=str, help='Path', required=True)
    plot_parser.add_argument('-y', '--file', type=str, help='Path', required=True)
    plot_parser.add_argument('-z', '--file', type=str, help='Path', required=True)
    plot_parser.add_argument('-h', '--file', type=str, help='Path', required=True)

    save_parser = subparsers.add_parser('gps', help='Save results into given path')
    save_parser.add_argument('-x', '--file', type=str, help='Path', required=True)
    save_parser.add_argument('-y', '--file', type=str, help='Path', required=True)
    save_parser.add_argument('-z', '--file', type=str, help='Path', required=True)
    save_parser.add_argument('-h', '--file', type=str, help='Path', required=True)

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()

    return args

def main(args):

    rclpy.init()
    if args.command == 'land':

        action_client = LandClient()
        action_client.get_logger().info('********************************')

        action_client.send_goal()
        rclpy.spin(action_client)

        action_client.get_logger().info('=================================')
    
    elif args.command == 'takeoff':
        
        action_client = TakeoffClient()
        action_client.get_logger().info('********************************')

        action_client.send_goal()
        rclpy.spin(action_client)

        action_client.get_logger().info('=================================')

    elif args.command == 'local':

        action_client = PoseClient()
        action_client.get_logger().info('********************************')

        action_client.send_goal()
        rclpy.spin(action_client)

        action_client.get_logger().info('=================================')

    elif args.command == 'gps':

        action_client = PoseClient()
        action_client.get_logger().info('********************************')

        action_client.send_goal()
        rclpy.spin(action_client)

        action_client.get_logger().info('=================================')

    

if __name__ == '__main__':

    args = init_arg_parser()
    main(args)

sys.exit()
