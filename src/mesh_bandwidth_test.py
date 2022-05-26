#!/usr/bin/env python3

from mimetypes import init
import string
import sys
import os
import argparse
from random import uniform
import time
import iperf3
import signal

from std_srvs.srv import SetBool, Trigger
from fog_msgs.srv import Vec4
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters

from px4_msgs.msg import VehicleGpsPosition
from std_msgs.msg import String

class FogClientAsync(Node):

    def __init__(self, args):
        super().__init__('fog_client_async')
        self.drone_device_id = os.getenv('DRONE_DEVICE_ID')
        self.args = args

    def run_server(self):
        server = iperf3.Server()
        server.bind_address = self.args.ip
        server.port = self.args.port
        print("Starting server, listening on %s:%d" % (self.args.ip, self.args.port))
        n = 1
        while True:
            result = server.run()
            print("\tTest number %d done" % n)
            n = n + 1

    def run_client(self):
        client = iperf3.Client()
        client.duration = 1
        client.server_hostname = self.args.ip
        client.port = self.args.port
        client.num_streams = 1

        result = client.run()
        return result
        
    def test(self):
        while True:
            result = self.run_client()

            if result.error:
                print(result.error)
                time.sleep(1)
            else:
                print('Test completed')
                print(result.sent_MB_s, result.received_MB_s)

class FogClientSync(Node):
    def __init__(self, args):
        super().__init__('minimal_subscriber', namespace=os.getenv('DRONE_DEVICE_ID'))
        self.drone_device_id = os.getenv('DRONE_DEVICE_ID')
        self.args = args
        self.vehicle_status_subs = None
        self.navigation_status_subs = None
        # self.msg_type = importlib.import_module(args.msg_type)
        self._waiting = True
        # self.topic = args.topic
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def gps_position_listener_cb(self, msg):
        self.get_logger().info('msg data={}'.format(msg.data))
        if msg.data == 'IDLE':
            self._waiting = False


    def create_subs(self):
        self.vehicle_gps_subs = self.create_subscription(
            VehicleGpsPosition,
            '/{}/fmu/vehicle_gps_position/out'.format(self.drone_device_id),
            self.gps_position_listener_cb,
            self.qos_profile)
        self.vehicle_gps_subs  # prevent unused variable warning
            
    def waiting(self):
        return self._waiting


    
def init_arg_parser():
    parser = argparse.ArgumentParser(
        prog='mesh_bandwith_test.py',
        epilog="See '<command> --help' to read about a specific sub-command."
    )

    parser.add_argument('--timeout', type=int, default=30)
    subparsers = parser.add_subparsers(dest='command', help='Sub-commands')

    server_parser = subparsers.add_parser('server', help='Run server')
    server_parser.add_argument('--ip', type=str, help='Ip address of the server, default=[127.0.0.1]', default='127.0.0.1')
    server_parser.add_argument('--port', type=int, default=5201)

    client_parser = subparsers.add_parser('client', help='Run client')
    client_parser.add_argument('-ip', type=str, help='Ip address of the server to connect, default=[127.0.0.1]', default='127.0.0.1')
    client_parser.add_argument('--port', type=int, help='port, default=[5201]', default=5201)
    client_parser.add_argument('--time', type=int)
    client_parser.add_argument('--uav_name', type=int)

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()

    return args

def random_points_range(startx, endx, starty, endy):

    x = round(uniform(startx, endx), 2)
    y = round(uniform(starty, endy), 2)
    z = round(uniform(1, 2), 2)

    return x, y, z


def get_waypoint(area):
    if area == 0:
        return 0.0, 0.0, 1.0
    elif area == 1:
        return random_points_range(-0.5, 0, 0, 1.5)
    elif area == 2:
        return random_points_range(-0.5, 0, -2, -3.5)
    elif area == 3:
        return random_points_range(-2, -2.5, -2, -3.5)
    elif area == 4:
        return random_points_range(-2, -2.5, 0, 1.5)
    print("Area command failed. Choose an integer between 0 and 4.")
    raise ValueError

def handler(signum, frame):
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

def main(args):

    status = 0

    rclpy.init()

    fog_client = FogClientAsync(args)
    fog_client_sync = FogClientSync(args)

    if args.command == 'server':
        fog_client.run_server()

    elif args.command == 'client':
        fog_client.test()
        
    fog_client_sync.create_subs()

    if fog_client_sync.waiting():
        status = False

    fog_client.destroy_node()
    fog_client_sync.destroy_node()

    rclpy.shutdown()

    return status

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    args = init_arg_parser()
    status = main(args)

sys.exit(status)
