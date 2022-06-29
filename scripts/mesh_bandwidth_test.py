#!/usr/bin/env python3
from mimetypes import init
import os
import sys
import string
import argparse
import time
import iperf3
import signal
import threading
import multiprocessing as mp
import shutil
import random
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import utm

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.srv import SetParameters, GetParameters
from  scipy.interpolate import griddata 

from px4_msgs.msg import VehicleGpsPosition, VehicleLocalPosition

class Iperf:

    def __init__(self, args, array=None, lock=None, duration=1):
        self.args = args
        self.lock = lock
        self.array = array
        self.test_num = 1
        self.duration = duration

    def run_server(self):
        server = iperf3.Server()
        server.bind_address = self.args.ip
        server.port = self.args.port
        print("Starting server, listening on %s:%d" % (self.args.ip, self.args.port))
        while True:
            result = server.run()
            print("\r\tTest number %d done" % self.test_num, end=" ")
            sys.stdout.flush()
            self.test_num += 1


    def run_client(self):
        client = iperf3.Client()
        client.duration = self.duration 
        client.server_hostname = self.args.ip
        client.port = self.args.port

        result = client.run()
        return result
        
    def test(self):
        print("Starting client, listening on %s:%d" % (self.args.ip, self.args.port))
        while True:
            result = self.run_client()
            if result.error:
                print(result.error)
                time.sleep(1)
            else:
                print('\r\tTest number {} done'.format(self.test_num), end=" ")
                sys.stdout.flush()
                self.test_num += 1

                self.lock.acquire()
                self.array[0] = result.sent_MB_s
                self.array[1] = result.received_MB_s
                self.lock.release()

class GpsPositionSubscriber(Node):
    def __init__(self, args, name, array, lock, index):
        super().__init__('gps_position_subscriber', namespace=name)
        self.drone_device_id = name
        self.args = args
        self.lock = lock
        self.array = array
        self.index = index
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            VehicleGpsPosition,
            '/{}/fmu/vehicle_gps_position/out'.format(name),
            self.gps_position_listener_cb,
            self.qos_profile)
        self.sub

    def gps_position_listener_cb(self, msg):
        self.lock.acquire()
        self.array[self.index] = msg.lat
        self.array[self.index+1] = msg.lon
        self.lock.release()

class LocalPositionSubscriber(Node):
    def __init__(self, args, name, array, lock, index):
        super().__init__('local_position_subscriber', namespace=name)
        self.drone_device_id = name
        self.args = args
        self.lock = lock
        self.array = array
        self.index = index
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            VehicleLocalPosition,
            '/{}/fmu/vehicle_local_position/out'.format(name),
            self.local_position_listener_cb,
            self.qos_profile)
        self.sub

    def local_position_listener_cb(self, msg):
        self.lock.acquire()
        self.array[self.index] = -msg.z
        self.array[self.index+1] = msg.heading
        self.lock.release()

        
class RosClient:
    def __init__(self, args, names, array, lock):
        self.nodes = []
        self.lock = lock
        self.names = names
        self.array = array

    def run(self):
        for i, name in enumerate(self.names):
            self.nodes.append(GpsPositionSubscriber(args, name, self.array, self.lock, 2+i*4))
            self.nodes.append(LocalPositionSubscriber(args, name, self.array, self.lock, 4+i*4))

        while True:
            for node in self.nodes:
                rclpy.spin_once(node, timeout_sec=1) 
                time.sleep(0.8)

    def destroy(self):
        for node in self.nodes:
            node.destroy()

class Plot(Node):
    def __init__(self, args, data):
        self.drone_device_id = os.getenv('DRONE_DEVICE_ID')
        self.args = args
        self.data = data

    def plot(self):
        lat_c = 1e-7*self.data['latitude_client']
        lon_c = 1e-7*self.data['longitude_client']
        lat_s = 1e-7*self.data['latitude_server']
        lon_s = 1e-7*self.data['longitude_server']
        
        xc, yc, _, _ = utm.from_latlon(lat_c, lon_c)
        xs, ys, _, _ = utm.from_latlon(lat_s, lon_s)

        x = xc - xs
        y = yc - ys

        i = self.data['sent_MBs']

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # define grid.
        xi = np.linspace(np.min(x),np.max(x),1000)
        yi = np.linspace(np.min(y),np.max(y),1000)
        xi, yi = np.meshgrid(xi, yi)
        
        # grid the data.
        zi = griddata((x, y), i, (xi, yi), method='linear')
        
        plt.contourf(xi,yi,zi)
        plt.plot(x,y,'k.')

        cbar = plt.colorbar()

        # bx = fig.add_subplot(122)
        # bx.pcolormesh(zi, norm=matplotlib.colors.LogNorm())
        # bx.pcolormesh(zi)

        plt.show()



def generate_fake_output(w):
    array = mp.Array('f', range(10))
    x = 5
    y = 15
    r = 10
    for i in range(500):
        array[0] = 50*random.gauss(1, 0.1)
        array[1] = 50*random.gauss(1, 0.1)
        array[2] = x+r*math.cos(i) + random.gauss(0,1.1)
        array[3] = x+r*math.sin(i) + random.gauss(0,1.1)
        array[4] = 3+random.gauss(0,1.1)
        array[5] = 0
        array[6] = x+random.gauss(0,1.1)
        array[7] = y+random.gauss(0,1.1)
        array[8] = 3+random.gauss(0,1.1)
        array[9] = 0

        w.writerow(array)
   
def init_arg_parser():
    parser = argparse.ArgumentParser(
        prog='mesh_bandwith_test.py',
        epilog="See '<command> --help' to read about a specific sub-command."
    )

    subparsers = parser.add_subparsers(dest='command', help='Sub-commands')

    server_parser = subparsers.add_parser('server', help='Run server')
    server_parser.add_argument('-i', '--ip', type=str, help='Ip address of the server, default=[127.0.0.1]', default='127.0.0.1')
    server_parser.add_argument('-p', '--port', help='port, default=[5201]', default=5201)

    client_parser = subparsers.add_parser('client', help='Run client')
    client_parser.add_argument('-i', '--ip', type=str, help='Ip address of the server to connect, default=[127.0.0.1]', default='127.0.0.1')
    client_parser.add_argument('-p', '--port', type=int, help='port, default=[5201]', default=5201)
    client_parser.add_argument('-sn', '--server_name', type=str, help='Name of the device where the server is running', required=True)
    client_parser.add_argument('-td', '--test_duration', type=int, help='Duration of each test cycle in seconds, default=[1]', default=1)

    plot_parser = subparsers.add_parser('plot', help='Plot the results')
    plot_parser.add_argument('-f', '--file', type=str, help='Path to a data file, default=[/tmp/data.csv]', default='/tmp/data.csv')

    save_parser = subparsers.add_parser('save', help='Save results into given path')
    save_parser.add_argument('-p', '--path', type=str, help='Path to copy the data file', required=True)

    fake_parser = subparsers.add_parser('fake', help='Generate fake data for the plot testing')

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()

    return args

def main(args):

    status = 1

    if args.command == 'client':
        lock = mp.Lock()
        names = [os.getenv('DRONE_DEVICE_ID'), args.server_name]
        array = mp.Array('f', range(2+4*len(names)))

        rclpy.init()
        file = open('/tmp/data.csv', 'w')
        writer = csv.writer(file)
        writer.writerow(["sent_MBs", "received_MBs", "latitude_client", "longitude_client", "altitude_client", "heading_client", "latitude_server", "longitude_server", "altitude_server", "heading_server"])

        iperf = Iperf(args, array, lock, args.test_duration)
        ros = RosClient(args, names, array, lock)

        ipp = mp.Process(target=iperf.test)
        rosp = mp.Process(target=ros.run)

        ipp.start()
        rosp.start()

        try: 
            while True:
                lock.acquire()
                # print("writing")
                writer.writerow(array)
                lock.release()
                time.sleep(1)
   
        except KeyboardInterrupt:
            ros.destroy()
            ipp.terminate()
            rosp.terminate()
            ipp.join()
            rosp.join()
            file.close()
            print('I am out')
            rclpy.shutdown()
            status = 0

    elif args.command == 'server':
        iperf = Iperf(args)
        iperf.run_server()
        status = 0

    elif args.command == 'plot':
        data = np.genfromtxt(args.file, dtype=float, delimiter=',', names=True)
        sp = Plot(args, data)
        sp.plot()
        status = 0

    elif args.command == 'save':
        shutil.copy('/tmp/data.csv', args.path) 
        status = 0

    elif args.command == 'fake':
        file = open('/tmp/data.csv', 'w')
        writer = csv.writer(file)
        writer.writerow(["sent_MBs", "received_MBs", "latitude_client", "longitude_client", "altitude_client", "heading_client", "latitude_server", "longitude_server", "altitude_server", "heading_server"])
        generate_fake_output(writer)
        file.close()
        status = 0

    return status

if __name__ == '__main__':

    args = init_arg_parser()
    status = main(args)

sys.exit(status)
