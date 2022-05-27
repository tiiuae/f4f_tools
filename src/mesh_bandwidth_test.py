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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.srv import SetParameters, GetParameters
from  scipy.interpolate import griddata 

from px4_msgs.msg import VehicleGpsPosition

class Iperf:

    def __init__(self, args, array=None, lock=None):
        self.args = args
        self.lock = lock
        self.array = array

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
                self.lock.acquire()
                self.array[0] = result.sent_MB_s
                self.array[1] = result.received_MB_s
                self.lock.release()

class PositionSubscriber(Node):
    def __init__(self, args, name, array, lock, index):
        super().__init__('position_subscriber', namespace=name)
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
        self.array[self.index+2] = msg.alt
        self.lock.release()

        
class RosClient:
    def __init__(self, args, names, array, lock):
        self.nodes = []
        self.lock = lock
        self.names = names
        self.array = array

    def run(self):
        for i, name in enumerate(self.names):
            self.nodes.append(PositionSubscriber(args, name, self.array, self.lock, 2+i*3))

        while True:
            print(len(self.nodes))
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
        x = self.data[:,2] - self.data[:,5]
        y = self.data[:,3] - self.data[:,6]
        i = self.data[:,0]
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # define grid.
        xi = np.linspace(np.min(x),np.max(x),100)
        yi = np.linspace(np.min(y),np.max(y),100)
        xi, yi = np.meshgrid(xi, yi)
        
        # grid the data.
        zi = griddata((x, y), i, (xi, yi), method='linear')
        
        plt.contourf(xi,yi,zi)
        plt.plot(x,y,'k.')
        # ax.scatter(x, y, c=i, edgecolors='none', norm=matplotlib.colors.LogNorm())

        cbar = plt.colorbar()

        # bx = fig.add_subplot(122)
        # bx.pcolormesh(zi, norm=matplotlib.colors.LogNorm())
        # bx.pcolormesh(zi)

        plt.show()



def generate_fake_output(w):
    array = mp.Array('f', range(8))
    x = 5
    y = 15
    r = 10
    for i in range(500):
        array[0] = 50*random.gauss(1, 0.1)
        array[1] = 50*random.gauss(1, 0.1)
        array[2] = x+r*math.cos(i) + random.gauss(0,0.1)
        array[3] = x+r*math.sin(i) + random.gauss(0,0.1)
        array[4] = 3+random.gauss(0,0.1)
        array[5] = x+random.gauss(0,0.1)
        array[6] = y+random.gauss(0,0.1)
        array[7] = 3+random.gauss(0,0.1)

        w.writerow(array)
   
def init_arg_parser():
    parser = argparse.ArgumentParser(
        prog='mesh_bandwith_test.py',
        epilog="See '<command> --help' to read about a specific sub-command."
    )

    subparsers = parser.add_subparsers(dest='command', help='Sub-commands')

    server_parser = subparsers.add_parser('server', help='Run server')
    server_parser.add_argument('--ip', type=str, help='Ip address of the server, default=[127.0.0.1]', default='127.0.0.1')
    server_parser.add_argument('-p', '--port', help='port, default=[5201]', default=5201)

    client_parser = subparsers.add_parser('client', help='Run client')
    client_parser.add_argument('-ip', '--ip', type=str, help='Ip address of the server to connect, default=[127.0.0.1]', default='127.0.0.1')
    client_parser.add_argument('-p', '--port', type=int, help='port, default=[5201]', default=5201)
    client_parser.add_argument('-n', '--server_name', type=str, help='Name of the uav where is server running')

    plot_parser = subparsers.add_parser('plot', help='Plot the results')
    plot_parser.add_argument('--file', type=str, help='Path to a file, default=[/tmp/data.csv]', default='/tmp/data.csv')

    save_parser = subparsers.add_parser('save', help='Save results into given path')
    save_parser.add_argument('--path', type=str, help='path to a file', required=True)

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
        array = mp.Array('f', range(2+3*len(names)))

        rclpy.init()
        file = open('/tmp/data.csv', 'w')
        writer = csv.writer(file)

        iperf = Iperf(args, array, lock)
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
        data = np.genfromtxt(args.file, delimiter=',')
        sp = Plot(args, data)
        sp.plot()
        status = 0

    elif args.command == 'save':
        shutil.copy('/tmp/data.csv', args.path) 
        status = 0

    elif args.command == 'fake':
        file = open('/tmp/data.csv', 'w')
        writer = csv.writer(file)
        generate_fake_output(writer)
        file.close()
        status = 0

    return status

if __name__ == '__main__':

    args = init_arg_parser()
    status = main(args)

sys.exit(status)
