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
from px4_msgs.msg import VehicleGpsPosition, VehicleLocalPosition

from scipy import interpolate

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
            '/{}/mesh/vehicle_gps_position/out'.format(name),
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
            '/{}/mesh/vehicle_local_position/out'.format(name),
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
                time.sleep(0.1)

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

        head_s = self.data['heading_server']
        head_c = self.data['heading_client']
        head = head_s + (head_s - head_c)
        
        xc, yc, _, _ = utm.from_latlon(lat_c, lon_c)
        xs, ys, _, _ = utm.from_latlon(lat_s, lon_s)

        x = xc - xs
        y = yc - ys

        d = np.round(self.data['sent_MBs'], 1)

        # define grid.
        margin = 3
        resolution = 100 

        xi = np.linspace(np.min(x)-margin,np.max(x)+margin, resolution)
        yi = np.linspace(np.min(y)-margin,np.max(y)+margin, resolution)
        xi, yi = np.meshgrid(xi, yi)
        
        # grid the data.
        zi = interpolate.griddata((x, y), d, (xi, yi), method='linear')

        zi[(pow(xi,2) + pow(yi,2) <= 4)] = np.nan
        
        fig = plt.figure()
        fig.tight_layout()

        ax = fig.add_subplot(111)
        ax.set(title=self.args.title, xlabel='x [m]', ylabel='y [m]')

        cbar_low = 5
        cbar_high = 40

        cont = ax.contourf(xi,yi,zi, np.arange(cbar_low, cbar_high, 2), cmap=plt.cm.get_cmap('YlGnBu'))

        # plot client drone positions
        for i in range(len(x)):
            ax.arrow(x[i], y[i], math.sin(head[i]), math.cos(head[i]), head_width=0.4, head_length=0.5, fc='k', ec='k')
            ax.plot(x[i], y[i], marker="o", markersize=1, markerfacecolor="black", markeredgecolor="black")
            # ax.annotate(d[i], xy=(x[i], y[i]))

        # plot server drone position
        ax.plot(0, 0, marker="o", markersize=2, markerfacecolor="red", markeredgecolor="black")
        ax.arrow(0, 0, math.sin(head_s[0]), math.cos(head_s[0]), head_width=0.3, head_length=0.5, fc='r', ec='r')

        cbar = plt.colorbar(cont)
        cbar.set_label('MB/s', rotation=270, labelpad=12)

        plt.show()


def generate_fake_output(w):
    array = mp.Array('f', range(10))
    x = 0
    y = 0 
    r = 10
    for i in range(500):
        array[0] = 20*random.gauss(1, 0.1)
        array[1] = 20*random.gauss(1, 0.1)
        array[2], array[3] = utm.to_latlon(150000 + x+r*math.cos(i) + random.gauss(0,1.1), 150000 + x+r*math.sin(i) + random.gauss(0,1.1), 33, "U")
        array[2] = array[2]/1e-7
        array[3] = array[3]/1e-7
        array[4] = 3+random.gauss(0,1.1)
        array[5] = 0
        array[6], array[7] = utm.to_latlon(150000 + random.gauss(0,1.1), 150000 + random.gauss(0,1.1), 33, "U")
        array[6] = array[6]/1e-7
        array[7] = array[7]/1e-7
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
    plot_parser.add_argument('-t', '--title', type=str, help='Title to the figure, default is nothing', default='Mesh test')

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
