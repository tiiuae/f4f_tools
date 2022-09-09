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

class Status:
    def __init__(self, args, data, lock):
        self.args = args
        self.lock = lock
        self.data = data

    def client(self):
        while True:
            self.lock.acquire()
            print("\r Written: {:.0f}, tests done: {:.0f}, CLIENT times - local: {:.2f}, gps: {:.2f}, SERVER times - local: {:.2f}, gps: {:.2f}".format(self.data[0], self.data[1], self.data[2], self.data[3], self.data[4], self.data[5]), end=" ")
            self.lock.release()
            sys.stdout.flush()

            time.sleep(1)

class Iperf:

    def __init__(self, args, status=None, status_lock=None, array=None, lock=None, duration=1):
        self.args = args
        self.lock = lock
        self.array = array
        self.duration = duration
        self.status = status
        self.lock_status = status_lock

    def run_server(self):
        server = iperf3.Server()
        server.bind_address = self.args.ip
        server.port = self.args.port
        print("Starting server, listening on %s:%d" % (self.args.ip, self.args.port))
        num_of_tests=0
        while True:
            result = server.run()
            print("\r\tTests done: {}".format(num_of_tests), end=" ")
            sys.stdout.flush()
            num_of_tests+=1

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
                self.lock_status.acquire()
                self.status[1] += 1
                self.lock_status.release()

                self.lock.acquire()
                self.array[0] = result.sent_MB_s
                self.array[1] = result.received_MB_s
                self.lock.release()

class Uav(Node):
    def __init__(self, name, array, lock, status, lock_status, index):
        super().__init__('ros_node', namespace=name)
        self.name = name
        self.lock = lock
        self.array = array
        self.index = 2+4*index
        self.status = status
        self.lock_status = lock_status
        self.last_gps_stamp = 0
        self.last_local_stamp = 0
        self.gps_initialized = False
        self.local_initialized = False
        self.status_index = index

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        gps_name = '/{}/mesh/vehicle_gps_position/out'.format(self.name)
        local_name = '/{}/mesh/vehicle_local_position/out'.format(self.name)

        self.local = self.create_subscription(
            VehicleLocalPosition,
            local_name,
            self.local_position_listener_cb,
            self.qos_profile)
        self.local

        self.gps = self.create_subscription(
            VehicleGpsPosition,
            gps_name,
            self.gps_position_listener_cb,
            self.qos_profile)
        self.gps

    def gps_position_listener_cb(self, msg):

        if not self.gps_initialized:
            self.last_gps_stamp = msg.timestamp
            self.gps_initialized = True
            return
            
        self.lock.acquire()
        self.array[self.index] = msg.lat
        self.array[self.index+1] = msg.lon
        self.lock.release()

        self.lock_status.acquire()
        self.status[2+2*self.status_index] = (msg.timestamp - self.last_gps_stamp)/1000000
        self.lock_status.release()
        self.last_gps_stamp = msg.timestamp

    def local_position_listener_cb(self, msg):
        if not self.local_initialized:
            self.last_local_stamp = msg.timestamp
            self.local_initialized = True
            return

        self.lock.acquire()
        self.array[self.index+2] = -msg.z
        self.array[self.index+3] = msg.heading
        self.lock.release()
        
        self.lock_status.acquire()
        self.status[2+2*self.status_index+1] = (msg.timestamp - self.last_local_stamp)/1000000
        self.lock_status.release()
        self.last_local_stamp = msg.timestamp

class RosClient():
    def __init__(self, args, names, array, lock, status, lock_status):
        self.lock = lock
        self.names = names
        self.array = array
        self.nodes = []
        self.status = status
        self.lock_status = lock_status

    def run(self):
        status_index = 0
        for i, name in enumerate(self.names):
            self.nodes.append(Uav(name, self.array, self.lock, self.status, self.lock_status, i))

        while True:
            for node in self.nodes:
                rclpy.spin_once(node)

    def destroy_node():
        for node in self.nodes:
            node.destroy()


class Plot:
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
    array = mp.Array('f', np.zeros(10))
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
        array = mp.Array('f', np.zeros(2+4*len(names)))
        status_data = mp.Array('f', np.zeros(2+2*len(names)))
        lock_status = mp.Lock()

        rclpy.init()
        file = open('/tmp/data.csv', 'w')
        writer = csv.writer(file)
        writer.writerow(["sent_MBs", "received_MBs", "latitude_client", "longitude_client", "altitude_client", "heading_client", "latitude_server", "longitude_server", "altitude_server", "heading_server"])

        iperf = Iperf(args, status_data, lock_status, array, lock, args.test_duration)
        ros = RosClient(args, names, array, lock, status_data, lock_status)
        st = Status(args, status_data, lock_status)

        ipp = mp.Process(target=iperf.test)
        rosp = mp.Process(target=ros.run)
        stp = mp.Process(target=st.client)

        ipp.start()
        rosp.start()
        stp.start()

        try: 
            while True:
                if 0.0 not in array:
                    lock.acquire()
                    writer.writerow(array)
                    lock.release()

                    lock_status.acquire()
                    status_data[0] += 1
                    lock_status.release()

                    time.sleep(1)
   
        except KeyboardInterrupt:
            ros.destroy_node()
            ipp.terminate()
            rosp.terminate()
            stp.terminate()
            ipp.join()
            rosp.join()
            stp.join()

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
