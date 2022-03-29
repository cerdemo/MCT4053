from myo import *
import datetime
import math
import logging
import argparse
import time
import sys
import numpy as np
from threading import Thread
from multiprocessing import Process
from os import listdir
from pythonosc import osc_message_builder
from pythonosc import udp_client
from scipy.signal import butter, filtfilt

# global parameters
PORT = 3000

# only for mac
TTY = [f for f in listdir("/dev") if f.startswith("tty.usbmodem")]
print(TTY)

# macs = {'m7':'d0-8d-fc-7f-f5-f1',
#         'm8':'ef-9d-fd-31-ea-10',
#         'mcagr':'d9-db-d4-2b-d4-17',
#         'm1':'c3-0c-f3-14-d7-f0'}
# HARDCODE THE MAC ADDRESSES
# MAC = ["d9:db:d4:2b:d4:17", "d0:8d:fc:7f:f5:f1"]
MAC = ["d0:8d:fc:7f:f5:f1", "d9:db:d4:2b:d4:17"]

print("The auto MAC address is: ", MAC)

# variables
myo = []
osc_client = udp_client.SimpleUDPClient("localhost", PORT)


def vector_3d_magnitude(x, y, z):
    """Calculate the magnitude of a 3d vector"""
    return math.sqrt((x * x) + (y * y) + (z * z))


def toEulerAngle(w, x, y, z):
    """ Quaternion to Euler angle conversion borrowed from wikipedia.
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles """
    # roll (x-axis rotation)
    sinr = +2.0 * (w * x + y * z)
    cosr = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # pitch (y-axis rotation)
    sinp = +2.0 * (w * y - z * x)
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny = +2.0 * (w * z + x * y)
    cosy = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


# def tkeo(x):
#     x = np.asarray(x)
#     y = np.copy(x)
#     # Teager–Kaiser Energy operator
#     y[1:-1] = x[1:-1] * x[1:-1] - x[:-2] * x[2:]
#     # correct the data in the extremities
#     y[0], y[-1] = y[1], y[-2]
#     return y


def proc_emg(index):
    def handler(emg_data):
        address = "/emg" + str(index)
        osc_client.send_message(address, emg_data)

    return handler  # should return the function so the *index can be identical


def proc_imu(index):
    def handler(quat_data, acc_data, gyro_data):
        address = "/acc" + str(index)
        address2 = "/quat" + str(index)
        address3 = "/gyro" + str(index)
        osc_client.send_message(address, acc_data)
        osc_client.send_message(address2, quat_data)
        osc_client.send_message(address3, gyro_data)

        address4 = "/euler" + str(index)
        address5 = "/accmag" + str(index)
        address6 = "/gyrmag" + str(index)
        roll, pitch, yaw = toEulerAngle(
            quat_data[0], quat_data[1], quat_data[2], quat_data[3]
        )
        osc_client.send_message(
            address4, (roll / math.pi, pitch / math.pi, yaw / math.pi)
        )  # vals sent in [-1,1] (not [-pi,pi])
        osc_client.send_message(
            address5, vector_3d_magnitude(acc_data[0], acc_data[1], acc_data[2])
        )  # magnitude of accelerometer vector
        osc_client.send_message(
            address6, vector_3d_magnitude(gyro_data[0], gyro_data[1], gyro_data[2])
        )  # magnitude of gyroscope vector

    return handler


def add(index):
    m = Myo(adapter=BT(tty="/dev/" + TTY[index], baudrate=115200))

    m.add_emg_handler(proc_emg(index))
    m.add_imu_handler(proc_imu(index))

    m.connect(address=MAC[index])
    m.sleep_mode(1)
    m.set_mode(
        EMG_Mode.send_emg.value,
        IMU_Mode.send_data.value,
        Classifier_Mode.disabled.value,
    )
    m.vibrate(1)
    myo.append(m)


def loop(index):
    try:
        print("myo {} starts at: {}".format(index, time.time()))
        while True:
            myo[index].run()

    except KeyboardInterrupt:
        pass
    finally:
        # myo[index].emg_to_csv()  # editz
        myo[index].disconnect()
        print("\nDisconnected")


# buraya ne alabilirz


def main():
    add(0)
    # add(1)
    t0 = Process(target=loop, args=(0,))
    t1 = Process(target=loop, args=(1,))
    t0.start()
    # t1.start()
    # t1.join()
    t0.join()


if __name__ == "__main__":
    main()
