#!/usr//bin/env python

import paramiko
import sys
from pynput import keyboard
import time

# The key combination to check
COMBINATIONS = [
    {keyboard.KeyCode(char='`')},
    {keyboard.KeyCode(char='~')}
]

# The currently active modifiers
current = set()

def execute():
    print("send")
    ssh.exec_command("source /opt/ros/kinetic/setup.bash && cd movo_ws && source ./devel/setup.bash && rostopic pub /movo/right_arm/cartesian_vel_cmd movo_msgs/JacoCartesianVelocityCmd \"header:\n"
    + "  seq: 0\n"
    + "  stamp: now\n"
    + "  frame_id: ''\n"
    + "x: 0.0\n"
    + "y: 0.0\n"
    + "z: 0.0\n"
    + "theta_x: 0.0\n"
    + "theta_y: 0.0\n"
    + "theta_z: 0.0\"")
    #
    ssh.exec_command("source /opt/ros/kinetic/setup.bash && cd movo_ws && source ./devel/setup.bash && rostopic pub /movo/left_arm/cartesian_vel_cmd movo_msgs/JacoCartesianVelocityCmd \"header:\n"
    + "  seq: 0\n"
    + "  stamp: now\n"
    + "  frame_id: ''\n"
    + "x: 0.0\n"
    + "y: 0.0\n"
    + "z: 0.0\n"
    + "theta_x: 0.0\n"
    + "theta_y: 0.0\n"
    + "theta_z: 0.0\"")

    ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("source /opt/ros/kinetic/setup.bash && cd movo_ws && source ./devel/setup.bash && rostopic pub /movo/gp_command movo_msgs/ConfigCmd \"header:\n"
      + "  seq: 0\n"
      + "  stamp: now\n"
      + "  frame_id: ''\n"
    + "gp_cmd: 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'\n"
    + "gp_param: 1 \"")
    ssh.close()
    ssh.connect(hostname=host, username=username, password=password)
    print("done")
    # print(ssh_stderr.read())

def on_press(key):
    if (isinstance(key, keyboard._xorg.KeyCode) and key.char == '\\'):
        sys.exit("quit")
    if any([key in COMBO for COMBO in COMBINATIONS]):
        current.add(key)
        if any(all(k in current for k in COMBO) for COMBO in COMBINATIONS):
            execute()

def on_release(key):
    if any([key in COMBO for COMBO in COMBINATIONS]):
        current.remove(key)


if __name__ == "__main__":


    paramiko.util.log_to_file("paramiko.log")

    host = "10.66.171.1"
    username = "movo"
    password = "Welcome00"


    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname=host, username=username, password=password)
    print("connected")

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    #
    #
    #
    # ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("cd movo_ws && ls")
    # print(ssh_stdout.read())


