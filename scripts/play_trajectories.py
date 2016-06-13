#!/usr/bin/env python

# Copyright (c) 2015 Max Planck Institute
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                    Jim Mainprice on Wednesday January 03 2016

# ACTIVE DOFS FOR BIO MODEL
# ID(0) : Joint(1), Dof : 6, Pelvis(min = 1.92, max = 2.08) : 0.17
# ID(1) : Joint(1), Dof : 7, Pelvis(min = 0.84, max = 1.03) : 0.18
# ID(2) : Joint(1), Dof : 8, Pelvis(min = 1.09, max = 1.30) : 0.22
# ID(3) : Joint(1), Dof : 9, Pelvis(min = -0.05, max = 0.05) : 0.10
# ID(4) : Joint(1), Dof : 10, Pelvis(min = -0.05, max = 0.05) : 0.10
# ID(5) : Joint(1), Dof : 11, Pelvis(min = 1.82, max = 2.57) : 0.75
# ID(6) : Joint(2), Dof : 12, TorsoX(min = -0.10, max = 0.26) : 0.36
# ID(7) : Joint(3), Dof : 13, TorsoZ(min = -0.51, max = 0.04) : 0.54
# ID(8) : Joint(4), Dof : 14, TorsoY(min = -0.04, max = 0.55) : 0.60
# ID(9) : Joint(8), Dof : 18, rShoulderTransX(min = 0.05, max = 0.16) : 0.11
# ID(10) : Joint(9), Dof : 19, rShoulderTransY(min = 0.12, max = 0.25) : 0.13
# ID(11) : Joint(10), Dof : 20, rShoulderTransZ(min = 0.06, max = 0.19) : 0.13
# ID(12) : Joint(11), Dof : 21, rShoulderY1(min = -1.09, max = 0.97) : 2.06
# ID(13) : Joint(12), Dof : 22, rShoulderX(min = -1.48, max = -0.26) : 1.22
# ID(14) : Joint(13), Dof : 23, rShoulderY2(min = -1.14, max = 1.39): 2.53
# ID(15) : Joint(14), Dof : 24, rArmTrans(min = 0.27, max = 0.45): 0.18
# ID(16) : Joint(15), Dof : 25, rElbowZ(min = 0.51, max = 2.04): 1.53
# ID(17) : Joint(16), Dof : 26, rElbowX(min = -0.47, max = 0.32): 0.79
# ID(18) : Joint(17), Dof : 27, rElbowY(min = -3.16, max = 3.16): 6.32
# ID(19) : Joint(18), Dof : 28, rForeArmTrans(min = 0.20, max = 0.30): 0.10
# ID(20) : Joint(19), Dof : 29, rWristZ(min = -0.96, max = 0.60): 1.56
# ID(21) : Joint(20), Dof : 30, rWristX(min = -0.34, max = 0.96): 1.30
# ID(22) : Joint(21), Dof : 31, rWristY(min = -0.25, max = 0.37): 0.62

import numpy as np
import sys
import time
from copy import deepcopy
import csv
import rospy
import threading
import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False,
             "quit": "q", "q" : "q" }
    if default is None:
        prompt = " [y/n or q] "
    elif default == "yes":
        prompt = " [Y/n or q] "
    elif default == "no":
        prompt = " [y/No r q] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes', 'no' or 'quit' "
                             "(or 'y', 'n' or 'q').\n")

class PlayFile:
    def __init__(self):

        self.herakles_bio_openrave_map = {}
        self.herakles_bio_openrave_map["Time"] = -1
        self.herakles_bio_openrave_map["PelvisTransX"] = 0
        self.herakles_bio_openrave_map["PelvisTransY"] = 1
        self.herakles_bio_openrave_map["PelvisTransZ"] = 2
        self.herakles_bio_openrave_map["PelvisRotX"] = 3
        self.herakles_bio_openrave_map["PelvisRotY"] = 4
        self.herakles_bio_openrave_map["PelvisRotZ"] = 5
        self.herakles_bio_openrave_map["TorsoX"] = 6
        self.herakles_bio_openrave_map["TorsoZ"] = 7
        self.herakles_bio_openrave_map["TorsoY"] = 8
        self.herakles_bio_openrave_map["rShoulderTransX"] = 9
        self.herakles_bio_openrave_map["rShoulderTransY"] = 10
        self.herakles_bio_openrave_map["rShoulderTransZ"] = 11
        self.herakles_bio_openrave_map["rShoulderY1"] = 16
        self.herakles_bio_openrave_map["rShoulderX"] = 17
        self.herakles_bio_openrave_map["rShoulderY2"] = 18
        self.herakles_bio_openrave_map["rArmTrans"] = 19
        self.herakles_bio_openrave_map["rElbowZ"] = 20
        self.herakles_bio_openrave_map["rElbowX"] = 21
        self.herakles_bio_openrave_map["rElbowY"] = 22
        self.herakles_bio_openrave_map["rForeArmTrans"] = 23
        self.herakles_bio_openrave_map["rWristZ"] = 24
        self.herakles_bio_openrave_map["rWristX"] = 25
        self.herakles_bio_openrave_map["rWristY"] = 26

        # Add an offset for the time
        for key in self.herakles_bio_openrave_map:
            self.herakles_bio_openrave_map[key] += 1

        for key in self.herakles_bio_openrave_map:
            print key + " : " + str(self.herakles_bio_openrave_map[key])

        self.active_dofs = []
        # self.active_dofs.append("Time")
        self.active_dofs.append("PelvisTransX")
        self.active_dofs.append("PelvisTransY")
        self.active_dofs.append("PelvisTransZ")
        # self.active_dofs.append("PelvisRotX")
        # self.active_dofs.append("PelvisRotY")
        self.active_dofs.append("PelvisRotZ")
        self.active_dofs.append("TorsoX")
        self.active_dofs.append("TorsoZ")
        self.active_dofs.append("TorsoY")
        self.active_dofs.append("rShoulderTransX")
        self.active_dofs.append("rShoulderTransY")
        self.active_dofs.append("rShoulderTransZ")
        self.active_dofs.append("rShoulderY1")
        self.active_dofs.append("rShoulderX")
        self.active_dofs.append("rShoulderY2")
        self.active_dofs.append("rArmTrans")
        self.active_dofs.append("rElbowZ")
        self.active_dofs.append("rElbowX")
        self.active_dofs.append("rElbowY")
        self.active_dofs.append("rForeArmTrans")
        self.active_dofs.append("rWristZ")
        self.active_dofs.append("rWristX")
        self.active_dofs.append("rWristY")

        for j, dof in enumerate(self.active_dofs):
            print "j= {j} and dof = {dof}".format(**locals())

        self.traj_human1 = []
        self.traj_human2 = []

        # Joint state publisher variables
        self.joint_state_publishers = None
        self.joint_state_topic_names = None
        self.q_cur = []

        # All joint properties
        self.joints = {}

        # Joint state default
        joint_state_topics = ["/human_1/joint_states", "/human_2/joint_states"]
        self.set_publish_joint_state(joint_state_topics)

    def set_all_joint_names(self, description_file):

        robot = (
            xml.dom.minidom.parseString(description_file).getElementsByTagName(
                'robot')[0])

        # Create all the joints based off of the URDF and
        # assign them joint limits based on their properties
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval) / 2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval,
                         'value': zeroval}
                self.joints[name] = joint

    def set_publish_joint_state(self, joint_state_topic_names):

        self.joint_state_topic_names = joint_state_topic_names
        self.joint_state_publishers = []

        for topic in self.joint_state_topic_names:
            self.joint_state_publishers.append(
                rospy.Publisher(topic, JointState))

    def publish_joint_state(self):

        for id_human, publisher in enumerate(self.joint_state_publishers):

            joint_state = JointState()
            joint_state.header.stamp = rospy.Time().now()
            joint_state.name = [""] * len(self.joints)
            joint_state.position = [0.] * len(self.joints)

            for id_dof, joint in enumerate(self.joints):
                joint_state.name[id_dof] = joint

                # TODO REMOVE THAT SHOULDER HACK
                # Should work on left arm model to avoid setting
                # the shoulder in the parser
                if joint == "lShoulderX":
                    joint_state.position[id_dof] = -pi / 2.

                if joint in self.active_dofs:
                    i = self.active_dofs.index(joint)
                    joint_state.position[id_dof] = self.q_cur[id_human][i]

            # Publish joint states
            publisher.publish(joint_state)

    def load_files(self, h1_filepath, h2_filepath):

        print "Trying to open file: ", h1_filepath

        # Parse CSV files
        with open(h1_filepath, 'r') as h1_file:
            self.traj_human1 = [row for row in
                                csv.reader(h1_file, delimiter=',')]
            self.traj_human1 = [map(float, row) for row in
                                self.traj_human1]  # Convert to floats

        print "Trying to open file: ", h2_filepath

        # Parse CSV files
        if h2_filepath is not None:
            with open(h2_filepath, 'r') as h2_file:
                self.traj_human2 = [row for row in
                                    csv.reader(h2_file, delimiter=',')]
                self.traj_human2 = [map(float, row) for row in
                                    self.traj_human2]  # Convert to floats
        else:
            self.traj_human2 = deepcopy(self.traj_human1)

    def get_active_dof_config(self, q_full):
        q_active = np.array([0.] * len(self.active_dofs))
        for id_dof, joint in enumerate(self.active_dofs):
            q_active[id_dof] = q_full[self.herakles_bio_openrave_map[joint]]
        return q_active

    def play_skeleton(self):

        while True:

            # for frame in self.frames:
            print len(self.traj_human1)

            scale = 1.

            t0_prev_time = time.time()
            t_total = time.time()
            traj_time = 0.0
            exec_time = 0.0
            nb_overshoot = 0

            for row1, row2 in zip(self.traj_human1, self.traj_human2):

                self.q_cur = [None] * len(self.joint_state_publishers)
                self.q_cur[0] = self.get_active_dof_config(row1)
                self.q_cur[1] = self.get_active_dof_config(row2)

                if rospy.is_shutdown():
                    break

                self.publish_joint_state()

                # Execution time
                t0 = time.time()
                dt_0 = t0 - t0_prev_time

                # Trajectory time
                dt = row1[0]
                traj_time += dt

                # Sleep
                if dt < dt_0:
                    nb_overshoot += 1
                    # print "dt : " , dt , " dt0 , ", dt_0, " , t0 : %.5f" % t0
                else:
                    time.sleep(dt - dt_0)
                    # sleep only of dt > dt_0
                    # TODO: Should use C++ for good execution times
                    t0 = time.time()
                    dt_0 = t0 - t0_prev_time

                t0_prev_time = t0
                exec_time += dt_0

            print "------------------------"
            print "Total time : ", float(time.time() - t_total)
            print "Exec time : ", exec_time
            print "Traj time : ", traj_time
            print "Nb. of overshoot : ", nb_overshoot
            print "Nb. of frames : , ", len(self.traj_human1)
            print "%.4f percent of overshoot " % float(
                100. * float(nb_overshoot) / float(len(self.traj_human1)))

            answer = query_yes_no("Replay ?:")
            if answer is True:
                continue
            elif answer is "q":
                print "set signal for shutdown"
                rospy.signal_shutdown("quit by user")
                break
            else:
                break
        print "end Loop."


if __name__ == "__main__":

    use_ros = True

    for index in range(1, len(sys.argv)):

        if sys.argv[index] == "-split" and index + 1 < len(sys.argv):
            human1_file = str(sys.argv[index + 1]) + "_human1_.csv"
            human2_file = str(sys.argv[index + 1]) + "_human2_.csv"
        if sys.argv[index] == "-h1" and index + 1 < len(sys.argv):
            human1_file = str(sys.argv[index + 1])
        if sys.argv[index] == "-h2" and index + 1 < len(sys.argv):
            human2_file = str(sys.argv[index + 1])
        if sys.argv[index] == "-noros":
            use_ros = False

    if use_ros:
        rospy.init_node('human_bio_trajectory_player')

        human1_file = rospy.get_param("/human1_traj_file", None)
        human2_file = rospy.get_param("/human2_traj_file", None)

        publish_joint_state = rospy.get_param("~human_publish_joint_state",
                                              'mocap_human_joint_state')
        joint_state_topic = rospy.get_param("~human_joint_state_topic", False)
        start_openrave = rospy.get_param("~start_openrave", True)

    if human1_file is None:

        print "Usage : "
        print " -h1 /path/to/directory/file1.csv   : sets the file for human1"
        print " -h2 /path/to/directory/file1.csv   : sets the file for human2"

    else:

        print "try to load file : ", human1_file
        print "try to load file : ", human2_file

        player = PlayFile()
        player.load_files(human1_file, human2_file)

        if use_ros:
            xml_description = rospy.get_param("/human_1/robot_description")
            player.set_all_joint_names(xml_description)
            print "publish_joint_state : ", publish_joint_state

        if use_ros and publish_joint_state:
            print "start thread"
            thread_player = threading.Thread(target=player.play_skeleton)
            thread_player.start()

            print "spin"
            rospy.spin()
