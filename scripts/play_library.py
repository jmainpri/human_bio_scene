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
#                                          Jim Mainprice on Monday June 03 2016

import numpy as np
import rospy
import os
import rospkg
from play_trajectories import *


class PlayLibrary:
    def __init__(self, trajectory_set="testing"):
        self.rospack = rospkg.RosPack()
        self.directory = (self.rospack.get_path("human_bio_dataset") +
                          "/" + trajectory_set + "/")
        self.human = {}
        self.human[0] = "human_one/"
        self.human[1] = "human_two/"
        self.xml_description = rospy.get_param("/human_1/robot_description")
        print self.directory

    def list_folder(self, human_id):
        trajectory_directory = (self.directory + self.human[human_id])
        # sorting will keep the indices correct
        trajectory_files = sorted(os.listdir(trajectory_directory))
        for i, f in enumerate(trajectory_files):
            trajectory_files[i] = trajectory_directory + f
        return trajectory_files

    def play_trajectory(self, human1_file, human2_file):
        print "load files"
        player = PlayFile()
        player.set_all_joint_names(self.xml_description)
        player.load_files(human1_file, human2_file)
        player.play_skeleton()

    def play_all_trajectories(self):
        human1_files = self.list_folder(0)
        human2_files = self.list_folder(1)
        for f1, f2 in zip(human1_files, human2_files):
            self.play_trajectory(f1, f2)
            if rospy.is_shutdown():
                break

    def run_node(self):
        rospy.init_node('human_bio_library_player')

        print "start thread"
        thread_player = threading.Thread(target=self.play_all_trajectories)
        thread_player.start()

        print "spin"
        rospy.spin()


if __name__ == "__main__":

    trajectory_set = "testing"
    for index in range(1, len(sys.argv)):
        if sys.argv[index] == "--trajectory_set" and index + 1 < len(sys.argv):
            trajectory_set = str(sys.argv[index + 1])

    player = PlayLibrary(trajectory_set)
    player.run_node()
