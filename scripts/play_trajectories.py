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

from human_bio_scene import *
import sys

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