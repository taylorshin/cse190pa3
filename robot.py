#!/usr/bin/env python
import rospy
import math
import random
import numpy as np
from copy import deepcopy
from random import randint
from std_msgs.msg import Bool
from cse_190_assi_3.msg import AStarPath, PolicyList
from read_config import read_config
from astar import astar
from mdp import mdp

class Robot:
    def __init__(self):
        self.config = read_config()
        rospy.init_node('robot', anonymous=True)

        self.astar_publisher = rospy.Publisher(
                '/results/path_list',
                AStarPath,
                queue_size = 1
        )

        self.mdp_publisher = rospy.Publisher(
                '/results/policy_list',
                PolicyList,
                queue_size = 1
        )

        self.complete_publisher = rospy.Publisher(
                '/map_node/sim_complete',
                Bool,
                queue_size = 1
        )

        # Read in input parameters from config file
        self.move_list = self.config['move_list']
        self.map_size = self.config['map_size']
        self.start = self.config['start']
        self.goal = self.config['goal']
        self.walls = self.config['walls']
        self.pits = self.config['pits']

        # A*
        self.astar_path()

        # MDP
        self.mdp_list()

        # Publish to sim_complete before shutdown for json result files
        rospy.sleep(1)
        self.complete_publisher.publish(True)
        rospy.sleep(1) 
        rospy.signal_shutdown(self)

    """ Callback that uses the A* algorithm to find optimal path """
    def astar_path(self):
        # Get optimal path sequence
        path = astar(self.move_list, self.map_size, self.start, self.goal, self.walls, self.pits)
        for p in path:
            if p:
                rospy.sleep(1)
                self.astar_publisher.publish(list(p))

    """ Callback that uses MDP """
    def mdp_list(self):
        policy_iterations = mdp()
        for iteration in policy_iterations:
            policy_list = sum(iteration, [])
            #print policy_list
            rospy.sleep(1)
            self.mdp_publisher.publish(policy_list)

if __name__ == '__main__':
    try:
        robot = Robot()
    except rospy.ROSInterruptException:
        pass
