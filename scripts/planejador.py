#!/usr/bin/env python
#coding: utf-8

import rospy
from custom_msgs_srvs.srv import Plan, PlanRequest, TrajectoryHandleService, TrajectoryHandleServiceRequest
from custom_msgs_srvs.msg import TrajectoryHandle
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PoseStamped


class PlanningManager:
    def __init__(self, planner):
        self.drone_pos = None
        self.goal_pos = None
        self.occ_grid = None

        map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        goal_sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        self.plan_pub = rospy.Publisher('path', TrajectoryHandle, queue_size=10)

        self.available_planners = {}
        rospy.wait_for_service('grid_planner')
        grid_planner = rospy.ServiceProxy('grid_planner', Plan)
        self.available_planners['grid_planner'] = grid_planner
        # faça o mesmo para outros planners que forem criados

        try:
            self.planner = self.available_planners[planner]
        except KeyError as e:
            rospy.logerr('The requested planner is not available.')

        rospy.wait_for_service('/red/avant_cmd/exec_trajectory')
        self.exec_trajectory = rospy.ServiceProxy('/red/avant_cmd/exec_trajectory', TrajectoryHandleService)

    def map_callback(self, msg):
        rospy.logdebug('Map received.')
        self.occ_grid = msg

    def odom_callback(self, msg):
        self.drone_pos = msg.pose.pose.position


    def goal_callback(self, msg):
        self.goal_pos = msg.pose.position
        rospy.loginfo('Goal position received. Proceeding to planning pipeline.')
        self.request_plan(self.drone_pos, self.goal_pos)

    def request_plan(self, start_position, goal_position):
        if self.occ_grid is None:
            rospy.logerr('No map received. Unable to plan.')
            return
        
        if start_position is None or goal_position is None:
            rospy.logerr('Start or goal position is unknown. Unable to plan.')
            return

        request = PlanRequest()
        request.occ_grid = self.occ_grid
        request.start_position = start_position
        request.goal_position = goal_position
        resp = self.planner(request)
        
        # self.plan_pub.publish(resp.trajectory)    NAO APAGA
        msg = TrajectoryHandleServiceRequest()
        msg.points = resp.trajectory.points
        self.exec_trajectory(msg)

rospy.init_node('planning_manager')
planner = rospy.get_param(param_name="planning_manager/planner", default="grid_planner")
manager = PlanningManager(planner)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()