#!/usr/bin/env python

import rospy
import sys
from nav_module import Goal2D
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool

PI = 3.14


class JackalNavigator:
    def __init__(self, name):
        self.name = name
        self.pub = rospy.Publisher(name + '/goal', MoveBaseGoal, queue_size=1)
        self.sub = rospy.Subscriber(name + '/res', Bool, self.resultCallback())
        self.results = []
        self.goals = []
        self.tasks = []
        self.finished_goals = 0

    def resultCallback(self, res):
        if(not res):
            rospy.logerr('{} unable to reach a target goal.'.format(self.name))
            rospy.signal_shutdown()
        self.results.append(res)
        self.finished_goals = self.finished_goals + 1
        rospy.loginfo('{} reached goal'.format(self.name))

    def addGoals(self, goals):
        for goal in goals:
            self.goals.append(goal)

    def addTasks(self, tasks):
        for task in tasks:
            self.tasks.append(task)
    
        
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('multi_jackal_navigation')

        jackals = []
        for name in sys.argv[1:]:
            jackals.append(JackalNavigator(name))

        # Some test goals for simulation
        tst_right = Goal2D(-3.0, -.8, PI/2, 'map')
        tst_left = Goal2D(-6.0, -2.0, -PI/2, 'map')
        tst_up = Goal2D(-8.2, -1.0, 1.6, 'map')
        tst_left2 = Goal2D(-6.0, 1.5, 0.0, 'map')

        # Jackal acoustic IM goals and tasks
        jackals[0].addGoals([tst_right, tst_left])
        jackals[0].addTasks(['listen', 'speak'])

        if(len(jackals) > 1):
            jackals[1].addGoals([tst_up, tst_left2])
            jackals[1].addTasks(['speak', 'listen'])
        
        rospy.loginfo('Pause while goal publisher becomes recognized...')
        rospy.sleep(5)
        rospy.loginfo('...finished pausing')

        num_goals = len(jackals[0].goals)
        for i in range(num_goals):
            
            # Move to goal
            for jackal in jackals:
                rospy.loginfo('Sending goal to {}'.format(jackal.name))
                jackal.pub.publish(jackal.goals[i].to_move_base())

            # Wait for results
            # TODO improve this implementation
            rospy.loginfo('Waiting for results from Jackals')
            for jackal in jackals:
                while(jackal.finished_goals < i+1):
                    continue

            # Perform task
            for jackal in jackals:
                rospy.loginfo('{} task: {}'.format(jackal.name, jackal.tasks[i]))
            rospy.sleep(3)

            # Reverse from wall?

        rospy.loginfo('Finished goal sequence.')
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption in single_jackal_navigation")
