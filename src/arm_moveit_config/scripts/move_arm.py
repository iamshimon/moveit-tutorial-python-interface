#!/usr/bin/env python
# Author: Shimon Payyanadan

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from rospy.exceptions import ROSInitException
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveitArmTutorial(object):

    def __init__(self):
            moveit_commander.roscpp_initialize(sys.argv) #Initializing moveit_commander

            rospy.init_node('move_group_python_interface_demo', anonymous= True) #start a rospy node


            #Initialize objects for robot, scene andself.group
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()

            self.group_name = 'arm'
            self.group = moveit_commander.MoveGroupCommander(self.group_name)


            #we create display_trajectory_publisher just to visualize a trajectory in Rviz. This is optional
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

    #Give the joint positions as input
    def move_to_joint_goal(self):
        joint_pose = self.group.get_current_joint_values() #Get current joint values from theself.group
        joint_goal = joint_pose

        #Set new joint goal values
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/4
        joint_goal[4] = 0
        joint_goal[5] = pi/3

        self.group.go(joint_goal, wait= True) #Execute the joint goal target

        self.group.stop() #Ensures that there is no residual movement

        self.group.clear_pose_targets()

    #Give the end-effector pose as input
    def move_to_pose_goal(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x= 0.4
        pose_goal.position.y = 0.3
        pose_goal.position.z = 0.2

        self.group.set_pose_target(pose_goal) #set the pose target for the group

        self.group.go(wait= True)

        self.group.stop()
        self.group.clear_pose_targets() #It is good to clear targets after execution


    #Give cartesian path
    def move_cartesian_path(self):

        waypoints = []
        scale = 2

        wpose =self.group.get_current_pose().pose
        wpose.position.z -= scale * 0.1 #Frist move up (z)
        wpose.position.y += scale * 0.2 # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        #Plan the path, Note : Just planning, not executing
        (plan, fraction) = self.group.compute_cartesian_path( waypoints, 0.01, 0.0) #waypoints to follow, eef_Step, jump_threshold
        self.group.execute(plan , wait = True) #move the robot 

    #This function will add an  object to the scene
    def add_object_to_scene(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'finger1'
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z += 0.05
        box_name = "box"
        self.scene.add_box(box_name, box_pose,(0.05, 0.05, 0.05))

        return self.wait_for_state_update(box_is_known=True)


    #Wait for collision to confirm box is added

    def wait_for_state_update(self, box_is_known = False, box_is_attached = False):
        start = rospy.get_time()
        seconds = rospy.get_time()

        box_name = 'box'

        timeout = 5
        while(seconds - start < timeout ) and not rospy.is_shutdown():
            # see if box is in attached objects
            attached_objs = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objs.keys()) > 0

            #See if box is in known_objetcs
            is_known = box_name in self.scene.get_known_object_names()

            #See if we are in the expected state
            if(is_attached == box_is_attached ) and (is_known == box_is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def attach_box(self):
        
        grasping_group = 'gripper'
        touch_links = self.robot.get_link_names(group= grasping_group)
        touch_links.append('link6')
        self.scene.attach_box('finger1', 'box', touch_links= touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False)


    def detach_box(self):
        self.scene.remove_attached_object('finger1', 'box')

        return self.wait_for_state_update(box_is_attached=False, box_is_known=True)

    def remove_box(self):
        self.scene.remove_world_object('box')

        
def main():
    try:

        arm = MoveitArmTutorial()

        arm.move_to_pose_goal()
        rospy.sleep(2)
        arm.add_object_to_scene()
        rospy.sleep(2)
        arm.attach_box()
        rospy.sleep(2)
        arm.move_to_joint_goal()
        rospy.sleep(2)
        arm.detach_box()
        rospy.sleep(2)
        arm.remove_box()
        
       

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()



