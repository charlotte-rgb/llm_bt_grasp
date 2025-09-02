#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Simple Robot Controller for Testing Generated Behavior Trees
# @author   LLM Behavior Tree Implementation

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point

class MockRobotController:
    """
    Mock robot controller for testing generated behavior trees
    """
    
    def __init__(self, robot_mode="virtual"):
        self.robot_mode = robot_mode
        self.current_position = [0.0, 0.0, 0.3]  # Start position
        self.gripper_open = True
        
        # Publishers for robot status
        self.position_pub = rospy.Publisher('/robot_position', Point, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_status', String, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        
        # Publish initial status
        self._publish_status()
        
        rospy.loginfo(f"Mock robot controller initialized in {robot_mode} mode")
        
    def moveTo(self, x, y, z):
        """
        Move robot to specified position
        
        Args:
            x, y, z: Target coordinates
            
        Returns:
            bool: True if movement successful
        """
        try:
            rospy.loginfo(f"Moving robot to position ({x}, {y}, {z})")
            
            # Simulate movement time
            movement_time = np.sqrt((x - self.current_position[0])**2 + 
                                  (y - self.current_position[1])**2 + 
                                  (z - self.current_position[2])**2) / 0.1  # 0.1 m/s speed
            
            # Update position
            self.current_position = [x, y, z]
            
            # Publish new position
            self._publish_status()
            
            rospy.loginfo(f"Movement completed in {movement_time:.2f} seconds")
            return True
            
        except Exception as e:
            rospy.logerr(f"Movement failed: {e}")
            return False
    
    def close_gripper(self):
        """Close the robot gripper"""
        try:
            rospy.loginfo("Closing gripper")
            self.gripper_open = False
            self._publish_status()
            rospy.sleep(0.5)  # Simulate gripper action time
            rospy.loginfo("Gripper closed")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to close gripper: {e}")
            return False
    
    def open_gripper(self):
        """Open the robot gripper"""
        try:
            rospy.loginfo("Opening gripper")
            self.gripper_open = True
            self._publish_status()
            rospy.sleep(0.5)  # Simulate gripper action time
            rospy.loginfo("Gripper opened")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to open gripper: {e}")
            return False
    
    def _publish_status(self):
        """Publish current robot status"""
        # Publish position
        position_msg = Point()
        position_msg.x = self.current_position[0]
        position_msg.y = self.current_position[1]
        position_msg.z = self.current_position[2]
        self.position_pub.publish(position_msg)
        
        # Publish gripper status
        gripper_msg = String()
        gripper_msg.data = "open" if self.gripper_open else "closed"
        self.gripper_pub.publish(gripper_msg)
        
        # Publish overall status
        status_msg = String()
        status_msg.data = f"Robot at ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}), gripper {gripper_msg.data}"
        self.status_pub.publish(status_msg)
    
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Keep publishing status
            self._publish_status()
            rate.sleep()


def main():
    """Main function"""
    # Initialize ROS node
    rospy.init_node('mock_robot_controller', anonymous=True)
    
    # Get parameters
    robot_mode = rospy.get_param('~robot_mode', 'virtual')
    
    # Create robot controller
    robot = MockRobotController(robot_mode)
    
    # Run the controller
    try:
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot controller interrupted")


if __name__ == "__main__":
    main()
