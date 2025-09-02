#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Simple Vision Simulator for Testing LLM Tree Generator
# @author   LLM Behavior Tree Implementation

import rospy
import random
import time
from std_msgs.msg import String

class VisionSimulator:
    """Simple vision simulator for testing behavior trees"""
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('vision_simulator', anonymous=True)
        
        # Publishers for vision topics
        self.objects_pub = rospy.Publisher('/detected_objects', String, queue_size=10)
        self.targets_pub = rospy.Publisher('/detected_targets', String, queue_size=10)
        self.safety_pub = rospy.Publisher('/safety_status', String, queue_size=10)
        
        # Simulated objects and targets
        self.objects = ["red cube", "blue sphere", "green box", "yellow cube", "orange ball"]
        self.targets = ["green box", "red container", "blue platform", "yellow shelf", "purple bin"]
        
        rospy.loginfo("Vision simulator initialized")
        
    def simulate_object_detection(self):
        """Simulate object detection"""
        while not rospy.is_shutdown():
            # Randomly select an object
            object_name = random.choice(self.objects)
            position = f"({random.uniform(0.3, 0.7):.2f}, {random.uniform(0.2, 0.8):.2f}, {random.uniform(0.05, 0.15):.2f})"
            
            message = f"Detected {object_name} at position {position}"
            self.objects_pub.publish(String(message))
            
            rospy.loginfo(f"Published: {message}")
            rospy.sleep(random.uniform(2.0, 5.0))  # Random interval
            
    def simulate_target_detection(self):
        """Simulate target detection"""
        while not rospy.is_shutdown():
            # Randomly select a target
            target_name = random.choice(self.targets)
            position = f"({random.uniform(0.1, 0.9):.2f}, {random.uniform(0.1, 0.9):.2f}, {random.uniform(0.05, 0.15):.2f})"
            
            message = f"Detected {target_name} at position {position}"
            self.targets_pub.publish(String(message))
            
            rospy.loginfo(f"Published: {message}")
            rospy.sleep(random.uniform(3.0, 6.0))  # Random interval
            
    def simulate_safety_status(self):
        """Simulate safety status"""
        while not rospy.is_shutdown():
            # Mostly safe, occasionally unsafe
            if random.random() < 0.9:  # 90% safe
                status = "safe"
            else:
                status = "unsafe - human detected"
                
            self.safety_pub.publish(String(status))
            rospy.loginfo(f"Safety status: {status}")
            rospy.sleep(random.uniform(1.0, 3.0))  # Random interval
            
    def run(self):
        """Run the vision simulator"""
        try:
            # Start simulation threads
            import threading
            
            object_thread = threading.Thread(target=self.simulate_object_detection)
            target_thread = threading.Thread(target=self.simulate_target_detection)
            safety_thread = threading.Thread(target=self.simulate_safety_status)
            
            object_thread.daemon = True
            target_thread.daemon = True
            safety_thread.daemon = True
            
            object_thread.start()
            target_thread.start()
            safety_thread.start()
            
            rospy.loginfo("Vision simulator running...")
            
            # Keep main thread alive
            while not rospy.is_shutdown():
                rospy.sleep(1.0)
                
        except rospy.ROSInterruptException:
            rospy.loginfo("Vision simulator interrupted")


def main():
    """Main function"""
    try:
        simulator = VisionSimulator()
        simulator.run()
    except Exception as e:
        rospy.logerr(f"Vision simulator error: {e}")


if __name__ == "__main__":
    main()
