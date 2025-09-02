#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Essential behavior classes for LLM-driven behavior trees
# @author   LLM Behavior Tree Implementation

import py_trees
import rospy
from std_msgs.msg import String
import time

# Mock robot interface for testing
class MockRobot:
    """Mock robot interface for testing behavior trees"""
    
    def __init__(self):
        self.current_position = [0.0, 0.0, 0.3]
        self.gripper_open = True
        rospy.loginfo("Mock robot initialized")
    
    def moveTo(self, x, y, z):
        """Move robot to specified position"""
        rospy.loginfo(f"Mock robot moving to ({x}, {y}, {z})")
        self.current_position = [x, y, z]
        rospy.sleep(1.0)  # Simulate movement time
        return True
    
    def close_gripper(self):
        """Close the robot gripper"""
        rospy.loginfo("Mock robot closing gripper")
        self.gripper_open = False
        rospy.sleep(0.5)
        return True
    
    def open_gripper(self):
        """Open the robot gripper"""
        rospy.loginfo("Mock robot opening gripper")
        self.gripper_open = True
        rospy.sleep(0.5)
        return True


class DetectObject(py_trees.behaviour.Behaviour):
    """Behavior for detecting objects in the environment"""
    
    def __init__(self, name, object_name, robot):
        super(DetectObject, self).__init__(name)
        self.object_name = object_name
        self.robot = robot
        self.detected = False
        self.detection_timeout = 10.0
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        
    def setup(self, timeout):
        self.detection_sub = rospy.Subscriber(
            '/detected_objects', String, self._detection_callback
        )
        return True
        
    def initialise(self):
        self.detected = False
        self.start_time = time.time()
        rospy.loginfo(f"Starting detection of {self.object_name}")
        # Publish status update
        status_msg = f"{self.name}:running:detecting"
        self.status_pub.publish(String(status_msg))
        
    def update(self):
        if time.time() - self.start_time > self.detection_timeout:
            rospy.logwarn(f"Detection timeout for {self.object_name}")
            # Publish failure status
            status_msg = f"{self.name}:failed:timeout"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.FAILURE
            
        if self.detected:
            rospy.loginfo(f"Successfully detected {self.object_name}")
            # Publish success status
            status_msg = f"{self.name}:completed:detected"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        if hasattr(self, 'detection_sub'):
            self.detection_sub.unregister()
            
    def _detection_callback(self, msg):
        if self.object_name.lower() in msg.data.lower():
            self.detected = True


class PickObject(py_trees.behaviour.Behaviour):
    """Behavior for picking up objects"""
    
    def __init__(self, name, object_name, robot):
        super(PickObject, self).__init__(name)
        self.object_name = object_name
        self.robot = robot
        self.current_step = 0
        self.steps = ['approach', 'grasp', 'lift']
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        
    def setup(self, timeout):
        return True
        
    def initialise(self):
        self.current_step = 0
        rospy.loginfo(f"Starting pick operation for {self.object_name}")
        # Publish status update
        status_msg = f"{self.name}:running:starting"
        self.status_pub.publish(String(status_msg))
        
    def update(self):
        if self.current_step >= len(self.steps):
            rospy.loginfo(f"Pick operation completed for {self.object_name}")
            # Publish completion status
            status_msg = f"{self.name}:completed:finished"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.SUCCESS
            
        step = self.steps[self.current_step]
        rospy.loginfo(f"Pick step {self.current_step + 1}/{len(self.steps)}: {step}")
        
        # Publish step status
        status_msg = f"{self.name}:running:{step}"
        self.status_pub.publish(String(status_msg))
        
        if step == 'approach':
            success = self.robot.moveTo(0.5, 0.0, 0.1)
            if success:
                self.current_step += 1
        elif step == 'grasp':
            success = self.robot.close_gripper()
            if success:
                self.current_step += 1
        elif step == 'lift':
            success = self.robot.moveTo(0.5, 0.0, 0.3)
            if success:
                self.current_step += 1
                
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        pass


class PlaceObject(py_trees.behaviour.Behaviour):
    """Behavior for placing objects"""
    
    def __init__(self, name, object_name, robot):
        super(PlaceObject, self).__init__(name)
        self.object_name = object_name
        self.robot = robot
        self.current_step = 0
        self.steps = ['approach_target', 'place', 'retreat']
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        
    def setup(self, timeout):
        return True
        
    def initialise(self):
        self.current_step = 0
        rospy.loginfo(f"Starting place operation for {self.object_name}")
        # Publish status update
        status_msg = f"{self.name}:running:starting"
        self.status_pub.publish(String(status_msg))
        
    def update(self):
        if self.current_step >= len(self.steps):
            rospy.loginfo(f"Place operation completed for {self.object_name}")
            # Publish completion status
            status_msg = f"{self.name}:completed:finished"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.SUCCESS
            
        step = self.steps[self.current_step]
        rospy.loginfo(f"Place step {self.current_step + 1}/{len(self.steps)}: {step}")
        
        # Publish step status
        status_msg = f"{self.name}:running:{step}"
        self.status_pub.publish(String(status_msg))
        
        if step == 'approach_target':
            success = self.robot.moveTo(0.0, 0.5, 0.1)
            if success:
                self.current_step += 1
        elif step == 'place':
            success = self.robot.open_gripper()
            if success:
                self.current_step += 1
        elif step == 'retreat':
            success = self.robot.moveTo(0.0, 0.5, 0.3)
            if success:
                self.current_step += 1
                
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        pass


class DetectTarget(py_trees.behaviour.Behaviour):
    """Behavior for detecting target locations"""
    
    def __init__(self, name, target_name, robot):
        super(DetectTarget, self).__init__(name)
        self.target_name = target_name
        self.robot = robot
        self.detected = False
        self.detection_timeout = 10.0
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        
    def setup(self, timeout):
        self.target_sub = rospy.Subscriber(
            '/detected_targets', String, self._target_callback
        )
        return True
        
    def initialise(self):
        self.detected = False
        self.start_time = time.time()
        rospy.loginfo(f"Starting detection of target {self.target_name}")
        # Publish status update
        status_msg = f"{self.name}:running:detecting"
        self.status_pub.publish(String(status_msg))
        
    def update(self):
        if time.time() - self.start_time > self.detection_timeout:
            rospy.logwarn(f"Target detection timeout for {self.target_name}")
            # Publish failure status
            status_msg = f"{self.name}:failed:timeout"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.FAILURE
            
        if self.detected:
            rospy.loginfo(f"Successfully detected target {self.target_name}")
            # Publish success status
            status_msg = f"{self.name}:completed:detected"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        if hasattr(self, 'target_sub'):
            self.target_sub.unregister()
            
    def _target_callback(self, msg):
        if self.target_name.lower() in msg.data.lower():
            self.detected = True


class SafetyCheck(py_trees.behaviour.Behaviour):
    """Behavior for monitoring safety conditions"""
    
    def __init__(self, name, robot):
        super(SafetyCheck, self).__init__(name)
        self.robot = robot
        self.safety_status = "safe"
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        
    def setup(self, timeout):
        self.safety_sub = rospy.Subscriber(
            '/safety_status', String, self._safety_callback
        )
        return True
        
    def initialise(self):
        rospy.loginfo("Starting safety monitoring")
        # Publish status update
        status_msg = f"{self.name}:running:monitoring"
        self.status_pub.publish(String(status_msg))
        
    def update(self):
        if self.safety_status == "safe":
            # Publish success status
            status_msg = f"{self.name}:completed:safe"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.SUCCESS
        else:
            rospy.logwarn(f"Safety check failed: {self.safety_status}")
            # Publish failure status
            status_msg = f"{self.name}:failed:unsafe"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        if hasattr(self, 'safety_sub'):
            self.safety_sub.unregister()
            
    def _safety_callback(self, msg):
        self.safety_status = msg.data


class WaitForCommand(py_trees.behaviour.Behaviour):
    """Behavior for waiting for LLM commands"""
    
    def __init__(self, name):
        super(WaitForCommand, self).__init__(name)
        self.command_received = False
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        
    def setup(self, timeout):
        self.command_sub = rospy.Subscriber(
            '/llm_command', String, self._command_callback
        )
        return True
        
    def initialise(self):
        self.command_received = False
        rospy.loginfo("Waiting for LLM command...")
        # Publish status update
        status_msg = f"{self.name}:running:waiting"
        self.status_pub.publish(String(status_msg))
        
    def update(self):
        if self.command_received:
            rospy.loginfo("LLM command received")
            # Publish success status
            status_msg = f"{self.name}:completed:received"
            self.status_pub.publish(String(status_msg))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        if hasattr(self, 'command_sub'):
            self.command_sub.unregister()
            
    def _command_callback(self, msg):
        self.command_received = True

