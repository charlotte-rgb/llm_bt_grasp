#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Status Monitor for Behavior Tree Execution
# @author   LLM Behavior Tree Implementation

import rospy
import time
from std_msgs.msg import String
from .behaviors import *

class ExecutionStatus:
    """Represents the current execution status of a behavior tree"""
    
    def __init__(self):
        self.current_behavior = None
        self.current_step = None
        self.completed_behaviors = []
        self.failed_behaviors = []
        self.start_time = None
        self.estimated_completion = None
        self.robot_status = "idle"
        self.vision_status = "scanning"
        self.safety_status = "safe"
        self.overall_progress = 0.0
        
    def update_behavior(self, behavior_name, step=None, status="running"):
        """Update the current behavior status"""
        self.current_behavior = behavior_name
        self.current_step = step
        if status == "completed":
            self.completed_behaviors.append(behavior_name)
        elif status == "failed":
            self.failed_behaviors.append(behavior_name)
        
        # Calculate progress
        total_behaviors = len(self.completed_behaviors) + len(self.failed_behaviors) + 1
        self.overall_progress = (len(self.completed_behaviors) / total_behaviors) * 100.0
        
    def get_status_summary(self):
        """Get a summary of current execution status"""
        return {
            'current_behavior': self.current_behavior,
            'current_step': self.current_step,
            'completed': self.completed_behaviors,
            'failed': self.failed_behaviors,
            'progress': self.overall_progress,
            'robot_status': self.robot_status,
            'vision_status': self.vision_status,
            'safety_status': self.safety_status,
            'elapsed_time': time.time() - self.start_time if self.start_time else 0
        }


class StatusMonitor:
    """Monitors and reports behavior tree execution status"""
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('status_monitor', anonymous=True)
        
        # Status tracking
        self.execution_status = ExecutionStatus()
        self.execution_status.start_time = time.time()
        
        # Subscribers for status updates
        self.behavior_sub = rospy.Subscriber('/behavior_status', String, self._behavior_callback)
        self.robot_sub = rospy.Subscriber('/robot_status', String, self._robot_callback)
        self.vision_sub = rospy.Subscriber('/vision_status', String, self._vision_callback)
        self.safety_sub = rospy.Subscriber('/safety_status', String, self._safety_callback)
        
        # Publishers for status responses
        self.status_response_pub = rospy.Publisher('/status_response', String, queue_size=10)
        
        # Status query service
        self.query_sub = rospy.Subscriber('/status_query', String, self._query_callback)
        
        rospy.loginfo("Status monitor initialized")
        
    def _behavior_callback(self, msg):
        """Handle behavior status updates"""
        try:
            # Parse behavior status message
            # Format: "behavior_name:status:step"
            parts = msg.data.split(':')
            if len(parts) >= 2:
                behavior_name = parts[0]
                status = parts[1]
                step = parts[2] if len(parts) > 2 else None
                
                self.execution_status.update_behavior(behavior_name, step, status)
                rospy.loginfo(f"Behavior status: {behavior_name} - {status}")
                
        except Exception as e:
            rospy.logwarn(f"Failed to parse behavior status: {e}")
    
    def _robot_callback(self, msg):
        """Handle robot status updates"""
        self.execution_status.robot_status = msg.data
        rospy.loginfo(f"Robot status: {msg.data}")
    
    def _vision_callback(self, msg):
        """Handle vision status updates"""
        self.execution_status.vision_status = msg.data
        rospy.loginfo(f"Vision status: {msg.data}")
    
    def _safety_callback(self, msg):
        """Handle safety status updates"""
        self.execution_status.safety_status = msg.data
        rospy.loginfo(f"Safety status: {msg.data}")
    
    def _query_callback(self, msg):
        """Handle status queries from users"""
        try:
            query = msg.data.lower()
            response = self._generate_natural_response(query)
            
            # Publish response
            self.status_response_pub.publish(String(response))
            rospy.loginfo(f"Status query: {query} -> {response}")
            
        except Exception as e:
            error_msg = f"Failed to process status query: {e}"
            self.status_response_pub.publish(String(error_msg))
            rospy.logerr(error_msg)
    
    def _generate_natural_response(self, query):
        """Generate natural language response to status query"""
        status = self.execution_status.get_status_summary()
        
        # Common query patterns
        if "progress" in query or "how far" in query:
            return self._generate_progress_response(status)
        elif "current" in query or "what" in query or "doing" in query:
            return self._generate_current_status_response(status)
        elif "time" in query or "how long" in query:
            return self._generate_time_response(status)
        elif "robot" in query:
            return self._generate_robot_response(status)
        elif "vision" in query or "camera" in query:
            return self._generate_vision_response(status)
        elif "safety" in query or "safe" in query:
            return self._generate_safety_response(status)
        elif "completed" in query or "done" in query:
            return self._generate_completion_response(status)
        elif "failed" in query or "error" in query:
            return self._generate_failure_response(status)
        else:
            return self._generate_general_status_response(status)
    
    def _generate_progress_response(self, status):
        """Generate response about execution progress"""
        progress = status['progress']
        completed = len(status['completed'])
        failed = len(status['failed'])
        
        if progress == 0:
            return "The task hasn't started yet."
        elif progress == 100:
            return "The task has been completed successfully!"
        elif failed > 0:
            return f"The task is {progress:.1f}% complete. {completed} behaviors succeeded, {failed} failed. Currently working on: {status['current_behavior']}"
        else:
            return f"The task is {progress:.1f}% complete. {completed} behaviors have been completed successfully. Currently working on: {status['current_behavior']}"
    
    def _generate_current_status_response(self, status):
        """Generate response about current execution status"""
        current = status['current_behavior']
        step = status['current_step']
        
        if not current:
            return "No task is currently running."
        
        if step:
            return f"Currently executing: {current}, step: {step}. Robot status: {status['robot_status']}, Vision: {status['vision_status']}"
        else:
            return f"Currently executing: {current}. Robot status: {status['robot_status']}, Vision: {status['vision_status']}"
    
    def _generate_time_response(self, status):
        """Generate response about execution time"""
        elapsed = status['elapsed_time']
        if elapsed == 0:
            return "The task hasn't started yet."
        
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)
        
        if minutes > 0:
            return f"The task has been running for {minutes} minutes and {seconds} seconds."
        else:
            return f"The task has been running for {seconds} seconds."
    
    def _generate_robot_response(self, status):
        """Generate response about robot status"""
        robot_status = status['robot_status']
        if "moving" in robot_status.lower():
            return f"The robot is currently moving: {robot_status}"
        elif "gripper" in robot_status.lower():
            return f"The robot is manipulating the gripper: {robot_status}"
        else:
            return f"Robot status: {robot_status}"
    
    def _generate_vision_response(self, status):
        """Generate response about vision status"""
        vision_status = status['vision_status']
        if "detected" in vision_status.lower():
            return f"Vision system has detected objects: {vision_status}"
        elif "scanning" in vision_status.lower():
            return "Vision system is actively scanning the environment for objects and targets."
        else:
            return f"Vision system status: {vision_status}"
    
    def _generate_safety_response(self, status):
        """Generate response about safety status"""
        safety_status = status['safety_status']
        if "safe" in safety_status.lower():
            return "All safety systems are functioning normally. The workspace is safe for robot operation."
        else:
            return f"Safety alert: {safety_status}. Please check the workspace before proceeding."
    
    def _generate_completion_response(self, status):
        """Generate response about completed behaviors"""
        completed = status['completed']
        if not completed:
            return "No behaviors have been completed yet."
        elif len(completed) == 1:
            return f"Completed: {completed[0]}"
        else:
            return f"Completed behaviors: {', '.join(completed)}"
    
    def _generate_failure_response(self, status):
        """Generate response about failed behaviors"""
        failed = status['failed']
        if not failed:
            return "No behaviors have failed so far."
        elif len(failed) == 1:
            return f"Failed behavior: {failed[0]}"
        else:
            return f"Failed behaviors: {', '.join(failed)}"
    
    def _generate_general_status_response(self, status):
        """Generate general status overview"""
        progress = status['progress']
        current = status['current_behavior']
        elapsed = status['elapsed_time']
        
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)
        
        response = f"Task Status Overview:\n"
        response += f"• Progress: {progress:.1f}%\n"
        response += f"• Current behavior: {current or 'None'}\n"
        response += f"• Elapsed time: {minutes}m {seconds}s\n"
        response += f"• Robot: {status['robot_status']}\n"
        response += f"• Vision: {status['vision_status']}\n"
        response += f"• Safety: {status['safety_status']}\n"
        response += f"• Completed: {len(status['completed'])} behaviors\n"
        response += f"• Failed: {len(status['failed'])} behaviors"
        
        return response
    
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Keep the monitor running
            rate.sleep()


def main():
    """Main function"""
    try:
        monitor = StatusMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Status monitor interrupted")
    except Exception as e:
        rospy.logerr(f"Status monitor error: {e}")


if __name__ == "__main__":
    main()
