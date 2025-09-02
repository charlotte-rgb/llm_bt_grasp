#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Demo script for Status Query System
# @author   LLM Behavior Tree Implementation

import rospy
import time
from std_msgs.msg import String

class StatusSystemDemo:
    """Demo class for the status query system"""
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('status_system_demo', anonymous=True)
        
        # Publisher for status queries
        self.query_pub = rospy.Publisher('/status_query', String, queue_size=10)
        
        # Subscriber for status responses
        self.response_sub = rospy.Subscriber('/status_response', String, self._response_callback)
        
        # Store responses
        self.responses = []
        
        rospy.loginfo("Status System Demo initialized")
        
    def _response_callback(self, msg):
        """Handle status response messages"""
        self.responses.append(msg.data)
        print(f"\n[RESPONSE] {msg.data}")
    
    def run_demo(self):
        """Run the status query demo"""
        print("\n" + "="*60)
        print("STATUS QUERY SYSTEM DEMO")
        print("="*60)
        print("This demo shows how users can query execution status")
        print("and receive natural language responses.")
        print("="*60)
        
        # Wait for system to be ready
        print("\nWaiting for status monitor to be ready...")
        rospy.sleep(2.0)
        
        # Example queries
        demo_queries = [
            "What is the current progress?",
            "What is the robot doing right now?",
            "How long has the task been running?",
            "What is the robot status?",
            "Is the vision system working?",
            "Are there any safety issues?",
            "What behaviors have been completed?",
            "Have any behaviors failed?",
            "Give me a status overview"
        ]
        
        print(f"\nRunning {len(demo_queries)} demo queries...")
        
        for i, query in enumerate(demo_queries, 1):
            print(f"\n--- Query {i}/{len(demo_queries)} ---")
            print(f"Query: {query}")
            
            # Send query
            self.query_pub.publish(String(query))
            
            # Wait for response
            rospy.sleep(1.0)
            
            # Small delay between queries
            if i < len(demo_queries):
                rospy.sleep(0.5)
        
        print("\n" + "="*60)
        print("DEMO COMPLETED")
        print("="*60)
        print(f"Total responses received: {len(self.responses)}")
        
        if self.responses:
            print("\nAll responses:")
            for i, response in enumerate(self.responses, 1):
                print(f"{i}. {response}")
        
        print("\n" + "="*60)
    
    def run_interactive_demo(self):
        """Run interactive demo where user can input queries"""
        print("\n" + "="*60)
        print("INTERACTIVE STATUS QUERY DEMO")
        print("="*60)
        print("Type your own status queries!")
        print("Type 'quit' to exit")
        print("Type 'help' for example queries")
        print("="*60)
        
        example_queries = [
            "What is the current progress?",
            "What is the robot doing right now?",
            "How long has the task been running?",
            "What is the robot status?",
            "Is the vision system working?",
            "Are there any safety issues?",
            "What behaviors have been completed?",
            "Have any behaviors failed?",
            "Give me a status overview"
        ]
        
        while not rospy.is_shutdown():
            try:
                # Get user input
                user_input = input("\nEnter your status query: ").strip()
                
                if user_input.lower() == 'quit':
                    print("Goodbye!")
                    break
                elif user_input.lower() == 'help':
                    self._show_help(example_queries)
                elif user_input:
                    # Send query
                    print(f"Sending query: {user_input}")
                    self.query_pub.publish(String(user_input))
                    
                    # Wait for response
                    rospy.sleep(1.0)
                else:
                    print("Please enter a valid query")
                    
            except KeyboardInterrupt:
                print("\nInterrupted by user")
                break
            except EOFError:
                print("\nEnd of input")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def _show_help(self, example_queries):
        """Show help and example queries"""
        print("\n" + "="*60)
        print("HELP & EXAMPLE QUERIES")
        print("="*60)
        print("You can ask about various aspects of the robot's execution:")
        print("\nProgress & Status:")
        print("• 'What is the current progress?'")
        print("• 'How far along is the task?'")
        print("• 'What is the robot doing right now?'")
        print("• 'What is currently happening?'")
        
        print("\nTime & Duration:")
        print("• 'How long has the task been running?'")
        print("• 'What is the elapsed time?'")
        
        print("\nRobot Status:")
        print("• 'What is the robot status?'")
        print("• 'Is the robot moving?'")
        print("• 'What is the gripper doing?'")
        
        print("\nVision & Detection:")
        print("• 'Is the vision system working?'")
        print("• 'What is the camera status?'")
        print("• 'Has anything been detected?'")
        
        print("\nSafety & Monitoring:")
        print("• 'Are there any safety issues?'")
        print("• 'Is the workspace safe?'")
        print("• 'What is the safety status?'")
        
        print("\nCompletion & Results:")
        print("• 'What behaviors have been completed?'")
        print("• 'What has been done so far?'")
        print("• 'Have any behaviors failed?'")
        print("• 'Are there any errors?'")
        
        print("\nGeneral Overview:")
        print("• 'Give me a status overview'")
        print("• 'What is the overall status?'")
        print("• 'Summarize the current situation'")
        
        print("\n" + "="*60)


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Status System Demo')
    parser.add_argument('--interactive', '-i', action='store_true', 
                       help='Run in interactive mode')
    
    args = parser.parse_args()
    
    try:
        # Create demo instance
        demo = StatusSystemDemo()
        
        if args.interactive:
            # Interactive mode
            demo.run_interactive_demo()
        else:
            # Automatic demo mode
            demo.run_demo()
            
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Demo error: {e}")


if __name__ == "__main__":
    main()
