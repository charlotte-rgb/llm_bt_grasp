#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Status Query CLI for Behavior Tree Execution
# @author   LLM Behavior Tree Implementation

import rospy
import sys
import time
from std_msgs.msg import String

class StatusQueryCLI:
    """Command-line interface for querying execution status"""
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('status_query_cli', anonymous=True)
        
        # Publisher for status queries
        self.query_pub = rospy.Publisher('/status_query', String, queue_size=10)
        
        # Subscriber for status responses
        self.response_sub = rospy.Subscriber('/status_response', String, self._response_callback)
        
        # Store latest response
        self.latest_response = None
        
        rospy.loginfo("Status Query CLI initialized")
        
    def _response_callback(self, msg):
        """Handle status response messages"""
        self.latest_response = msg.data
        
    def query_status(self, query):
        """Send a status query and wait for response"""
        print(f"\nQuerying: '{query}'")
        print("Waiting for response...")
        
        # Reset response
        self.latest_response = None
        
        # Publish query
        self.query_pub.publish(String(query))
        
        # Wait for response (with timeout)
        timeout = 10  # 10 seconds timeout
        start_time = time.time()
        
        while self.latest_response is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.latest_response:
            print("\n" + "="*60)
            print("STATUS RESPONSE:")
            print("="*60)
            print(self.latest_response)
            print("="*60)
            return self.latest_response
        else:
            print("Timeout waiting for status response")
            return None
    
    def interactive_mode(self):
        """Run in interactive mode"""
        print("\n" + "="*60)
        print("STATUS QUERY CLI - INTERACTIVE MODE")
        print("="*60)
        print("Ask questions about the robot's current status!")
        print("Type 'quit' to exit")
        print("Type 'help' for example queries")
        print("="*60)
        
        # Example queries
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
                    # Query status
                    self.query_status(user_input)
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
    
    def run_single_query(self, query):
        """Run a single status query"""
        return self.query_status(query)


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Status Query CLI for Behavior Tree Execution')
    parser.add_argument('--query', '-q', help='Single status query to execute')
    parser.add_argument('--interactive', '-i', action='store_true', help='Run in interactive mode')
    
    args = parser.parse_args()
    
    try:
        # Create CLI instance
        cli = StatusQueryCLI()
        
        if args.query:
            # Single query mode
            cli.run_single_query(args.query)
        elif args.interactive:
            # Interactive mode
            cli.interactive_mode()
        else:
            # Default to interactive mode
            cli.interactive_mode()
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
