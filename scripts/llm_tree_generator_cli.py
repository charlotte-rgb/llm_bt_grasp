#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# @brief    Command-line interface for LLM-driven behavior tree generation
# @author   LLM Behavior Tree Implementation

import rospy
import sys
import argparse
import time
from std_msgs.msg import String

# Import our LLM tree generator
from llm_bt_grasp.llm_tree_generator import create_llm_tree_generator

class LLMTreeGeneratorCLI:
    """
    Command-line interface for LLM-driven behavior tree generation
    """
    
    def __init__(self, model_name="tiiuae/falcon-rw-1b", device=0):
        self.model_name = model_name
        self.device = device
        self.generator = None
        self.generated_tree = None
        
        # Initialize ROS node
        rospy.init_node('llm_tree_generator_cli', anonymous=True)
        
        # Initialize LLM generator
        self._initialize_generator()
        
        # Subscribe to generated tree topics
        self.tree_sub = rospy.Subscriber('/generated_behavior_tree', String, self._tree_callback)
        self.status_sub = rospy.Subscriber('/tree_generation_status', String, self._status_callback)
        
    def _initialize_generator(self):
        """Initialize the LLM tree generator"""
        try:
            print(f"Initializing LLM tree generator with model: {self.model_name}")
            self.generator = create_llm_tree_generator(self.model_name, self.device)
            print("LLM tree generator initialized successfully!")
        except Exception as e:
            print(f"Failed to initialize generator: {e}")
            sys.exit(1)
    
    def _tree_callback(self, msg):
        """Callback for received generated tree"""
        self.generated_tree = msg.data
        print("\n" + "="*60)
        print("GENERATED BEHAVIOR TREE:")
        print("="*60)
        print(self.generated_tree)
        print("="*60)
    
    def _status_callback(self, msg):
        """Callback for generation status"""
        print(f"\n[STATUS] {msg.data}")
    
    def generate_tree_from_command(self, user_command):
        """
        Generate behavior tree from user command
        
        Args:
            user_command (str): Natural language command
        """
        print(f"\nGenerating behavior tree for command: '{user_command}'")
        print("Please wait...")
        
        # Reset generated tree
        self.generated_tree = None
        
        # Generate tree
        xml_tree = self.generator.generate_tree_from_command(user_command)
        
        if xml_tree:
            print("Tree generation completed!")
            return xml_tree
        else:
            print("Tree generation failed!")
            return None
    
    def interactive_mode(self):
        """Run in interactive mode"""
        print("\n" + "="*60)
        print("LLM BEHAVIOR TREE GENERATOR - INTERACTIVE MODE")
        print("="*60)
        print("Type 'quit' to exit")
        print("Type 'help' for examples")
        print("="*60)
        
        while not rospy.is_shutdown():
            try:
                # Get user input
                user_input = input("\nEnter your command: ").strip()
                
                if user_input.lower() == 'quit':
                    print("Goodbye!")
                    break
                elif user_input.lower() == 'help':
                    self._show_help()
                elif user_input:
                    # Generate tree
                    self.generate_tree_from_command(user_input)
                    
                    # Wait for tree to be generated
                    timeout = 30  # 30 seconds timeout
                    start_time = time.time()
                    while self.generated_tree is None and (time.time() - start_time) < timeout:
                        time.sleep(0.1)
                    
                    if self.generated_tree is None:
                        print("Timeout waiting for tree generation")
                else:
                    print("Please enter a valid command")
                    
            except KeyboardInterrupt:
                print("\nInterrupted by user")
                break
            except EOFError:
                print("\nEnd of input")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def _show_help(self):
        """Show help and examples"""
        print("\n" + "="*60)
        print("HELP & EXAMPLES")
        print("="*60)
        print("The system can understand natural language commands for robot tasks.")
        print("\nExample commands:")
        print("• 'Pick up the red cube and place it on the green box'")
        print("• 'Detect the blue sphere, pick it up, and put it in the red container'")
        print("• 'Find the yellow cube, grab it, and place it on the blue platform'")
        print("• 'Move the orange ball from the table to the shelf'")
        print("\nThe system will automatically generate a behavior tree with:")
        print("• DetectObject nodes for finding objects")
        print("• PickObject nodes for grasping")
        print("• DetectTarget nodes for finding destinations")
        print("• PlaceObject nodes for placement")
        print("• Appropriate tree structure (Sequence, Parallel, etc.)")
        print("="*60)
    
    def save_tree_to_file(self, xml_tree, filename):
        """
        Save generated tree to XML file
        
        Args:
            xml_tree (str): XML behavior tree
            filename (str): Output filename
        """
        try:
            with open(filename, 'w') as f:
                f.write(xml_tree)
            print(f"Tree saved to: {filename}")
        except Exception as e:
            print(f"Failed to save tree: {e}")


def main():
    """
    Main function for command-line interface
    """
    parser = argparse.ArgumentParser(description='LLM-driven Behavior Tree Generator CLI')
    parser.add_argument('--model', default='tiiuae/falcon-rw-1b',
                       help='LLM model to use for generation')
    parser.add_argument('--device', type=int, default=0,
                       help='Device ID for GPU acceleration')
    parser.add_argument('--command', '-c',
                       help='Single command to process (non-interactive mode)')
    parser.add_argument('--output', '-o',
                       help='Output file to save generated tree')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Run in interactive mode')
    
    args = parser.parse_args()
    
    try:
        # Create CLI instance
        cli = LLMTreeGeneratorCLI(args.model, args.device)
        
        if args.command:
            # Single command mode
            print(f"Processing command: {args.command}")
            xml_tree = cli.generate_tree_from_command(args.command)
            
            if xml_tree and args.output:
                cli.save_tree_to_file(xml_tree, args.output)
            elif xml_tree:
                print("\nGenerated tree (use --output to save to file):")
                print(xml_tree)
                
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
