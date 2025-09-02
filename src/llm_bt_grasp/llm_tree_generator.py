#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LLM-driven Behavior Tree Generator

This module provides functionality to generate behavior trees dynamically
from natural language user commands using LLM models.
"""

import re
import xml.etree.ElementTree as ET
from xml.dom import minidom
import rospy
from std_msgs.msg import String
from .behaviors import *

class LLMTreeGenerator:
    """
    Generates behavior trees from natural language commands using LLM models
    """
    
    def __init__(self, model_name="tiiuae/falcon-rw-1b", device=0):
        self.model_name = model_name
        self.device = device
        self.generator = None
        self.prompt_template = self._get_prompt_template()
        
        # Initialize LLM model
        self._load_model()
        
        # ROS publishers for generated trees
        self.tree_pub = rospy.Publisher('/generated_behavior_tree', String, queue_size=10)
        self.status_pub = rospy.Publisher('/tree_generation_status', String, queue_size=10)
        
    def _load_model(self):
        """Load the LLM model"""
        try:
            from transformers import pipeline
            self.generator = pipeline(
                "text-generation",
                model=self.model_name,
                device=self.device
            )
            rospy.loginfo(f"[LLM] Model {self.model_name} loaded successfully")
        except ImportError:
            rospy.logwarn("Transformers not available, using mock generator")
            self.generator = MockGenerator()
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            self.generator = MockGenerator()
    
    def _get_prompt_template(self):
        """Get the prompt template for LLM generation"""
        return """You are a robotics AI that constructs behavior trees.
The user gives you a task. Output an XML Behavior Tree.

Use these node types:
- DetectObject (input: name)
- PickObject (input: name) 
- DetectTarget (input: name)
- PlaceObject (input: name)
- SafetyCheck (no input)
- WaitForCommand (no input)

Available tree structures:
- Sequence: Execute behaviors in order
- Parallel: Execute behaviors simultaneously
- Selector: Try behaviors until one succeeds
- Retry: Retry a behavior multiple times

Example:
User: "Pick up the blue cube and place it on the yellow cube"

Output:
<root BTCPP_format="4" main_tree_to_execute="LLM_SubTree">
  <BehaviorTree ID="LLM_SubTree">
    <Sequence name="Pick and Place Task">
      <DetectObject name="blue cube"/>
      <PickObject name="blue cube"/>
      <DetectTarget name="yellow cube"/>
      <PlaceObject name="blue cube"/>
    </Sequence>
  </BehaviorTree>
</root>

User: "{}"

Output:"""
    
    def generate_tree_from_command(self, user_command):
        """
        Generate a behavior tree from a natural language command
        
        Args:
            user_command (str): Natural language command from user
            
        Returns:
            str: Generated XML behavior tree
        """
        try:
            rospy.loginfo(f"Generating behavior tree for command: {user_command}")
            
            # Generate XML using LLM
            prompt = self.prompt_template.format(user_command)
            generated_text = self._generate_with_llm(prompt)
            
            # Extract XML from generated text
            xml_tree = self._extract_xml(generated_text)
            
            # Validate and clean XML
            cleaned_xml = self._clean_and_validate_xml(xml_tree)
            
            # Publish generated tree
            self.tree_pub.publish(cleaned_xml)
            self.status_pub.publish("SUCCESS: Tree generated successfully")
            
            rospy.loginfo("Behavior tree generated successfully")
            return cleaned_xml
            
        except Exception as e:
            error_msg = f"ERROR: Failed to generate tree: {str(e)}"
            rospy.logerr(error_msg)
            self.status_pub.publish(error_msg)
            return None
    
    def _generate_with_llm(self, prompt):
        """Generate text using the LLM model"""
        try:
            if hasattr(self.generator, 'generate'):
                # Real LLM model
                result = self.generator(
                    prompt,
                    max_length=500,
                    num_return_sequences=1,
                    temperature=0.7,
                    do_sample=True
                )
                return result[0]['generated_text']
            else:
                # Mock generator
                return self.generator.generate(prompt)
        except Exception as e:
            rospy.logwarn(f"LLM generation failed: {e}, using fallback")
            return self._fallback_generation(prompt)
    
    def _extract_xml(self, generated_text):
        """Extract XML content from generated text"""
        # Look for XML content between <root> and </root>
        xml_pattern = r'<root.*?</root>'
        match = re.search(xml_pattern, generated_text, re.DOTALL | re.IGNORECASE)
        
        if match:
            return match.group(0)
        else:
            # Try to find any XML-like content
            xml_pattern = r'<.*?>.*?</.*?>'
            match = re.search(xml_pattern, generated_text, re.DOTALL | re.IGNORECASE)
            if match:
                return f"<root BTCPP_format=\"4\" main_tree_to_execute=\"LLM_SubTree\">\n  <BehaviorTree ID=\"LLM_SubTree\">\n    {match.group(0)}\n  </BehaviorTree>\n</root>"
            else:
                raise ValueError("No valid XML found in generated text")
    
    def _clean_and_validate_xml(self, xml_string):
        """Clean and validate the generated XML"""
        try:
            # Parse XML to check validity
            root = ET.fromstring(xml_string)
            
            # Pretty print the XML
            rough_string = ET.tostring(root, 'utf-8')
            reparsed = minidom.parseString(rough_string)
            pretty_xml = reparsed.toprettyxml(indent="  ")
            
            # Remove empty lines
            lines = [line for line in pretty_xml.split('\n') if line.strip()]
            cleaned_xml = '\n'.join(lines)
            
            return cleaned_xml
            
        except ET.ParseError as e:
            rospy.logwarn(f"XML parsing error: {e}, attempting to fix")
            return self._fix_xml(xml_string)
    
    def _fix_xml(self, xml_string):
        """Attempt to fix common XML issues"""
        # Common fixes for malformed XML
        fixes = [
            (r'<(\w+)([^>]*?)(?<!/)>', r'<\1\2></\1>'),  # Fix unclosed tags
            (r'&([^a-zA-Z])', r'&amp;\1'),  # Fix ampersands
            (r'<(\w+)([^>]*?)(?<!/)>([^<]*)', r'<\1\2>\3</\1>'),  # Fix content in unclosed tags
        ]
        
        fixed_xml = xml_string
        for pattern, replacement in fixes:
            fixed_xml = re.sub(pattern, replacement, fixed_xml)
        
        return fixed_xml
    
    def _fallback_generation(self, prompt):
        """Fallback generation when LLM fails"""
        # Simple rule-based generation
        user_command = prompt.split("User: ")[-1].split("\n")[0].strip('"')
        
        # Extract objects and actions using simple patterns
        objects = re.findall(r'\b(?:the\s+)?(\w+\s+\w+)', user_command)
        
        if len(objects) >= 2:
            object1, object2 = objects[0], objects[1]
            
            # Simple pick and place pattern
            if "pick" in user_command.lower() and "place" in user_command.lower():
                return f"""<root BTCPP_format="4" main_tree_to_execute="LLM_SubTree">
  <BehaviorTree ID="LLM_SubTree">
    <Sequence name="Pick and Place Task">
      <DetectObject name="{object1}"/>
      <PickObject name="{object1}"/>
      <DetectTarget name="{object2}"/>
      <PlaceObject name="{object1}"/>
    </Sequence>
  </BehaviorTree>
</root>"""
        
        # Default fallback
        return """<root BTCPP_format="4" main_tree_to_execute="LLM_SubTree">
  <BehaviorTree ID="LLM_SubTree">
    <Sequence name="Default Task">
      <DetectObject name="object"/>
      <PickObject name="object"/>
      <DetectTarget name="target"/>
      <PlaceObject name="object"/>
    </Sequence>
  </BehaviorTree>
</root>"""
    
    def parse_xml_to_behavior_tree(self, xml_string, robot):
        """
        Parse XML behavior tree and convert to py_trees behavior tree
        
        Args:
            xml_string (str): XML behavior tree
            robot: Robot controller instance
            
        Returns:
            py_trees.behaviour.Behaviour: Constructed behavior tree
        """
        try:
            root = ET.fromstring(xml_string)
            
            # Find the main behavior tree
            behavior_tree = root.find(".//BehaviorTree")
            if behavior_tree is None:
                raise ValueError("No BehaviorTree found in XML")
            
            # Parse the tree structure
            tree_root = self._parse_node(behavior_tree, robot)
            
            return tree_root
            
        except Exception as e:
            rospy.logerr(f"Failed to parse XML to behavior tree: {e}")
            # Return default tree
            return self._create_default_tree(robot)
    
    def _parse_node(self, xml_node, robot):
        """Recursively parse XML node to py_trees behavior"""
        node_type = xml_node.tag
        
        if node_type == "Sequence":
            sequence = py_trees.composites.Sequence(xml_node.get("name", "Sequence"))
            for child in xml_node:
                child_behavior = self._parse_node(child, robot)
                sequence.add_child(child_behavior)
            return sequence
            
        elif node_type == "Parallel":
            parallel = py_trees.composites.Parallel(xml_node.get("name", "Parallel"))
            for child in xml_node:
                child_behavior = self._parse_node(child, robot)
                parallel.add_child(child_behavior)
            return parallel
            
        elif node_type == "Selector":
            selector = py_trees.composites.Selector(xml_node.get("name", "Selector"))
            for child in xml_node:
                child_behavior = self._parse_node(child, robot)
                selector.add_child(child_behavior)
            return selector
            
        elif node_type == "DetectObject":
            name = xml_node.get("name", "object")
            return DetectObject(f"Detect {name}", name, robot)
            
        elif node_type == "PickObject":
            name = xml_node.get("name", "object")
            return PickObject(f"Pick {name}", name, robot)
            
        elif node_type == "DetectTarget":
            name = xml_node.get("name", "target")
            return DetectTarget(f"Detect {name}", name, robot)
            
        elif node_type == "PlaceObject":
            name = xml_node.get("name", "object")
            return PlaceObject(f"Place {name}", name, robot)
            
        elif node_type == "SafetyCheck":
            return SafetyCheck("Safety Check", robot)
            
        elif node_type == "WaitForCommand":
            return WaitForCommand("Wait for Command")
            
        else:
            rospy.logwarn(f"Unknown node type: {node_type}, skipping")
            return None
    
    def _create_default_tree(self, robot):
        """Create a default behavior tree when parsing fails"""
        # Create a simple default tree
        sequence = py_trees.composites.Sequence("Default Task")
        sequence.add_child(DetectObject("Detect object", "object", robot))
        sequence.add_child(PickObject("Pick object", "object", robot))
        sequence.add_child(DetectTarget("Detect target", "target", robot))
        sequence.add_child(PlaceObject("Place object", "object", robot))
        return sequence


class MockGenerator:
    """Mock generator for testing when LLM is not available"""
    
    def generate(self, prompt):
        """Generate mock behavior tree XML"""
        return """<root BTCPP_format="4" main_tree_to_execute="LLM_SubTree">
  <BehaviorTree ID="LLM_SubTree">
    <Sequence name="Mock Task">
      <DetectObject name="mock object"/>
      <PickObject name="mock object"/>
      <DetectTarget name="mock target"/>
      <PlaceObject name="mock object"/>
    </Sequence>
  </BehaviorTree>
</root>"""


def create_llm_tree_generator(model_name="tiiuae/falcon-rw-1b", device=0):
    """
    Factory function to create LLM tree generator
    
    Args:
        model_name (str): Name of the LLM model to use
        device (int): Device ID for GPU acceleration
        
    Returns:
        LLMTreeGenerator: Configured tree generator instance
    """
    return LLMTreeGenerator(model_name, device)
