# LLM Behavior Tree for Grasping (llm_bt_grasp)

A ROS package that implements LLM-driven behavior trees for robotic grasping and object manipulation tasks using the Doosan M1013 robot.

## 🎯 **Overview**

This package provides a modular, extensible framework for creating behavior trees that can:
- **Detect objects** using computer vision
- **Pick and place objects** with safety monitoring
- **Integrate with LLM commands** for natural language control
- **Support multiple robot modes** (virtual simulation and real robot)
- **Provide various tree types** for different use cases

## 🏗️ **Architecture**

### **Core Components**

1. **Behavior Classes** (`src/llm_bt_grasp/behaviors.py`)
   - `DetectObject` - Object detection behavior
   - `PickObject` - Pick-up operation behavior
   - `PlaceObject` - Placement operation behavior
   - `DetectTarget` - Target location detection
   - `SafetyCheck` - Safety monitoring behavior
   - `WaitForCommand` - LLM command waiting behavior

2. **LLM Tree Generator** (`src/llm_bt_grasp/llm_tree_generator.py`)
   - **Dynamic behavior tree generation** from natural language commands
   - **LLM integration** with models like Falcon-RW-1B
   - **XML parsing and validation** for generated trees
   - **Fallback generation** when LLM fails
   - **Complete workflow integration** with XML tree parser

3. **Command Line Interface** (`scripts/llm_tree_generator_cli.py`)
   - Interactive interface for generating behavior trees
   - Support for single commands and batch processing

4. **Vision Simulator** (`scripts/vision_simulator.py`)
   - Simulates vision system for testing in virtual mode

5. **Robot Controller** (`scripts/robot_controller.py`)
   - Mock robot controller for testing generated trees

6. **Status Monitor** (`src/llm_bt_grasp/status_monitor.py`)
   - **Real-time execution monitoring** and status tracking
   - **LLM-enhanced natural language response generation** for user queries
   - **Context-aware status responses** using current robot state
   - **Progress tracking** and behavior completion monitoring
   - **Fallback to rule-based responses** if LLM is unavailable

7. **Status Query CLI** (`scripts/status_query_cli.py`)
   - **Interactive interface** for querying execution status
   - **Natural language queries** with intelligent responses
   - **Real-time status updates** during behavior tree execution

## 🚀 **Quick Start**

### **Prerequisites**
- ROS Melodic
- Python 2.7
- Doosan robot packages
- py_trees and py_trees_ros

### **Installation**

1. **Clone and build:**
   ```bash
   cd ~/cobotrees_ws/src
   # Copy the llm_bt_grasp package here
   cd ..
   catkin_make
   source devel/setup.bash
   ```

2. **Test LLM tree generation:**
   ```bash
   # Launch the complete LLM tree generator system
   roslaunch llm_bt_grasp llm_tree_generator.launch
   
   # Or run the CLI directly
   rosrun llm_bt_grasp llm_tree_generator_cli.py --interactive
   ```

## 🧠 **LLM Tree Generator & XML Parser**

### **Complete Workflow: Generation to Execution**

The system now provides a complete workflow from natural language commands to executable behavior trees:

1. **LLM Generation**: Converts natural language to XML
2. **XML Parsing**: Converts XML to py_trees behavior tree
3. **Tree Validation**: Ensures the tree is executable
4. **Ready for Execution**: Returns a runnable behavior tree object

### **Dynamic Behavior Tree Generation**

The LLM Tree Generator allows you to create behavior trees dynamically from natural language commands:

```bash
# Launch the LLM tree generator system
roslaunch llm_bt_grasp llm_tree_generator.launch

# Or run the CLI directly
rosrun llm_bt_grasp llm_tree_generator_cli.py --interactive

# Process a single command
rosrun llm_bt_grasp llm_tree_generator_cli.py --command "Pick up the red cube and place it on the green box" --output tree.xml
```

### **Example Commands**

- **"Pick up the blue cube and place it on the yellow cube"**
- **"Detect the red sphere, grab it, and put it in the green container"**
- **"Move the orange ball from the table to the shelf"**
- **"Find the purple cube and place it on the blue platform"**

### **Generated XML Output**

```xml
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
```

### **LLM Integration Features**

- **Model**: Uses Falcon-RW-1B by default (configurable)
- **Prompt Engineering**: Optimized prompts for robotics tasks
- **XML Validation**: Automatic cleaning and validation of generated XML
- **Fallback Generation**: Rule-based generation when LLM fails
- **Real-time Generation**: Generate trees on-the-fly from user input

## 📋 **Usage Examples**

### **1. Interactive Mode**
```bash
# Launch the complete system
roslaunch llm_bt_grasp llm_tree_generator.launch

# Or run CLI directly
rosrun llm_bt_grasp llm_tree_generator_cli.py --interactive
```

### **2. Single Command Processing**
```bash
# Process a single command
rosrun llm_bt_grasp llm_tree_generator_cli.py \
  --command "Pick up the red cube and place it on the green box" \
  --output generated_tree.xml
```

### **3. Custom LLM Model**
```bash
# Use a different LLM model
rosrun llm_bt_grasp llm_tree_generator_cli.py \
  --model "microsoft/DialoGPT-medium" \
  --device 1 \
  --interactive
```

### **4. Complete Workflow Test**
```bash
# Test the complete workflow from generation to execution
rosrun llm_bt_grasp test_complete_workflow.py

# This will monitor the entire process:
# Natural Language → LLM Generation → XML → Behavior Tree → Validation
```

## 🔍 **Status Query System**

### **Real-Time Execution Monitoring**

The Status Query System allows users to ask questions about the robot's current execution status and receive natural language responses:

```bash
# Launch the complete system with status monitoring
roslaunch llm_bt_grasp llm_tree_generator.launch

# Query status interactively
rosrun llm_bt_grasp status_query_cli.py --interactive

# Run a single status query
rosrun llm_bt_grasp status_query_cli.py --query "What is the current progress?"

# Demo the status system
rosrun llm_bt_grasp demo_status_system.py
```

### **Example Status Queries**

- **"What is the current progress?"** → "The task is 75.0% complete. 3 behaviors have been completed successfully. Currently working on: Pick red cube"
- **"What is the robot doing right now?"** → "Currently executing: Pick red cube, step: grasp. Robot status: closing gripper, Vision: scanning"
- **"How long has the task been running?"** → "The task has been running for 2 minutes and 15 seconds."
- **"Are there any safety issues?"** → "All safety systems are functioning normally. The workspace is safe for robot operation."
- **"Give me a status overview"** → Complete status summary with progress, time, and system status

### **Status Query Features**

- **Real-time Monitoring**: Track behavior execution progress
- **Natural Language**: Ask questions in plain English
- **LLM-Enhanced Responses**: AI-powered, context-aware status information
- **Intelligent Context Understanding**: LLM considers current robot state for responses
- **Progress Tracking**: Monitor completion percentage and timing
- **System Health**: Check robot, vision, and safety status
- **Behavior History**: See completed and failed behaviors
- **Fallback Reliability**: Rule-based responses if LLM is unavailable

## 🔄 **Complete Workflow Example**

### **End-to-End Task Execution with LLM-Enhanced Status Monitoring**

This section demonstrates the complete workflow from natural language command to intelligent status monitoring, showing how all components work together with actual files, topics, and messages.

#### **Step 1: User Provides Natural Language Command**
```bash
# User types a natural language command
"Pick up the red cube and place it on the green box"
```

**Implementation Details:**
- **File**: `scripts/llm_tree_generator_cli.py`
- **Input Method**: Interactive CLI or command-line argument
- **Processing**: Command is sent to LLM Tree Generator via internal method call

#### **Step 2: LLM Tree Generator Creates Behavior Tree**
The LLM processes the command and generates a structured behavior tree:

**Files Involved:**
- **Main File**: `src/llm_bt_grasp/llm_tree_generator.py`
- **Model Loading**: `_load_llm_model()` method
- **Tree Generation**: `generate_tree_from_command()` method
- **XML Processing**: `_extract_xml()`, `_clean_and_validate_xml()` methods

**LLM Prompt Template** (from `_get_prompt_template()`):
```python
prompt = """You are a robotics AI that constructs behavior trees.
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

User: "{}"

Output:"""
```

**Generated XML Output:**
```xml
<root BTCPP_format="4" main_tree_to_execute="LLM_SubTree">
  <BehaviorTree ID="LLM_SubTree">
    <Sequence name="Pick and Place Task">
      <DetectObject name="red cube"/>
      <PickObject name="red cube"/>
      <DetectTarget name="green box"/>
      <PlaceObject name="red cube"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**ROS Topics Published:**
- **Topic**: `/generated_behavior_tree` (std_msgs/String)
- **Topic**: `/tree_generation_status` (std_msgs/String)
- **Message**: Generated XML tree or error status

**What happens internally:**
1. **Command Processing**: LLM receives natural language input via prompt template
2. **Context Understanding**: LLM interprets robotics terminology and task requirements
3. **Tree Generation**: Creates XML structure with appropriate behaviors using Falcon-RW-1B model
4. **XML Validation**: System validates and cleans the generated XML using `_extract_xml()` and `_clean_and_validate_xml()`
5. **Deployment**: Tree is published to ROS topics for execution

#### **Step 3: Behavior Tree Execution Begins**
The robot starts executing the generated behavior tree:

**Files Involved:**
- **Behavior Classes**: `src/llm_bt_grasp/behaviors.py`
- **Behavior Tree Execution**: py_trees framework integration
- **Robot Control**: Mock robot controller (`scripts/robot_controller.py`) or real robot via Doosan packages

**ROS Topics for Monitoring:**
```bash
# Monitor execution progress
rostopic echo /behavior_status      # Behavior execution status
rostopic echo /robot_status         # Robot movement and gripper status
rostopic echo /vision_status        # Object detection status
rostopic echo /safety_status        # Workspace safety status
```

**Topic Details:**
- **`/behavior_status`**: `std_msgs/String` - Format: `"behavior_name:status:step"`
- **`/robot_status`**: `std_msgs/String` - Format: `"moving to position"`, `"closing gripper"`, etc.
- **`/vision_status`**: `std_msgs/String` - Format: `"scanning"`, `"object detected"`, `"target located"`
- **`/safety_status`**: `std_msgs/String` - Format: `"safe"`, `"collision detected"`, `"human detected"`

**Execution Flow:**
1. **DetectObject("red cube")** → Vision system scans for red cube
   - **File**: `behaviors.py` - `DetectObject` class
   - **Status Published**: `/behavior_status` → `"DetectObject:running:scanning"`
   - **Vision Topic**: `/vision_status` → `"scanning for red cube"`

2. **PickObject("red cube")** → Robot moves to detected position and grasps
   - **File**: `behaviors.py` - `PickObject` class
   - **Status Published**: `/behavior_status` → `"PickObject:running:grasp"`
   - **Robot Topic**: `/robot_status` → `"moving to detected position"`

3. **DetectTarget("green box")** → Vision system locates green box
   - **File**: `behaviors.py` - `DetectTarget` class
   - **Status Published**: `/behavior_status` → `"DetectTarget:running:scanning"`

4. **PlaceObject("red cube")** → Robot places cube on target
   - **File**: `behaviors.py` - `PlaceObject` class
   - **Status Published**: `/behavior_status` → `"PlaceObject:running:placing"`

#### **Step 4: Real-Time Status Monitoring**
Throughout execution, the Status Monitor tracks all system states:

**Files Involved:**
- **Main File**: `src/llm_bt_grasp/status_monitor.py`
- **Status Class**: `ExecutionStatus` class for tracking execution state
- **ROS Integration**: Subscribers and publishers for status communication

**ROS Topics Subscribed:**
- **`/behavior_status`** → `_behavior_callback()` method
- **`/robot_status`** → `_robot_callback()` method  
- **`/vision_status`** → `_vision_callback()` method
- **`/safety_status`** → `_safety_callback()` method

**Status Data Structure** (from `ExecutionStatus` class):
```python
# Status Monitor continuously updates:
execution_status = {
    'current_behavior': 'PickObject',
    'current_step': 'grasp',
    'progress': 50.0,
    'robot_status': 'closing gripper',
    'vision_status': 'object detected',
    'safety_status': 'safe',
    'elapsed_time': 45.2,
    'completed': ['DetectObject'],
    'failed': []
}
```

**Status Update Methods:**
- **`update_behavior()`**: Updates current behavior and calculates progress
- **`get_status_summary()`**: Returns formatted status dictionary
- **Progress Calculation**: `(completed_behaviors / total_behaviors) * 100.0`

#### **Step 5: User Queries Status with Natural Language**
Users can ask questions about the execution at any time:

**Files Involved:**
- **CLI Interface**: `scripts/status_query_cli.py`
- **Status Monitor**: `src/llm_bt_grasp/status_monitor.py`
- **LLM Integration**: `_generate_llm_response()` method

**ROS Topics for Status Queries:**
- **Query Input**: `/status_query` (std_msgs/String) - User questions
- **Query Response**: `/status_response` (std_msgs/String) - LLM-generated answers

**CLI Usage:**
```bash
# Launch status query interface
rosrun llm_bt_grasp status_query_cli.py --interactive

# Run single query
rosrun llm_bt_grasp status_query_cli.py --query "What is the current progress?"

# Demo the system
rosrun llm_bt_grasp demo_status_system.py
```

**Example Queries and LLM Responses:**

**Query**: "Why is this taking so long?"
**LLM Response**: *"Based on the current status, the robot is taking longer than expected because it's currently executing 'Pick red cube' which involves precise positioning. The vision system has detected the object and the robot is carefully closing the gripper to ensure a secure grasp. This precision is necessary for successful pick-and-place operations."*

**Query**: "What's the next step?"
**LLM Response**: *"After completing the current 'Pick red cube' behavior, the next step will be 'DetectTarget green box' where the vision system will scan for the green box to determine the placement location. Then the robot will execute 'PlaceObject red cube' to complete the task."*

**Query**: "Is everything working correctly?"
**LLM Response**: *"Yes, everything is working correctly! The task is 50% complete with 1 behavior successfully finished. The robot is currently in the 'grasp' step of picking the red cube, and all safety systems are functioning normally. The vision system has successfully detected the target object, so the execution is proceeding as planned."*

#### **Step 6: LLM-Enhanced Response Generation Process**
When a user query arrives, here's what happens:

**Files and Methods Involved:**
- **Status Monitor**: `src/llm_bt_grasp/status_monitor.py`
- **Query Handler**: `_query_callback()` method
- **Status Formatter**: `_format_status_for_llm()` method
- **LLM Generator**: `_generate_llm_response()` method
- **LLM Model**: `_load_llm_model()` method

**1. Status Context Gathering** (from `_format_status_for_llm()`):
   ```python
   status_msg = """• Current Task: PickObject
   • Progress: 50.0%
   • Elapsed Time: 0m 45s
   • Robot Status: closing gripper
   • Vision System: object detected
   • Safety Status: safe
   • Completed Behaviors: 1
   • Failed Behaviors: 0"""
   ```

**2. LLM Prompt Creation** (from `_generate_llm_response()`):
   ```python
   prompt = f"""You are a helpful robot status assistant. 

   Current Robot Status:
   {status_msg}

   User Question: {user_query}

   Please provide a natural, helpful response based on the current status. Be conversational and informative.

   Response:"""
   ```

**3. LLM Processing**: 
   - **Model**: Falcon-RW-1B (configurable via `_load_llm_model()`)
   - **Parameters**: max_length=200, temperature=0.7, do_sample=True
   - **Fallback**: Rule-based responses if LLM fails

**4. Response Delivery**: 
   - **Topic**: `/status_response` (std_msgs/String)
   - **Publisher**: `status_response_pub` in StatusMonitor class
   - **Message Format**: Natural language response from LLM

**LLM Integration Details:**
- **Model Loading**: Uses Hugging Face transformers pipeline
- **Device Support**: GPU acceleration (device=0 by default)
- **Error Handling**: Automatic fallback to rule-based responses
- **Response Extraction**: Splits on "Response:" and strips whitespace

#### **Step 7: Task Completion and Final Status**
When the task completes:

**Files Involved:**
- **Status Monitor**: `src/llm_bt_grasp/status_monitor.py`
- **CLI Interface**: `scripts/status_query_cli.py`
- **Demo System**: `scripts/demo_status_system.py`

**Final Status Query:**
```bash
# Final status query
rosrun llm_bt_grasp status_query_cli.py --query "Give me a final summary"

# Or use demo system
rosrun llm_bt_grasp demo_status_system.py
```

**LLM Response**: *"Excellent! The task has been completed successfully. Here's what was accomplished: The robot successfully detected the red cube, picked it up with precision, located the green box target, and placed the cube exactly where requested. The entire operation took 1 minute and 23 seconds, with all 4 behaviors completing without any failures. The robot is now idle and ready for the next command."*

**Status Data at Completion:**
```python
final_status = {
    'current_behavior': None,
    'current_step': None,
    'progress': 100.0,
    'robot_status': 'idle',
    'vision_status': 'ready',
    'safety_status': 'safe',
    'elapsed_time': 83.0,  # 1m 23s
    'completed': ['DetectObject', 'PickObject', 'DetectTarget', 'PlaceObject'],
    'failed': []
}
```

### **Key Benefits of This Workflow**

1. **Natural Language Interface**: Users can control robots using plain English
2. **Intelligent Status Monitoring**: LLM provides context-aware, conversational responses
3. **Real-Time Updates**: Continuous monitoring of all system components
4. **Fallback Reliability**: Rule-based responses if LLM is unavailable
5. **Seamless Integration**: All components work together through ROS topics
6. **User-Friendly**: No need to learn complex robotics commands or status codes

### **System Architecture During Execution**

**ROS Topic Flow:**
```
User Query (/status_query) → Status Monitor → LLM Model → Natural Response (/status_response)
     ↓
Status Monitor ← Behavior Tree ← Vision/Robot Systems
     ↓
LLM Context: Current Status + User Question
     ↓
Intelligent, Context-Aware Response
```

**File Dependencies:**
- **`status_monitor.py`** ← Subscribes to behavior, robot, vision, safety topics
- **`behaviors.py`** ← Publishes behavior status updates
- **`llm_tree_generator.py`** ← Generates initial behavior trees
- **`status_query_cli.py`** ← Provides user interface for queries

This workflow demonstrates how the LLM-enhanced system transforms complex robotic operations into intuitive, natural language interactions while maintaining full system visibility and control.

### **Technical Implementation Summary**

**Core ROS Topics Used:**
| Topic | Message Type | Publisher | Subscriber | Purpose |
|-------|-------------|-----------|------------|---------|
| `/generated_behavior_tree` | `std_msgs/String` | `LLMTreeGenerator` | Behavior Tree Executor | Generated XML trees |
| `/tree_generation_status` | `std_msgs/String` | `LLMTreeGenerator` | Monitoring | Generation success/failure |
| `/constructed_behavior_tree` | `std_msgs/String` | `LLMTreeGenerator` | Monitoring | Tree construction status |
| `/tree_parsing_status` | `std_msgs/String` | `XMLTreeParser` | Monitoring | XML parsing status |
| `/behavior_status` | `std_msgs/String` | `Behavior Classes` | `StatusMonitor` | Behavior execution updates |
| `/robot_status` | `std_msgs/String` | Robot Controller | `StatusMonitor` | Robot movement/gripper status |
| `/vision_status` | `std_msgs/String` | Vision System | `StatusMonitor` | Object detection status |
| `/safety_status` | `std_msgs/String` | Safety Monitor | `StatusMonitor` | Workspace safety status |
| `/status_query` | `std_msgs/String` | `StatusQueryCLI` | `StatusMonitor` | User status questions |
| `/status_response` | `std_msgs/String` | `StatusMonitor` | `StatusQueryCLI` | LLM-generated answers |

**Key Python Classes and Methods:**
- **`LLMTreeGenerator`**: `generate_tree_from_command()`, `generate_and_execute_tree()`, `_generate_with_llm()`, `_extract_xml()`
- **`XMLTreeParser`**: `parse_xml_to_tree()`, `validate_tree()`, `get_tree_info()`, `_parse_node()`
- **`StatusMonitor`**: `_query_callback()`, `_generate_llm_response()`, `_format_status_for_llm()`
- **`ExecutionStatus`**: `update_behavior()`, `get_status_summary()`
- **`StatusQueryCLI`**: `query_status()`, `interactive_mode()`

**Message Format Examples:**
```python
# Behavior Status: "behavior_name:status:step"
"DetectObject:running:scanning"
"PickObject:completed:grasp"
"PlaceObject:failed:positioning"

# Robot Status: Descriptive text
"moving to detected position"
"closing gripper"
"idle"

# Vision Status: Detection state
"scanning for red cube"
"object detected at (x, y, z)"
"target located"
```

**LLM Configuration:**
- **Default Model**: `tiiuae/falcon-rw-1b`
- **Generation Parameters**: max_length=200, temperature=0.7, do_sample=True
- **Fallback**: Rule-based responses if LLM unavailable
- **Device**: GPU acceleration (device=0)

## 🔧 **Command Line Options**

```bash
rosrun llm_bt_grasp llm_tree_generator_cli.py [OPTIONS]

Options:
  --model MODEL_NAME    LLM model to use (default: tiiuae/falcon-rw-1b)
  --device DEVICE_ID    Device ID for GPU acceleration (default: 0)
  --command COMMAND     Single command to process (non-interactive mode)
  --output FILENAME     Output file to save generated tree
  --interactive         Run in interactive mode
```

## 🌳 **Generated Behavior Tree Examples**

### **Simple Pick-and-Place**
```
Sequence("Pick and Place Task")
├── DetectObject("red cube")
├── PickObject("red cube")
├── DetectTarget("green box")
└── PlaceObject("red cube")
```

### **Safety-Enhanced Task**
```
Parallel("Safe Task")
├── Sequence("Main Task")
│   ├── DetectObject("blue sphere")
│   ├── PickObject("blue sphere")
│   ├── DetectTarget("red container")
│   └── PlaceObject("blue sphere")
└── SafetyCheck("Safety Monitor")
```

### **Complex Multi-Step Task**
```
Sequence("Complex Task")
├── SafetyCheck("Initial Safety")
├── DetectObject("yellow cube")
├── PickObject("yellow cube")
├── DetectTarget("blue platform")
├── PlaceObject("yellow cube")
└── SafetyCheck("Final Safety")
```

## 🔌 **Integration Points**

### **Vision System Integration**
- **Topics**: `/detected_objects`, `/detected_targets`, `/safety_status`
- **Message Type**: `std_msgs/String`
- **Format**: `"Detected {object_name} at position (x, y, z)"`

### **Robot Control Integration**
- **Interface**: Doosan robot control via `dsrMoveIt`
- **Methods**: `moveTo()`, `close_gripper()`, `open_gripper()`
- **Fallback**: Mock robot for testing

### **LLM Integration**
- **Topic**: `/llm_command`
- **Format**: Natural language commands
- **Example**: `"pick red cube and place in green box"`

## 🧪 **Testing and Development**

### **Virtual Mode Testing**
```bash
# Launch with vision simulator
roslaunch llm_bt_grasp llm_grasp_tree.launch mode:=virtual

# Monitor topics
rostopic echo /detected_objects
rostopic echo /detected_targets
rostopic echo /safety_status
```

### **Real Robot Testing**
```bash
# Launch with real robot
roslaunch llm_bt_grasp llm_grasp_tree.launch \
  mode:=real \
  host:=192.168.137.100 \
  port:=12345
```

### **Visualization**
```bash
# Launch rqt_py_trees for tree visualization
rosrun rqt_py_trees rqt_py_trees --clear-config
```

## 📁 **Package Structure**

```
llm_bt_grasp/
├── package.xml                    # Package manifest
├── CMakeLists.txt                # Build configuration
├── README.md                     # This file
├── src/
│   └── llm_bt_grasp/
│       ├── __init__.py           # Package initialization
│       ├── behaviors.py          # Behavior class definitions
│       ├── llm_tree_generator.py # LLM-driven tree generation
│       ├── xml_tree_parser.py    # XML to behavior tree parser
│       └── status_monitor.py     # Execution status monitoring
├── scripts/
│   ├── llm_tree_generator_cli.py # Command-line interface
│   ├── vision_simulator.py       # Vision simulation
│   ├── robot_controller.py       # Mock robot controller
│   ├── status_query_cli.py       # Status query interface
│   ├── demo_status_system.py     # Status system demo
│   └── test_complete_workflow.py # Complete workflow test
├── launch/
│   └── llm_tree_generator.launch # Launch file
└── config/                        # Configuration files (future)
```

## 🔒 **Safety Features**

- **Collision Detection**: Built-in safety checks during robot movement
- **Human Detection**: Monitors workspace for human presence
- **Timeout Protection**: Configurable timeouts for all operations
- **Fallback Behaviors**: Automatic fallback when primary behaviors fail
- **Emergency Stop**: Integration with robot safety systems

## 🚧 **Customization**

### **Adding New Behaviors**
1. Create new behavior class inheriting from `py_trees.behaviour.Behaviour`
2. Implement required methods: `setup()`, `initialise()`, `update()`, `terminate()`
3. Add to `behaviors.py`
4. Integrate into tree builder functions

### **Modifying Robot Movements**
1. Edit pose coordinates in behavior classes
2. Adjust speed and acceleration parameters
3. Modify gripper control sequences

### **Extending Tree Types**
1. Add new tree creation function to `tree_builder.py`
2. Update main script to support new tree type
3. Add launch file parameters if needed

## 🐛 **Troubleshooting**

### **Common Issues**

1. **Import Errors**
   - Ensure package is built: `catkin_make`
   - Source workspace: `source devel/setup.bash`

2. **Robot Connection Failed**
   - Check IP address and port settings
   - Verify robot is powered on and accessible
   - Check firewall settings

3. **Vision System Not Detecting**
   - Ensure vision nodes are running
   - Check topic names and message formats
   - Verify camera calibration

4. **Behavior Tree Not Executing**
   - Check ROS master is running
   - Verify all dependencies are installed
   - Check console output for error messages

### **Debug Mode**
```bash
# Enable debug logging
rosservice call /rosout/set_logger_level ros.llm_bt_grasp DEBUG

# Monitor specific topics
rostopic echo /llm_grasp_behavior_tree/tree
```

## 🤝 **Contributing**

To extend this package:

1. **Fork the repository**
2. **Create a feature branch**
3. **Implement your changes**
4. **Add tests and documentation**
5. **Submit a pull request**

### **Development Guidelines**
- Follow ROS coding standards
- Add docstrings to all functions and classes
- Test thoroughly in simulation before real robot deployment
- Update documentation for new features

## 📄 **License**

This package is licensed under the MIT License. See the LICENSE file for details.

## 🙏 **Acknowledgments**

- Built on the Doosan ROS packages
- Uses py_trees for behavior tree implementation
- Integrates with existing vision and robot control systems

## 📞 **Support**

For questions and support:
- Check the troubleshooting section
- Review the code examples
- Open an issue on the repository
- Contact the development team

---

**Happy Grasping! 🤖✋**


