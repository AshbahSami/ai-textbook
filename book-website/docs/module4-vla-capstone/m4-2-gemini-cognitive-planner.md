---
title: The Cognitive Core with Gemini
---

# The Cognitive Core with Gemini

With our voice commands transcribed into text and published to a ROS 2 topic, the next crucial step is to translate these natural language commands into a sequence of low-level robot actions. This is the role of the **Cognitive Planner**, powered by the **Gemini API**. Gemini's advanced reasoning and function calling capabilities allow us to build a robust planner that can interpret complex instructions and generate actionable plans.

## Setting up the Gemini API Key

Similar to the Whisper API, your Gemini API key needs to be securely managed using environment variables.

```bash
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
```

## Creating the Gemini Cognitive Planner Node

We will create a ROS 2 Python node that subscribes to the `/command_text` topic, uses the Gemini API to generate a plan, and then publishes this plan as a queue of ROS 2 Action Goals.

Create a new Python file, `gemini_planner_node.py`, within your ROS 2 package (e.g., `vla_ros_nodes`).

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped # For navigation goals, for example
from rcl_interfaces.msg import SetParametersResult
import google.generativeai as genai
import os
import json

class GeminiPlannerNode(Node):
    def __init__(self):
        super().__init__('gemini_planner_node')
        self.subscription = self.create_subscription(
            String,
            '/command_text',
            self.command_callback,
            10
        )
        self.action_publisher = self.create_publisher(String, '/robot_action_sequence', 10)
        self.get_logger().info('Gemini Planner Node has been started.')

        # Check for Gemini API Key
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not self.gemini_api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable not set!")
            raise ValueError("GEMINI_API_KEY environment variable not set!")

        genai.configure(api_key=self.gemini_api_key)
        self.model = genai.GenerativeModel('gemini-pro') # Or 'gemini-pro-vision' if visual context is needed

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'gemini_api_key' and param.type_ == SetParametersResult.Type.STRING:
                self.gemini_api_key = param.value
                genai.configure(api_key=self.gemini_api_key)
                self.get_logger().info(f'Gemini API Key updated.')
        return SetParametersResult(successful=True)

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: "{msg.data}"')
        plan = self.generate_plan(msg.data)
        if plan:
            action_msg = String()
            action_msg.data = json.dumps(plan)
            self.action_publisher.publish(action_msg)
            self.get_logger().info(f'Published action sequence: {plan}')
        else:
            # FR-010: Provide audible feedback for planning failure
            self.get_logger().warn("I'm sorry, I cannot plan that action. Can you rephrase?")


    def generate_plan(self, command):
        # Define available robot actions for Gemini
        tools = [
            {
                "function_declarations": [
                    {
                        "name": "navigate_to_location",
                        "description": "Navigates the robot to a specified named location.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "location": {"type": "string", "description": "The name of the location (e.g., 'kitchen', 'bedroom')."}
                            },
                            "required": ["location"]
                        }
                    },
                    {
                        "name": "find_object",
                        "description": "Searches for a specified object in the current environment.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "object_name": {"type": "string", "description": "The name of the object to find (e.g., 'red ball', 'cup')."}
                            },
                            "required": ["object_name"]
                        }
                    },
                    {
                        "name": "grasp_object",
                        "description": "Attempts to grasp the currently identified object.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                # No specific parameters for simplicity, assumes object is found
                            },
                        }
                    },
                    {
                        "name": "place_object",
                        "description": "Places the object currently held by the robot at a specified location.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "location": {"type": "string", "description": "The name of the location to place the object (e.g., 'table', 'shelf')."}
                            },
                            "required": ["location"]
                        }
                    }
                ]
            }
        ]

        try:
            # Prompt Gemini for a sequence of actions
            prompt = (f"Given the command '{command}', generate a sequence of low-level robot actions "
                      f"using the available tools. Ensure the plan is safe and achievable. "
                      f"If the command is unclear or impossible, state that you cannot plan the action."
                      f"Respond with a JSON list of function calls.")
            
            response = self.model.generate_content(
                prompt,
                tools=tools
            )

            # Extract function calls from the response
            if response.candidates and response.candidates[0].function_calls:
                # Gemini may return multiple function calls, parse them into a list
                function_calls_list = []
                for fc in response.candidates[0].function_calls:
                    function_calls_list.append({
                        "name": fc.name,
                        "args": {k: v for k, v in fc.args.items()}
                    })
                return function_calls_list
            else:
                self.get_logger().warn(f"Gemini did not return function calls for command: '{command}'")
                return None

        except Exception as e:
            self.get_logger().error(f"Error calling Gemini API: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = GeminiPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Prompt Engineering and Function Calling

The key to effective cognitive planning with Gemini lies in **prompt engineering** and defining the correct **function declarations**.

-   **Prompt Engineering**: Our prompt clearly instructs Gemini to generate a sequence of actions using the provided tools and to handle unclear commands gracefully.
-   **Function Calling**: We define a set of `tools` (robot actions) that Gemini can "call". This allows Gemini to translate high-level natural language (`"Clean the room"`) into a structured, executable plan (`["navigate(kitchen)", "find_object(cup)", "grasp_object()"]`).

This cognitive planning node forms the "brain" of our VLA system, translating human intent into a robot's action sequence. In the final section, we will integrate this planner with our robot's perception and control systems.
