#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from so_arm_bt_interfaces.srv import DetectObjects
from ament_index_python.packages import get_package_share_directory
import os
import requests
import json

class BTCompilerNode(Node):
    def __init__(self):
        super().__init__('bt_compiler_node')
        
        # Parameters
        self.declare_parameter('api_key', 'YOUR_NVIDIA_API_KEY')
        self.declare_parameter('api_url', 'https://integrate.api.nvidia.com/v1/chat/completions')
        self.declare_parameter('model', 'nvidia/nemotron-3-nano-30b-a3b')



        self.declare_parameter('output_path', 'generated_tree.xml')
        
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        if self.api_key == 'YOUR_NVIDIA_API_KEY' or not self.api_key:
            self.api_key = os.environ.get('NVIDIA_API_KEY', '')
        self.api_key = self.api_key.strip()
        self.api_url = self.get_parameter('api_url').get_parameter_value().string_value
        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        
        # Subscribers/Services
        self.intent_sub = self.create_subscription(String, 'task_intent', self.intent_callback, 10)
        self.vision_client = self.create_client(DetectObjects, 'detect_objects')
        
        self.get_logger().info('BT Compiler Node initialized using NVIDIA Nemotron')

    def intent_callback(self, msg):
        intent = msg.data
        self.get_logger().info(f'Received intent: {intent}')
        
        # 1. Get world state from vision
        world_state = self.get_world_state()
        
        # 2. Compile to BT XML
        xml_content = self.compile_with_llm(intent, world_state)
        
        if xml_content:
            # Save to package share directory
            package_path = os.path.join(get_package_share_directory('so_arm_bt'), 'config')
            if not os.path.exists(package_path):
                os.makedirs(package_path)
            
            actual_path = os.path.join(package_path, self.output_path)
            with open(actual_path, 'w') as f:
                f.write(xml_content)
            self.get_logger().info(f'Generated tree saved to: {actual_path}')
            
            # Optional: Notify executor to reload or just let user run it manually
        else:
            self.get_logger().error('Failed to generate tree')

    def get_world_state(self):
        if not self.vision_client.wait_for_service(timeout_sec=1.0):
            return "Vision service not available. Minimal state: home, ready poses known."
        
        request = DetectObjects.Request()
        future = self.vision_client.call_async(request)

        # In a real node, don't block. For simplicity:
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            if not response.detections:
                return "No objects currently detected. NOTE: You MUST still use the ObjectVisible and RetryUntilSuccessful pattern to find objects mentioned in the task, as they may become visible after the startup wait."
            detections = [f"{d.label} at x:{d.pose.pose.position.x:.2f}, y:{d.pose.pose.position.y:.2f}" for d in response.detections]
            return "Visible objects: " + ", ".join(detections)
        return "Failed to fetch vision state. Assume objects mentioned in task exist but require detection."

    def compile_with_llm(self, intent, world_state):
        prompt = f"""

System: You are a ROS 2 Behavior Tree compiler. Output ONLY valid XML and nothing else. No explanation, no intro.

ALLOWED NODES:
- <MoveToPose target_pose="home" /> (Moves to static home position)
- <GripperAction state="open|close" />
- <AttachObject object_id="ID" />
- <ObjectVisible object_id="LABEL" found_pose="{{found_pose}}" /> (Outputs dynamic pose to blackboard)
- <PrecisePick target_pose="{{found_pose}}" offset="0.05" /> (C++ node: 5cm hover in world-z, then slow approach)
- <Wait msec="1000" />
- <Sequence> <Fallback> <RetryUntilSuccessful num_attempts="50">

EXAMPLE PICK AND PLACE:
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <Wait msec="2000"/>
      <RetryUntilSuccessful num_attempts="50">
        <ObjectVisible object_id="red_cube" found_pose="{{pick_pose}}"/>
      </RetryUntilSuccessful>
      <GripperAction state="open"/>
      <PrecisePick target_pose="{{pick_pose}}" offset="0.05"/>
      <GripperAction state="close"/>
      <AttachObject object_id="red_cube"/>
      <MoveToPose target_pose="home"/>
    </Sequence>
  </BehaviorTree>
</root>

GOLDEN RULES FOR PICKING:
1. ALWAYS use <PrecisePick> for the final approach to a detected object.
2. ALWAYS use offset="0.05" for PrecisePick.
3. ALWAYS wrap <ObjectVisible> in <RetryUntilSuccessful num_attempts="50">.
4. ALWAYS wrap all children of <BehaviorTree> in a single <Sequence> or <Fallback> tag.

DEADLY SINS:
- Using <Retry> or <RetryUntilFailure>. You MUST ONLY use <RetryUntilSuccessful>.
- Combining <MoveToPose> and <PrecisePick> redundantly.
- Missing the <Wait msec="2000"/> at the very start.
- Having more than one child directly under <BehaviorTree>.

STRUCTURE:
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      ...nodes...
    </Sequence>
  </BehaviorTree>
</root>




WORLD STATE:
{world_state}

TASK:
{intent}

XML:
"""
        
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        }
        
        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": prompt}],
            "temperature": 0.1,
            "top_p": 0.7,
            "max_tokens": 2048
        }

        
        try:
            response = requests.post(self.api_url, headers=headers, json=data)
            if response.status_code != 200:
                self.get_logger().error(f'API Error {response.status_code}: {response.text}')
            response.raise_for_status()
            
            message = response.json()['choices'][0]['message']
            content = message.get('content')
            self.get_logger().info(f'LLM raw content: {content}')
            
            if content is None or "<root" not in content:

                # Try reasoning if content is empty or lacks XML
                reasoning = message.get('reasoning') or message.get('reasoning_content') or ""
                if "<root" in reasoning:
                    content = reasoning
                    self.get_logger().warn('LLM content lacked XML, extracted from reasoning.')
                elif content is None:
                    content = ""

            # 1. Try to find XML in backticks first (most reliable)
            if '```xml' in content:
                content = content.split('```xml')[1].split('```')[0]
            elif '```' in content:
                content = content.split('```')[1].split('```')[0]

            # 2. Extract from first <root to last </root>
            if "<root" in content:
                start = content.find("<root")
                end = content.rfind("</root>")
                if start != -1 and end != -1:
                    content = content[start:end + len("</root>")]
                elif start != -1:
                    content = content[start:]
                    self.get_logger().warn('Found <root but missing </root>, taking remainder.')
            
            self.get_logger().info(f'Extracted XML: {content[:100]}...')
            return content.strip()



        except Exception as e:
            self.get_logger().error(f'LLM API call failed: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = BTCompilerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
