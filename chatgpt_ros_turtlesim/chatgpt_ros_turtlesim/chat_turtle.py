import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
import re
import math

from chatgpt_ros_interfaces.action import Chat
from chatgpt_ros_interfaces.srv import SetPrompt, AddExample
from turtlesim.srv import SetPen, TeleportRelative, TeleportAbsolute
from turtlesim.msg import Pose
from std_srvs.srv import Empty


system_prompt = """You are an assistant helping me with the simulator for robots.
When I ask you to do something, you are supposed to give me Python code that is needed to achieve that task using simulator and then an explanation of what that code does.
You are only allowed to use the functions I have defined for you.
You are not to use any other hypothetical functions that you think might exist.
You can use simple Python functions from libraries such as math and numpy."""

user_prompt = """Here are some functions you can use to command the robot.

turtlebot.clear() - Delete the turtle's drawings from the screen. Do not move turtle. State and position of the turtle as well as drawings of other turtles are not affected.
turtlebot.forward(distance) - Move the turtle forward by the specified distance, in the direction the turtle is headed.
turtlebot.backward(distance) - Move the turtle backward by distance, opposite to the direction the turtle is headed. Do not change the turtle's heading.
turtlebot.right(angle) - Turn turtle right by degrees.
turtlebot.left(angle) - Turn turtle left by degrees.
turtlebot.setposition(x, y) - Move turtle to an absolute position. If the pen is down, draw line. Do not change the turtle's orientation.
turtlebot.setheading(to_angle) - Set the orientation of the turtle to to_angle.
turtlebot.home() - Move turtle to the origin - coordinates (0,0) - and set its heading to its start-orientation.
turtlebot.pendown() - Pull the pen down - drawing when moving.
turtlebot.penup() - Pull the pen up - no drawing when moving.
turtlebot.get_position() - Return the turtle's current location (x,y).
turtlebot.get_heading() - Return the turtle's current heading.

A few useful things: 
Instead of moveToPositionAsync() or moveToZAsync(), you should use the function setposition() that I have defined for you.
This is a two-dimensional environment, and setposition() accepts only 2 parameters.
Note that the robot is initially in the center of a square area of size 10*10, and you are not allowed to let the robot touch the boundary of the area."""

examples = [
    {
        "user": "move 2 units forward",
        "assistant": """```python
turtlebot.forward(10)
```
This code uses the `forward()` function to move the robot to a new position that is 2 units before the current position."""
    }
]


class TurtleBot:
    def __init__(self, node, turtle_namespace='/turtle1'):
        self._node = node
        self._set_pen_cli = node.create_client(SetPen, turtle_namespace + "/set_pen")
        self._teleport_relative_cli = node.create_client(TeleportRelative, turtle_namespace + "/teleport_relative")
        self._teleport_absolute_cli = node.create_client(TeleportAbsolute, turtle_namespace + "/teleport_absolute")
        self._clear_cli = node.create_client(Empty, "/clear")
        self._pose_sub = node.create_subscription(Pose, turtle_namespace + "/pose", self._pose_callback, 1)
        self._origin = [5.544444561004639, 5.544444561004639, 0.0]
        self._pose = [0.0, 0.0, 0.0]
        
        self.home()
    
    def _pose_callback(self, msg):
        self._pose[0] = msg.x - self._origin[0]
        self._pose[1] = msg.y - self._origin[1]
        self._pose[2] = msg.theta -  - self._origin[2]
    
    def _degree_to_radian(self, angle):
        return angle * math.pi / 180.0
    
    def clear(self):
        """Delete the turtle's drawings from the screen. Do not move turtle. State and position of the turtle as well as drawings of other turtles are not affected.
        """
        rclpy.spin_until_future_complete(self._node, self._clear_cli.call_async(Empty.Request()))
    
    def forward(self, distance):
        """Move the turtle forward by the specified distance, in the direction the turtle is headed.

        Args:
            distance (integer or float): a number
        """
        req = TeleportRelative.Request()
        req.linear = float(distance)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
        
    
    def backward(self, distance):
        """Move the turtle backward by distance, opposite to the direction the turtle is headed. Do not change the turtle's heading.

        Args:
            distance (integer or float): a number
        """
        req = TeleportRelative.Request()
        req.linear = -float(distance)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
    
    def right(self, angle):
        """Turn turtle right by degrees.

        Args:
            angle (integer or float): a number
        """
        req = TeleportRelative.Request()
        req.angular = -self._degree_to_radian(angle)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
    
    def left(self, angle):
        """Turn turtle left by degrees.

        Args:
            angle (integer or float): a number
        """
        req = TeleportRelative.Request()
        req.angular = self._degree_to_radian(angle)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
    
    def setposition(self, x, y):
        """Move turtle to an absolute position. If the pen is down, draw line. Do not change the turtle's orientation.

        Args:
            x (integer or float): a number
            y (integer or float): a number
        """
        req = TeleportAbsolute.Request()
        req.x = float(x) + self._origin[0]
        req.y = float(y) + self._origin[1]
        req.theta = self._pose[2]  + self._origin[2]
        rclpy.spin_until_future_complete(self._node, self._teleport_absolute_cli.call_async(req))
    
    def setheading(self, to_angle):
        """Set the orientation of the turtle to to_angle.

        Args:
            to_angle (integer or float): a number
        """
        req = TeleportAbsolute.Request()
        req.x = self._pose[0] + self._origin[0]
        req.y = self._pose[1] + self._origin[1]
        req.theta = to_angle + self._origin[2]
        rclpy.spin_until_future_complete(self._node, self._teleport_absolute_cli.call_async(req))
    
    def home(self):
        """Move turtle to the origin - coordinates (5.0, 5.0) - and set its heading to its start-orientation.
        """
        req = TeleportAbsolute.Request()
        req.x = self._origin[0]
        req.y = self._origin[1]
        req.theta = self._origin[2]
        rclpy.spin_until_future_complete(self._node, self._teleport_absolute_cli.call_async(req))
    
    def pendown(self):
        """Pull the pen down - drawing when moving.
        """
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 0
        rclpy.spin_until_future_complete(self._node, self._set_pen_cli.call_async(req))
    
    def penup(self):
        """Pull the pen up - no drawing when moving.
        """
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 1
        rclpy.spin_until_future_complete(self._node, self._set_pen_cli.call_async(req))
    
    def get_position(self):
        """Return the turtle's current location (x,y).

        Returns:
            list[float]: a list of numbers
        """
        return self._pose[:2]
    
    def get_heading(self):
        """Return the turtle's current heading.

        Returns:
            float: a number
        """
        return self._pose[2]


class ChatTurlteClient(Node):
    def __init__(self):
        super().__init__('chat_action_client')
        self._action_client = ActionClient(self, Chat, 'chat')
        self._set_prompt_cli = self.create_client(SetPrompt, "set_prompt")
        self._add_example_cli = self.create_client(AddExample, "add_example")
    
    def initialize(self):
        req = SetPrompt.Request()
        req.prompt = system_prompt
        self._set_prompt_cli.wait_for_service()
        set_prompt_future = self._set_prompt_cli.call_async(req)
        rclpy.spin_until_future_complete(self, set_prompt_future)
        
        for ex in examples:
            req = AddExample.Request()
            req.user = ex['user']
            req.assistant = ex['assistant']
            self._add_example_cli.wait_for_service()
            add_example_future = self._add_example_cli.call_async(req)
            rclpy.spin_until_future_complete(self, add_example_future)
        
        self.send_goal(user_prompt, slient=True)
    
    def send_goal(self, messages, slient=False):
        goal_msg = Chat.Goal()
        goal_msg.messages = messages

        self._action_client.wait_for_server()
        if slient:
            send_goal_future = self._action_client.send_goal_async(goal_msg)
        else:
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        get_result_future = send_goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        if not slient:
            print()

        return get_result_future.result().result.content

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback.delta, end='', flush=True)


def extract_python_code(content):
    code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        full_code = "\n".join(code_blocks)

        if full_code.startswith("python"):
            full_code = full_code[7:]

        return full_code
    else:
        return None


def interact_function(node):
    turtlebot = TurtleBot(node)
    while rclpy.ok():
        user_input = input('user: ')
        reply = node.send_goal(user_input)
        code = extract_python_code(reply)
        if code is not None:
            try:
                exec(code)
            except Exception as e:
                node.send_goal(str(e))


def main(args=None):
    rclpy.init(args=args)

    node = ChatTurlteClient()
    node.initialize()
    
    interact_thread = threading.Thread(target=interact_function, args=(node,))
    interact_thread.run()
    
    executer = SingleThreadedExecutor()
    executer.add_node(node)
    executer.spin()


if __name__ == '__main__':
    main()
