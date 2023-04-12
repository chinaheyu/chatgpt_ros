import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from chatgpt_ros_interfaces.action import Chat


class ChatActionClient(Node):

    def __init__(self):
        super().__init__('chatgpt_action_client')
        self._action_client = ActionClient(self, Chat, 'chat')

    def send_goal(self, messages):
        goal_msg = Chat.Goal()
        goal_msg.messages = messages

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        print()
        self.send_goal(input("Input: "))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback.delta, end='', flush=True)


def main(args=None):
    rclpy.init(args=args)

    action_client = ChatActionClient()

    action_client.send_goal(input("Input: "))

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
