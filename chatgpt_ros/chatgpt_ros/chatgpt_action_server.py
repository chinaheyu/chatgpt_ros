import datetime
import openai
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from chatgpt_ros_interfaces.action import Chat
from chatgpt_ros_interfaces.srv import SetPrompt, AddExample
from std_srvs.srv import Empty


class ChatBot:
    """
    Chatbot class for conversation.
    """

    def __init__(self, prompt=None):
        if prompt is None:
            current_date = datetime.datetime.now().strftime('%Y-%m-%d')
            self.prompt = f"You are ChatGPT, a large language model trained by OpenAI. Answer as concisely as possible. Knowledge cutoff: 2021-09-01, Current date: {current_date}."
        else:
            self.prompt = prompt
        self.reset()

    def chat(self, messages):
        """
        Chat with chatgpt model.
        :param content: User input.
        :return: None
        """
        self.messages.append({"role": "user", "content": messages})
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.messages,
            temperature=0,
            stream=True
        )
        # create variables to collect the stream of chunks
        collected_chunks = []
        collected_messages = []
        # iterate through the stream of events
        for chunk in response:
            collected_chunks.append(chunk)  # save the event response
            chunk_message = chunk['choices'][0]['delta']  # extract the message
            collected_messages.append(chunk_message)  # save the message
            yield chunk_message.get('content', '')

        full_reply_role = ''.join([m.get('role', '') for m in collected_messages])
        full_reply_content = ''.join([m.get('content', '') for m in collected_messages])
        self.messages.append({"role": full_reply_role, "content": full_reply_content})
    
    def get_latest_content(self):
        return self.messages[-1]['content']
    
    def reset(self):
        self.messages = [
            {
                "role": "system",
                "content": self.prompt
            }
        ]


class ChatActionServer(Node):
    def __init__(self):
        super().__init__('chat_action_server')
        self._action_server = ActionServer(self, Chat, 'chat', self.execute_callback)
        self._reset_service = self.create_service(Empty, 'reset', self.reset_callback)
        self._set_prompt_service = self.create_service(SetPrompt, 'set_prompt', self.set_prompt_callback)
        self._add_example_service = self.create_service(AddExample, 'add_example', self.add_example_callback)
        self._chatbot = ChatBot()

    def execute_callback(self, goal_handle):
        messages = goal_handle.request.messages
        self.get_logger().info(f'Chat goal received: {messages}')
        
        feedback_msg = Chat.Feedback()
        for res in self._chatbot.chat(messages):
            feedback_msg.delta = res
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = Chat.Result()
        result.content = self._chatbot.get_latest_content()
        self.get_logger().info('Chat finished.')
        return result

    def reset_callback(self, request, response):
        self._chatbot.reset()
        return response

    def set_prompt_callback(self, request, response):
        self._chatbot.prompt = request.prompt
        self._chatbot.reset()
        self.get_logger().info(f'Chat prompt changed: {self._chatbot.prompt}')
        return response

    def add_example_callback(self, request, response):
        self._chatbot.messages.append({"role": "user", "content": request.user})
        self._chatbot.messages.append({"role": "assistant", "content": request.assistant})
        return response


def main(args=None):
    rclpy.init(args=args)

    chat_action_server = ChatActionServer()

    rclpy.spin(chat_action_server)


if __name__ == '__main__':
    main()
