import datetime
import os
import json
import urllib.request
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

    def __init__(self, api_key, api_base, prompt=None):
        self.api_key = api_key
        self.api_base = api_base
        if prompt is None:
            current_date = datetime.datetime.now().strftime('%Y-%m-%d')
            self.prompt = f"You are ChatGPT, a large language model trained by OpenAI. Answer as concisely as possible. Knowledge cutoff: 2021-09-01, Current date: {current_date}."
        else:
            self.prompt = prompt
        self.reset()
    
    def _parse_stream(self, s):
        """Parse server-sent events"""
        for line in s:
            line = line.decode('utf-8')
            if line.startswith(':'):
                continue
            if line.strip() == "data: [DONE]":
                return None
            if line.startswith("data: "):
                yield json.loads(line[len("data: "):])

    def _request(self):
        """Send request to openai"""
        req = urllib.request.Request(
            url=f'{self.api_base}/chat/completions',
            data=json.dumps({
                "model": "gpt-3.5-turbo",
                "temperature": 0,
                "stream": True,
                "messages": self.messages
            }).encode('utf-8'),
            headers={
                "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36",
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}"
            }
        )
        return self._parse_stream(urllib.request.urlopen(req))

    def chat(self, messages):
        """
        Chat with chatgpt model.
        :param content: User input
        :return: A content generator
        """
        self.messages.append({"role": "user", "content": messages})
        
        collected_messages = []
        for chunk in self._request():
            chunk_message = chunk['choices'][0]['delta']
            collected_messages.append(chunk_message)
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
        # initialize node
        super().__init__('chatgpt_action_server')
        
        # load openai api key
        api_key = os.getenv("OPENAI_API_KEY")
        api_base = os.getenv("OPENAI_API_BASE", "https://api.openai.com/v1")
        assert api_key is not None, "OPENAI_API_KEY is not found in environment variables."

        # create chatbot object
        self._chatbot = ChatBot(api_key, api_base)
        
        # create ros action server
        self._action_server = ActionServer(self, Chat, 'chat', self._execute_callback)
        
        # create service
        self._reset_service = self.create_service(Empty, 'reset', self._reset_callback)
        self._set_prompt_service = self.create_service(SetPrompt, 'set_prompt', self._set_prompt_callback)
        self._add_example_service = self.create_service(AddExample, 'add_example', self._add_example_callback)

    def _execute_callback(self, goal_handle):
        messages = goal_handle.request.messages
        self.get_logger().info(f'Chat goal received: {messages}')
        
        # stream chat result
        feedback_msg = Chat.Feedback()
        for res in self._chatbot.chat(messages):
            feedback_msg.delta = res
            goal_handle.publish_feedback(feedback_msg)

        # chat finished
        goal_handle.succeed()

        # return total chat reply
        result = Chat.Result()
        result.content = self._chatbot.get_latest_content()
        self.get_logger().info('Chat finished.')
        return result

    def _reset_callback(self, request, response):
        """Reset chat history"""
        self._chatbot.reset()
        return response

    def _set_prompt_callback(self, request, response):
        """Set system prompt"""
        self._chatbot.prompt = request.prompt
        self._chatbot.reset()
        self.get_logger().info(f'Chat prompt changed: {self._chatbot.prompt}')
        return response

    def _add_example_callback(self, request, response):
        """Add example"""
        self._chatbot.messages.append({"role": "user", "content": request.user})
        self._chatbot.messages.append({"role": "assistant", "content": request.assistant})
        return response


def main(args=None):
    rclpy.init(args=args)

    chat_action_server = ChatActionServer()

    rclpy.spin(chat_action_server)


if __name__ == '__main__':
    main()
