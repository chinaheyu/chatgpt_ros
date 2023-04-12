# chatgpt_ros

https://user-images.githubusercontent.com/59406481/230091094-198598c8-d4c2-4cd6-ad87-e34df3f56664.mp4

## Quick Start

First you need to get your api key from openai and set it as an environment variable.

```
export OPENAI_API_KEY=<your api key>
```

After that you can start the action server and thus establish communication with chatgpt.

```
ros2 run chatgpt_ros chatgpt_action_server
```

This command starts a node named `/chatgpt_action_server`, which provides the following services and actions.

Services:
- /reset (std_srvs/srv/Empty): Reset chat history.
- /set_prompt (chatgpt_ros_interfaces/srv/SetPrompt): Set system prompt.
- /add_example (chatgpt_ros_interfaces/srv/AddExample): Add example.

Actions:
- /chat (chatgpt_ros_interfaces/action/Chat): Chat with ChatGPT.

This is the description of the `/chat` action.

```
string messages
---
string content
---
string delta
```

The goal (messages) of the chat action is the user's input. The response from chatgpt will keep coming back from feedback (delta). When the chat ends, a full reply (content) is returned.

You can start an action client to test the `/chat` action.

```
ros2 run chatgpt_ros chatgpt_action_client
```

## Turtlesim Demo

We also provide a demo on how to control the turtlesim in ros using chatgpt. You can start the demo with the following command:

```
ros2 launch chatgpt_ros_turtlesim demo.py
```

This will pop up a separate command window for entering commands to control the turtle. You can instruct the turtles to draw graphics through natural language.
