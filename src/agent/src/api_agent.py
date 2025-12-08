#!/usr/bin/env python3
import os, json, ast, threading, queue
import regex as re
from pynput import keyboard
import time

from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
from langchain.prompts import PromptTemplate
from langchain_community.document_loaders import TextLoader
from langchain.text_splitter import CharacterTextSplitter
from langchain.chains import RetrievalQA
from langchain_community.embeddings import HuggingFaceEmbeddings
from langchain_community.vectorstores import FAISS
from langchain_openai import ChatOpenAI
from langchain.chains import LLMChain

import rospy
from std_msgs.msg import String

from tools import AgentTools

# print(os.getenv("OPENAI_API_KEY"))
# print(os.getenv("OPENAI_BASE_URL"))

# It is recommended to put keys in environment variables to avoid hardcoding
class AgentRobot:
    def __init__(self):
        self._init_agent()
        self.token_usage = []                             # Token usage per turn

        '''Interactive message processing'''
        self.cmd_queue = queue.Queue()                  # Queue to detect user input (input from both CLI and UI is stored here)
        threading.Thread(target=self.cli_input_thread, daemon=True).start()
        
        self.interrupt_flag = False                     # Interrupt flag, only True when a command is entered during loop execution
        self.listener = keyboard.Listener(on_press=self._listen_for_interrupt)
        self.listener.daemon = True
        self.listener.start()
        rospy.Subscriber('/agent_node/user_command', String, self.ui_command_callback)          # Receive user commands from the UI side

        '''History'''
        self.looping = False                            # Whether the loop is currently executing
        self.log_pub = rospy.Publisher('/agent_node/agent_log', String, queue_size=50)          # Publish history logs to the UI side

    def _init_agent(self):
        '''Model and agent tools'''
        self.llm = ChatOpenAI(
            model="qwen3-max", 
            temperature=0.7,
        )
        self.agent_tool = AgentTools()

        '''Dialogue LLMChain construction'''
        tool_description_str = self.agent_tool.get_tools_description()
        print(f'🧰 Tools:\n{tool_description_str}\n')
        prompt_template = PromptTemplate(
            input_variables=["question"],
            template=f"""
            1. You need to **think carefully**, **follow user instructions and requirements**, and **take action**. Finally, you must only answer with valid double-quoted JSON format data containing the attributes:
            message: Used for your thought content or what you want to say to the user (within 300 words),
            action: The tool function to be called,
            action_input: The input parameters for the tool (if the tool has no input, use None).
            - If you believe the final task is indeed completed, or if you encounter a situation requiring further user instructions, fill 'Final' in 'action' and the final complete answer in 'message'.
            
            2. The only tools available are as follows. Please think flexibly about which tool to call at each step:
            {tool_description_str}.
            
            3. The history information I give you will include the following:
            - My task 'Question'. There may be multiple tasks; the latest one is the current task;
            - Your historical 'Output';
            - The 'Observation' returned from tool calls;
            - User's intermediate input 'UserInput';
            - Your previous 'Answer' to the user.
            
            Now answer my command:
            Scenario: You are an operator of a robot (UAV/UGV/Robotic Dog, etc.). If operating a UAV, you must maintain sufficient flight altitude (>= 2m). The equipped camera has a field of view of over 20m and can only see the view directly in front of the robot;
            Task: {{question}}
        """
        )
        self.chain = LLMChain(
            prompt=prompt_template,
            llm=self.llm,
        )

    def cli_input_thread(self):
        '''Detect user input in the command line'''
        while True:
            try:
                user_input = input("\n")
                self.cmd_queue.put(user_input)
            except EOFError:
                break
            
    def ui_command_callback(self, msg):
        '''Receive user commands from the UI side'''
        self.cmd_queue.put(msg.data)
        if self.looping:
            self.interrupt_flag = True

    def _listen_for_interrupt(self, key):
        """Background thread to listen for user input. Triggers an interrupt when the ESC key is detected, allowing the agent to pause for further input."""
        try:
            if key == keyboard.Key.esc and self.looping:   # Press ESC
                # print("⚠️ User interruption detected. Pausing after the current turn is completed.")
                self.interrupt_flag = True
        except AttributeError:
            pass

    def extract_json_from_response(self, response: str):
        '''Parse JSON'''
        json_candidates = re.findall(r'\{(?:[^{}]|(?R))*\}', response)

        valid_jsons = []
        for candidate in json_candidates:
            try:
                obj = json.loads(candidate)  # Try standard JSON first
                valid_jsons.append(obj)
            except json.JSONDecodeError:
                try:
                    # Use ast.literal_eval to support single-quote style
                    obj = ast.literal_eval(candidate)
                    valid_jsons.append(obj)
                except (ValueError, SyntaxError):
                    continue
        return valid_jsons

    def run_one_task(self, user_input, history=None):
        '''Run a single task'''
        self.looping = True
        print(f'❓ {user_input}\n')
        history = f"{history}\n{user_input}" if history else user_input
        turn = 0

        while True:
            print(f"\033[34m---------------------------Turn {turn + 1}--------------------------------\033[0m")
            complete_response = self.chain.generate([{"question": history}])                # Obtain complete return data
            response = complete_response.generations[0][0].text
            self.token_usage.append(complete_response.llm_output["token_usage"]['total_tokens'])

            parsed_data = self.extract_json_from_response(response)[-1]
            if parsed_data:
                # print(f"📧 Successfully parsed as Python object: {parsed_data}")
                pass

            if parsed_data['action'] == 'Final':
                print(f"\n✅ Final Answer: {parsed_data['message']}\n")
                history = f"{history}\nAnswer: {parsed_data['message']}"
                self.log_pub.publish(json.dumps({"role": "assistant", "type": "Answer", "content": parsed_data['message']}))
                break
            else:
                # Print LLM output, find tool
                print(f"📧 Thought: {parsed_data['message']}\n")
                self.log_pub.publish(json.dumps({"role": "assistant", "type": "Output", "content": parsed_data}))
                tool = next(tool for tool in self.agent_tool.tools if tool.name == parsed_data['action'])
                print(f"🧰 Action: Calling tool {tool.name}, Input: {parsed_data['action_input']}")
                
                # Call tool, process return result
                result = tool.func(parsed_data['action_input'])
                result = result['result'] if isinstance(result, dict) else result
                self.log_pub.publish(json.dumps({"role": "assistant", "type": "Observation", "content": result}))
                print(f"🔥 Observation: {result}\n")

                history = f"{history}\nOutput: {parsed_data}\nObservation: {result}"
                turn += 1

            rospy.sleep(1)

            # Check for interrupt requests
            if self.interrupt_flag:
                print("\033[33m---------------------------⏸PAUSED---------------------------\033[0m")
                new_instruction = self.cmd_queue.get()                  # Get user input from queue
                print(f"👉 User input detected: {new_instruction}\n")
                self.log_pub.publish(json.dumps({"role": "user", "type": "UserInput", "content": new_instruction}))
                
                history = f"{history}\nUserInput: {new_instruction}"
                self.interrupt_flag = False

        self.looping = False
        print(f"\033[32m---------------------------COMPLETED--------------------------------\033[0m")
        return history

    def run_loop(self, first_question=None):
        """Execute multiple tasks in a loop, user chooses whether to clear history.
           first_question: Optional initial task (string)
        """
        history = None
        total_time = 0

        # If there is a default question, execute it once first
        if first_question:
            self.log_pub.publish(json.dumps({"role": "user", "type": "UserInput", "content": first_question}))
            history = self.run_one_task("Question: " + first_question, history)

        while not rospy.is_shutdown():
            user_input = self.cmd_queue.get()                           # Get user input from queue
            if user_input.lower() in ["exit", "quit", "q"]:             # End program
                print("👋 Terminating program.\n")
                break
            self.log_pub.publish(json.dumps({"role": "user", "type": "UserInput", "content": user_input}))

            # Execute task
            time_start = time.time()
            history = self.run_one_task("Question: " + user_input, history)
            one_time = time.time() - time_start
            total_time += one_time
            print(f"✅ Time this run: {one_time:.2f}s, Cumulative time: {total_time:.2f}s\nToken consumption: {sum(self.token_usage)}\nMotion path: {self.agent_tool.path_record}\n")
        
        print(f'📂 ALL: {history}')


# ======================
# Main Loop
# ======================
if __name__ == "__main__":
    rospy.init_node('agent_node')
    agent_robot = AgentRobot()
    rospy.sleep(1)
    agent_robot.run_loop()
    rospy.spin()