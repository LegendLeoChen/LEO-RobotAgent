from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import json
import os
import psutil

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})  # Allow cross-origin access

processes = {}
# Task JSON file
TASKS_FILE = os.path.join(os.path.dirname(__file__), "../config/service.json")
# Question JSON file
QUESTIONS_FILE = os.path.join(os.path.dirname(__file__), "../config/questions.json")
# Tools JSON file
TOOLS_FILE = os.path.join(os.path.dirname(__file__), "../config/tools.json")

# ======== Terminal launch ROS command ========
def run_cmd(name, cmd):
    if name in processes:
        processes[name].terminate()
    full_cmd = f"{cmd}; exec bash"
    terminal_cmd = f"gnome-terminal -- bash -c \"{full_cmd}\""
    print(f"🚀 Start new terminal: {terminal_cmd}")
    processes[name] = subprocess.Popen(terminal_cmd, shell=True)

@app.route('/start', methods=['POST'])
def start_ros_process():
    data = request.get_json()
    target = data.get("target")
    args = data.get("args", "")

    cmd = f"{target} {args}".strip()

    run_cmd(f"{target.replace('/', '_')}", cmd)
    return jsonify({"status": "ok", "msg": f"Start: {target}"})

# ======== Get task list ========
@app.route('/tasks', methods=['GET'])
def get_tasks():
    if not os.path.exists(TASKS_FILE):
        print("No tasks config file found, creating a new one")
        with open(TASKS_FILE, "w", encoding="utf-8") as f:
            json.dump({"buttons": []}, f, indent=2)
    with open(TASKS_FILE, "r", encoding="utf-8") as f:
        tasks = json.load(f)
    return jsonify(tasks)

# ======== Modify task list ========
@app.route('/tasks', methods=['POST'])
def add_task():
    new_task = request.get_json()
    if not new_task.get("label") or not new_task.get("cmd"):
        return jsonify({"success": False, "msg": "label/cmd is required."})

    # Read existing tasks
    if os.path.exists(TASKS_FILE):
        with open(TASKS_FILE, "r", encoding="utf-8") as f:
            tasks = json.load(f)
    else:
        tasks = {"buttons": []}

    # Add new task
    tasks["buttons"].append(new_task)

    # Save back to file
    with open(TASKS_FILE, "w", encoding="utf-8") as f:
        json.dump(tasks, f, indent=2, ensure_ascii=False)

    return jsonify({"success": True})

# ===== Read preset question list =====
@app.route('/questions', methods=['GET'])
def get_questions():
    if not os.path.exists(QUESTIONS_FILE):
        # Init empty question list
        with open(QUESTIONS_FILE, "w", encoding="utf-8") as f:
            json.dump({"question": []}, f, indent=2, ensure_ascii=False)
    with open(QUESTIONS_FILE, "r", encoding="utf-8") as f:
        questions = json.load(f)
    return jsonify(questions)

# ======== Get tool list ========
@app.route('/tools', methods=['GET'])
def get_tools():
    """Read tools.json and return"""
    if not os.path.exists(TOOLS_FILE):
        print("⚠️ Tools config file not found, creating a new one.")
        with open(TOOLS_FILE, "w", encoding="utf-8") as f:
            json.dump({"tools": []}, f, indent=2, ensure_ascii=False)

    with open(TOOLS_FILE, "r", encoding="utf-8") as f:
        tools = json.load(f)
    return jsonify(tools)

# ======== Overwrite update tools list ========
@app.route('/tools', methods=['POST'])
def update_tools():
    """
    Receive complete tools JSON and overwrite local tools.json
    Expected frontend format:
    {
      "tools": [
        {"name": "uav_fly", "description": "...", "activation": true},
        {"name": "car_run", "description": "...", "activation": false}
      ]
    }
    """
    data = request.get_json()
    if not data or "tools" not in data:
        return jsonify({"success": False, "msg": "Invalid JSON, key 'tools' missing"})

    # Validate structure
    tools = data["tools"]
    if not isinstance(tools, list):
        return jsonify({"success": False, "msg": "tools must be a list"})

    for t in tools:
        if "name" not in t or "description" not in t or "activation" not in t:
            return jsonify({"success": False, "msg": f"Invalid tool format: {t}"})

    # Write to file
    with open(TOOLS_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"✅ Tools config updated, total {len(tools)} tools")
    return jsonify({"success": True, "msg": f"Successfully updated {len(tools)} tools"})

# ======== Check agent node running status ========
@app.route('/agent_status', methods=['GET'])
def agent_status():
    running = False
    for proc in psutil.process_iter(['cmdline']):
        try:
            cmdline = ' '.join(proc.info['cmdline'])
            if 'rosrun' in cmdline and 'api_agent.py' in cmdline:
                running = True
                break
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    return jsonify({"running": running})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
