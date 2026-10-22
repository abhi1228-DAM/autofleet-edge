# AutoFleet-Edge: Autonomous Mobile Robot (AMR) Navigation Engine with Natural Language Task Orchestrator

## 🚀 Overview
AutoFleet-Edge is an advanced robotics software framework designed to bridge unstructured human natural language instructions with deterministic ROS 2 Navigation (Nav2) stacks. It features an LLM agent orchestrator that parses user intent into structured navigation actions with safety guardrails.

## 🛠️ Tech Stack
- **Robotics Middleware:** ROS 2 (Humble), Nav2, Gazebo Simulation
- **AI & Agentic Engine:** Python, Pydantic, LangChain/OpenAI Schemas
- **Environment & DevOps:** Docker, VS Code DevContainers, macOS Silicon compatible

## 📂 System Architecture
1. **Natural Language Parser:** Translates prompt strings into validated spatial coordinates ($x, y$).
2. **ROS 2 Bridge Node:** Publishes standard `geometry_msgs/PoseStamped` goals to navigation topics.
3. **Safety Evaluation Layer:** Deterministic fallback rules rejecting unmapped or unsafe destinations.

## 🏃 Quick Start (Docker)
```bash
docker build -f .devcontainer/Dockerfile -t autofleet-ros2 .
docker run --rm -it --net=host -v $(pwd):/ros2_ws autofleet-ros2 python3 /ros2_ws/ros_action_bridge.py
### **What would you like to do next?**
1. Initialize Git and push this repository to your GitHub profile (`abhi1228-DAM`)?
2. Draft the exact high-engagement **LinkedIn post copy** with a technical hook to showcase your project to global recruiters?
