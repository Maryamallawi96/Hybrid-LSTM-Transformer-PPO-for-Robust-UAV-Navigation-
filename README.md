# 🚁 Hybrid LSTM-Transformer PPO  
---
## 📝 **Overview**  
This project implements a novel deep reinforcement learning algorithm combining LSTM and Transformer architectures with Proximal Policy Optimization (PPO) to enhance UAV navigation in partially observable and dynamic environments. The approach balances short-term reactivity with long-term planning, enabling robust obstacle avoidance and efficient path planning using only LiDAR inputs.
---
## ✨ **Features**  
- 🔄 Hybrid PPO agent integrating LSTM and Transformer for memory-augmented navigation.  
- 🕹️ End-to-end training in Gazebo simulated 3D environments with dynamic obstacles.  
- 📡 Utilizes 2D LiDAR inputs for obstacle detection and environment perception.  
- 🛤️ Demonstrates improved path smoothness, fewer collisions, and stable policy convergence.  
- 🤖 Compatible with PX4 SITL, MAVROS, and ROS Noetic for real-world UAV deployment.

---

## 📂 **Repository Structure**

Hybrid LSTM-Transformer PPO/
├── launch/ # ROS launch files for simulation
├── models/ # UAV and obstacle models
├── scripts/ # Training code and environment wrapper
├── worlds/ # Gazebo simulation worlds
├── CMakeLists.txt # Build configurations
├── package.xml # ROS package manifest
└── README.md # This file


Hybrid LSTM-Transformer PPO/
├── launch/ # ROS launch files for simulation
├── models/ # UAV and obstacle models
├── scripts/ # Training code and environment wrapper
├── worlds/ # Gazebo simulation worlds
├── CMakeLists.txt # Build configurations
├── package.xml # ROS package manifest
└── README.md # This file

---

## ⚙️ **Installation & Dependencies**

- 🐢 ROS Noetic  
- 🌌 Gazebo 11  
- 🛩️ PX4 Autopilot SITL  
- 🚀 MAVROS  
- 🐍 Python packages:  
  - `stable-baselines3`  
  - `gymnasium`  
  - `torch`  
  - `numpy`  
  - `rospy`  

Install Python dependencies via:

```bash
pip install stable-baselines3[extra] gymnasium torch numpy rospy

---
▶️ Launch Simulation
bash
Copy
roslaunch hybrid_lstm_transformer_ppo simulation.launch
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

🌍 World File & Training Environments
The repository includes a comprehensive world file that contains both the training environment and the generalization environment, allowing the agent to learn and be evaluated in varied scenarios


🤖 Train Agent

rosrun hybrid_lstm_transformer_ppo train_hybrid_ppo.py
The UAV will autonomously navigate while learning to avoid obstacles, balancing reactive and strategic planning


📊 Results & Visualization  📸
The following media demonstrate the performance and capabilities of the Hybrid LSTM-Transformer PPO for UAV navigation:

🎯 Drone Navigation Through Obstacles
👁️ Test - Front View
🔝 Test - Top View

🏋️ Training Environments 
🌟 Unseen Environment  

## 🎯 Drone Navigation Through Obstacles
A video showcasing the drone successfully navigating through a complex obstacle course using the trained policy.  
[▶️ Watch Video](Media3/Drone%20Navigation%20Through%20Obstacles..MP4)

### 👁️ Test - Front View
A front-facing view of the drone during navigation trials.  
[▶️ Watch Video](Media3/Test%20(front%20view).mp4)

### 🔝 Test - Top View
A top-down view highlighting path planning and obstacle avoidance.  
[▶️ Watch Video](Media3/Test%20(Top%20view).mp4)

### 🏋️ Training Environments
Snapshot of simulated environments used for training.  
[Training Environments](Media3/Training%20environments.png)

### 🌟 Unseen Environment
Screenshot depicting the drone navigating a previously unseen environment.  
[Unseen Environment](Media3/unseen%20env.png)


## 👩‍💻 **Author**  
**Maryam Allawi**  
✉️ Email:pgs.maryam.allawi@uobasrah.edu.iq  
🌐 GitHub:[Maryamallawi96](https://github.com/Maryamallawi96)

---
**Thank you for checking out the project! Feel free to open issues or pull requests for improvements.** 🚀

