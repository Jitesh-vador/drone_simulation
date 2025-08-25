# 🚁 Drone Simulation

**A 3D simulation of a path-finding drone with dynamic obstacles** designed to simulate real-time drone behavior and test path-planning algorithms in a physics environment.

> Lightweight desktop simulation intended for research, learning, and demoing pathfinding in dynamic environments.

---

## 🔍 Overview

This project simulates a drone navigating a 3D environment with moving obstacles. It is useful for experimenting with:
- Path planning (A*, D\*, RRT, etc.)
- Real-time obstacle avoidance
- Physics-based motion using a simulator (e.g., PyBullet)
- Visualizing agent behavior in a dynamic world

This repo contains the core simulation script(s), example environments, and utilities to run experiments locally.

---

## ✨ Key Features

- 3D physics-based environment for drone movement
- Dynamic obstacles that move or appear/disappear during simulation
- Path-finding integration (A* / grid-based planner — extendable)
- Visual visualization of drone, obstacles, and planned path
- Modular code to plug in different planners or controllers

---

## 📁 Project Structure (example)

> Update these names if your files/folders differ.

drone_simulation/
├── drone3.py # Main simulation script (entrypoint)
├── README.md # This documentation


## 🖼 Screenshots / Video Placeholders

Replace these with your actual images or short demo GIFs/videos (put files in `screenshots/` or `media/`):

- **Simulation View (Overview)**  
  <img width="1256" height="949" alt="image" src="https://github.com/user-attachments/assets/e49d5df3-1f2b-451d-9987-9166d9615c35" />


- **Dynamic Obstacles in Motion**  
  <img width="999" height="911" alt="image" src="https://github.com/user-attachments/assets/a0c1c8fb-b5ec-4426-8416-c7386a44605c" />


## ⚙️ Requirements

Below are common packages used in drone/physics simulation projects. Adjust as needed for your repo.

- Python 3.8+
- pybullet
- numpy
- scipy
- matplotlib (optional — for plots)
- pygame (optional — if using a custom renderer)
- networkx (optional — if using graph-based planners)



## 🚀 Installation & Quick Start

1. **Clone the repository**

git clone https://github.com/Jitesh-vador/drone_simulation.git
cd drone_simulation
Create a virtual environment (recommended)

python3 -m venv venv
source venv/bin/activate      # Windows: venv\Scripts\activate
Install dependencies

pip install -r requirements.txt
Run the simulation

If your main script is drone3.py, run:

python drone3.py
If your script accepts flags, try:

python drone3.py --map envs/simple_map.json --planner astar --visual true
(Adjust flags to match your actual CLI options — if none exist, simply run the script and follow prompts.)

🧭 How to Use / Typical Workflow
Choose or create an environment in envs/ (terrain, obstacles, start/goal).

Select a planner in planners/ (A*, RRT, etc.) or implement your own.

Configure simulation parameters (drone dynamics, obstacle speed, timestep).

Launch the simulation and observe:

Drone following the planned path

Path re-planning if obstacles block the route

Logging of metrics (path length, time, collisions)

📐 Example Config (envs/example_config.json)

{
  "map": "envs/simple_map.csv",
  "start": [0, 0, 1],
  "goal": [20, 20, 1],
  "obstacles": [
    {"type": "moving_box", "pos": [5,5,0], "velocity":[0.2,0,0]},
    {"type": "moving_box", "pos": [10,10,0], "velocity":[-0.15,0.1,0]}
  ],
  "planner": "astar",
  "timestep": 0.01
}
✅ Tips & Extending the Project
Add more planners: Implement D* Lite, RRT*, or potential fields under planners/.

Controller improvements: Replace simple waypoint following with PID or MPC controllers.

Sensor models: Simulate noisy sensors (lidar, depth camera) and use them for online obstacle detection.

Benchmarking: Add scripts to run multiple scenarios and log success rate, time, collisions.

GUI / Web UI: Create a small web dashboard (Flask + WebGL) or use a recording to show demos.

🧪 Tests & Experiments
Add experiment scripts under experiments/ to reproduce results:

experiments/run_batch.py — run multiple maps/planners and log metrics.

experiments/visualize_results.py — aggregate and plot performance.

🙌 Contributing
Contributions, issues, and feature requests are welcome!

Fork the project

Create a new branch: git checkout -b feature/awesome-stuff

Make your changes and commit: git commit -am 'Add new planner'

Push to the branch: git push origin feature/awesome-stuff

Open a Pull Request

Please include a short description of your changes and related tests/examples.



Acknowledgements
PyBullet authors and the physics-simulation community

Open-source pathfinding implementations (A*, RRT) for reference

Any datasets or tools on which you based the simulation
