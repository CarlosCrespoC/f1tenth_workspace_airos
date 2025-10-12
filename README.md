# F1Tenth Workspace Tutorial

Welcome to the F1Tenth club repository! This guide will help you set up and use the workspace for F1Tenth competitions.

---

## 1. Prerequisites

- Ubuntu 20.04 or later
- ROS Noetic installed
- Git installed

---

## 2. Clone the Repository

```bash
cd ~/
mkdir -p f1tenth_ws/src
cd f1tenth_ws/src
git clone https://github.com/your-org/f1tenth_workspace_airos.git
```

---

## 3. Install Dependencies

```bash
cd ~/f1tenth_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. Build the Workspace

```bash
cd ~/f1tenth_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Connect to the Roboracer

- Ensure your laptop is connected to the same Wi-Fi network as the Roboracer.
- Find the Roboracer's IP address (ask your team lead or check the router).
- Test the connection:
    ```bash
    ping <roboracer-ip>
    ```

---

## 6. Launch Core ROS Nodes

```bash
roslaunch bringup bringup.launch
```
- This starts the essential nodes for communication and control.

---

## 7. Teleoperate the Roboracer

- Use the keyboard teleop node:
    ```bash
    roslaunch teleop_keyboard teleop_keyboard.launch
    ```
- Use arrow keys to control the car.

---

## 8. Run Autonomous Algorithms

- Launch your algorithm:
    ```bash
    roslaunch <your_package> <your_algorithm>.launch
    ```

---

## 9. Visualize Data

- Start RViz for visualization:
    ```bash
    rosrun rviz rviz
    ```

---

## 10. Troubleshooting

- Check ROS master URI and network settings.
- Use `rostopic list` and `rosnode list` to debug.

---

## 11. Additional Resources

- [F1Tenth Official Docs](https://f1tenth.org/)
- [ROS Wiki](http://wiki.ros.org/)

---

Happy racing!