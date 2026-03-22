# Control Software for Autonomous Inspection of Flat Roofs

This project presents the development of control software for a mobile robot designed to perform **autonomous inspection of drains and covers** in buildings with flat roofs, specifically for the School of Electrical and Electronic Engineering (EIEE) at the **Universidad del Valle**.

---

## 📝 Project Description
The maintenance of flat roofs is vital to prevent obstructions in rainwater evacuation ducts, which cause leaks and structural deterioration. In tropical environments like Cali, organic waste is the main source of clogging. 

This system automates the inspection patrol to:
* **Reduce occupational risks**: It prevents maintenance personnel from working at heights without guardrails and in difficult-to-access conditions.
* **Optimize resources**: It allows for the early detection of obstructions through automatic alerts.

## 🛠️ System Architecture
The software uses a centralized architecture based on the **ROS 1 (Robot Operating System)** middleware.

### Main Components:
* **Graphical User Interface (GUI)**: Developed in Python with the **PyQt5** library to manage inspection schedules and routes.
* **Control Software**: Manages inspection actions and the robot's autonomy.
* **Artificial Vision Module**: Uses Convolutional Neural Networks (CNN) and *One-shot learning* techniques to compare the current state of the drains with reference images.
* **Database**: Stores points of interest, patrol reports, and generated alerts.

## 🚀 Technical Capabilities
* **Navigation and Mapping**: Implementation of **SLAM** (Simultaneous Localization And Mapping) algorithms for robot self-referencing.
* **Trajectory Planning**: 
    * **Global**: Use of algorithms such as **A-star (A*)** and **Dijkstra** to find optimal routes.
    * **Local**: Real-time obstacle avoidance using **Dynamic Window Approach (DWA)** or **Timed Elastic Band (TEB)**.
* **Waste Detection**: A system capable of identifying physical changes in the environment by measuring the percentage of similarity between feature vectors.

---
**Author:** Harold Andres Riascos Manyoma  
**Director:** Eval Bladimir Bacca Cortes, Ph.D.  
**Institution:** Universidad del Valle, Faculty of Engineering.
