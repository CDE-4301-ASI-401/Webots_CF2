# Webots_CF

A simulation framework for quadcopters using the Crazyflie platform within the Webots environment.
![image](RUN%201,%20SAFMC%201%20CONFIG%201.png) 

## Overview

This repository provides a simulation environment for the Crazyflie quadcopter using Webots. It was originally developed to support the development and optimization of the Swarm Gradient Bug Algorithm (SGBA) in simulation before deployment on real hardware.

## Features

- Accurate simulation of Crazyflie quadcopter dynamics
- Integration with Webots for 3D visualization and physics
- Modular controller structure for easy algorithm testing
- Basic wall-following and autonomous behaviors
- Realistic sensor feedback (e.g., flow deck, multiranger)

## Getting Started

### Prerequisites

- [Webots](https://cyberbotics.com/) (tested with version 2025a)
- Python (recommended 3.8+)
- Python packages: `pandas`, `matplotlib`

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/CDE-4301-ASI-401/Webots_CF.git

   ```
2. Open Webots and load the desired .wbt file from the worlds/ directory.

### Usage
1. Launch Webots and open the desired .wbt file (e.g., CompetitionLayout.wbt).
2. Assign a controller to each drone (e.g., SGBA_controller).
3. Start the simulation — the positions of each drone will be saved in the path_logs/ folder. 
4. To visualize the paths:
- Open plotter.ipynb in the path_visualization/ folder.
- Update the notebook to point to the correct drone IDs and file paths.
- Since the paths are plotted on a top-down image of the arena, you may need to generate your own image and set the correct path to it in the notebook.
