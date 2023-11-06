# RoadSide Lidar Evalution
This repo contains the code we write to conduct an investigation over the roadside sensing ability of different types of Lidars.
## Carla Simulation
Prerequisites：
+ carla0.9.14 installed
### run the code
+ First, please make sure your python environment is ready by running
    ```bash
    pip install -r requirements.txt
    ```
+ Second, please run the simulation by
    ```python
    python roadsideLidar.py
    ```
### About the code
In code v0.1 a basic scene of roadside Lidar sensing has been established. It contains 3 main modules, namely:
+ a module that place a Lidar placed at roadside
+ a visualization module to visualize Lidar output in real-time
+ a module that spawns some npc cars(object detection targets)
### Code in the future
Lately, I am working on installing [a Lidar simulation Library](https://github.com/PJLab-ADG/PCSim/tree/main/LiDARSimLib) into CARLA in order to diversify the categories of Lidars the system can simulate.
The Next version of the code will focus on not only this part, but also the pipeline to generate good-to-go datasets.