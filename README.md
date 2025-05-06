# Gazebo Presentation

## Instructions
Before every turtlebot3 command, run on the terminal:
```export TURTLEBOT3_MODEL=waffle_pi```

Or add it to your .bashrc profile or equivalent:
```echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc```

## Run Gazebo simulation
```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py```

## Run Simple Autonomous Drive
```ros2 run turtlebot3_gazebo turtlebot3_drive```

## Run SLAM and RViz
```ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True```

## Run python OpenCV ROS Node script
```source venv/bin/activate```

```python image_subscriber.py```
