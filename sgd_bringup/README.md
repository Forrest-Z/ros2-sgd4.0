# sgd_bringup

The `sgd_bringup` package contains launch files and configurations to start the shared guide dog.

## Launch *Simulation* with Gazebo

```bash
ros2 launch sgd_bringup sgd_startup.launch.py sim:=True
```

## Launch the *Robot*

```bash
ros2 launch sgd_bringup sgd_startup.launch.py
```

## Optional Launch Commands

```bash
ros2 launch sgd_bringup sgd_startup.launch.py slam:=True
```

Start the Shared Guide Dog / Simulation with SLAM algorithms (experimental feature).
