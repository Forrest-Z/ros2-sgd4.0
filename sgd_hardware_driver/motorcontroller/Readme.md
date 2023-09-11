# Motorcontroller


![Overview of publisher and subscriber for motorcontroller node](/doc/motorcontroller_node.png)


## Parameters

| Parameter Name | Type           | Default Value    | Description    |
| -------------- | -------------- | ---------------- | -------------- |
| log_dir        | String         | ".ros/log/"      | PLOG log directory |
| log_severity   | String         | "I"              | PLOG logging severity |
| odom_topic     | String         | "odom"           | Topic on which odometry data is published to the ros system |
| odom_sim_topic | String         | "odom/sim"       | Topic on which odometry data from the simulation is received |
| battery_state_topic | String    | "battery"        | Topic on which battery information (e.g. voltage) is published to the ros system |
| pico_base_pub_topic | String    | "pico_base_motor" | Topic on which motor commands are published to the hardware |
| sgd_move_topic | String         | "sgd_move_base"  | Topic on which move commands are received from the ros system |
| gps_topic      | String         | "gps"            | Topic on which gps position is received |
| imu_topic      | String         | "imu"            | Topic on which imu data is received
| pico_base_sub_topic | String    | "pico_base_odom" | Topic on which odometry data from the hardware is received |
| compass_gain   | Double         | 0.045            |  |
| gps_orientation_gain | Double   | 0.5              |  |
| wheel_separation | Double       | 0.71             | Separation of the wheels on the shared guide dog |
| wheel_circumference | Double    | 0.68             |  |
| sim_battery_volt | Double       | 37.0             | Battery voltage used in simulation mode |
| relative       | Boolean        | True             |  |


## Manual Mode

ros2 topic pub <topic_name> <msg_type> '<args>'

ros2 topic pub motor_desired_values geometry_msgs/msg/Quaternion "{x: 20, y: 20}"