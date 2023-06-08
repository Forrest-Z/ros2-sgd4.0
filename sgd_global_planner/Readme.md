# Global Planner


## Parameters

| Parameter Name  | Type           | Default Value    | Description    |
| --------------- | -------------- | ---------------- | -------------- |
| log_dir         | String         | ".ros/log/"      | PLOG log directory |
| log_severity    | String         | "I"              | PLOG logging severity |
| waypoints_topic | String         | "global_plan"    | Topic on which the computed global plan is published |
| clicked_point_topic | String     | "clicked_point"  | Topic on which to subscribe to clicked point (from rviz) |
| map_info_srv    | String         | "get_map_info"   | Service that provides map info |
| global_plan_srv | String         | "get_global_plan"| Service to calculate a new global plan |
| yaml_filename   | String         | "map.yaml"       | Yaml to read information about the map from |
| global_frame    | String         | "earth"          | Name of the global frame |
| map_frame       | String         | "map"            | Name of the local (map) frame |
| robot_base_frame| String         | "base_link"      | Name of the robot base frame |
