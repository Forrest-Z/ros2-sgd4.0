# Lifecycle Manager

## State Machine

Die State Machine monitors den State der nodes. 

Frage: Wie kann man erkennen, dass die Node nicht mehr arbeitet?
- Bei einem normalen Shutdown gibt es eine Notification über das Topic /transition_event
- Bei einem Crash wird eine Node exit_publisher gestartet, die eine Nachricht mit dem Nodenamen auf dem Topic 'transition_event' published

## Manual transition

```
ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

## Possible States

Die State IDs sind zusätzlich in der [Foxy Dokumentation](https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/State.html) beschrieben.


| Name | Value (uint8_t)     |
| ---- | -----               |
| PRIMARY_STATE_UNKNOWN            | 0  |
| PRIMARY_STATE_UNCONFIGURED       | 1  |
| PRIMARY_STATE_INACTIVE           | 2  |
| PRIMARY_STATE_ACTIVE             | 3  |
| PRIMARY_STATE_FINALIZED          | 4  |
| TRANSITION_STATE_CONFIGURING     | 10 |
| TRANSITION_STATE_CLEANINGUP      | 11 |
| TRANSITION_STATE_SHUTTINGDOWN    | 12 |
| TRANSITION_STATE_ACTIVATING      | 13 |
| TRANSITION_STATE_DEACTIVATING    | 14 |
| TRANSITION_STATE_ERRORPROCESSING | 15 |


/rosapi/nodes [rosapi_msgs/srv/Nodes]