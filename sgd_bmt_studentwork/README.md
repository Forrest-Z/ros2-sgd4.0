# Shared Dog

## Requirements

1. Docker and Docker Compose
2. Python v3.10
3. VSCode and VSCode Extensions recommendations

## Introduction

This repo has 4 important parts.

- 2. A Jupiter Notebook (located in `notebooks/dummy_broker`) starts an `MQTT broker` and a `dummy client` that publishes `dummy data` to the main `V2X topics` on localhost.
- 2. The `Node-RED` application is deployed as a `Docker` container for visualizing test data in action. This is an essential component of the entire system that allows us to connect to an `MQTT broker` (either real or dummy) and convert raw data into a valid JSON file. It consists of two flows: a `Test Flow` and a `Production Flow`. The test data mentioned can be collected through the `Test Flow` of the Node-RED app. The data is then processed and stored in a valid JSON file. The main difference between the `Test Flow` and `Production Flow` is that the second one is designed to connect to the actual MQTT Broker of the MK5.
- 3. The React application is deployed as a Docker container for the `Visualizer`, which allows viewing the collected map data. Specifically, it focuses on displaying the locations of the traffic lights that have been gathered by the MK5.Please keep in mind that two things are required. Firstly, you need a valid `Google Maps API key`. Secondly, you will need a valid JSON file processed by `Node-RED` because the MK5 stores the messages in a JSON file with an invalid format for the `Visualizer`.
- 4. There are two `ROS2` packages that can be deployed in a `production` environment. The first one is called `sdtl_interfaces`, which contains the main interfaces. The second one is named `sdtl_srv` and serves as the primary package housing the main service. It connects to the real `MQTT broker` initiated by the MK5. This `sdtl_srv` receives the current location of the system and an array of points that represent a route to follow. The output of the service will be the status of the next traffic light along the route.

## How to start

### Test environment for visualizing traffic lights

1. To begin, open the `notebooks/dummy_broker.ipynb` and execute the first five cells prior to the `clean up` section. It is suggested to stop the final cell, perform the clean up, and then restart the process every 15 minutes if you wish to conduct longer tests.
2. Then, start the `Node-RED` container by running the command `docker compose --profile development up nodered` in a terminal session. Next, navigate to http://localhost:1880 and open the `Test Flow` within the `Node-RED` interface. The `Test Flow` will automatically connect to the dummy MQTT broker. You can then click on the light blue square located to the right of the injection node called `create valid json`. This action will trigger a process task, generating a valid JSON file for the `Visualizer`.
3. Lastly, initiate the visualizer container by executing the command `docker compose --profile development up webdev` in a separate terminal session. Then, go to http://localhost:3000 in your web browser. You will be able to view the displayed traffic lights on the webpage. Dont forget your Google Maps API Key. This key could be set as env var in the docker-compose.yaml or after oppening http://localhost:3000

# Relevant data/links

- ISO/TS 19091:2019
- ISO/TS 19091 2019-06
- https://www.car-2-car.org/about-c-its/c-its-faqs
- https://www.car-2-car.org/about-c-its/c-its-glossary
- https://www.car-2-car.org/documents/general-documents
- https://standards.iso.org/iso/ts/19091/ed-2/en/ISO-TS-19091-addgrp-C-2018.asn
- Karte für v2x https://geoportal-hamburg.de/geo-online/
- https://tavf.hamburg/wir-sind-tavf#c36

This project was bootstrapped with [Create React App](https://github.com/facebook/create-react-app).

# Links for ROS 2

- Installation - https://docs.ros.org/en/foxy/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html#install-remote-development-extension
- Video series about devcontainer extensions for VS Code - https://www.youtube.com/watch?v=61M2takIKl8&list=PLj6YeMhvp2S5G_X6ZyMc8gfXPMFPg3O31
- Create a new package https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- Writing a simple publisher and subscriber (Python) - https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- Writing a simple service and client (Python) - https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
- Add Srv and msg - https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html

### How to build a ros2 package

- `rosdep update`
- in ros folder or under `/home/ws` run `rosdep install -i --from-path src --rosdistro foxy -y`
- `colcon build --packages-select sdtl_interfaces && source install/setup.bash` for building the nterfaces
- `colcon build --packages-select sdtl_action_server && source install/setup.bash` for building a server
- `colcon build --packages-select sdtl_action_client && source install/setup.bash` for building the test client

#### build the project at once

- under `/home/ws` run `install/setup.bash` and the `colcon build`
- `ros2 interface show sdtl_interfaces/action/SDTLPedestrianTrafficLight` for inspecting the action

### Run action server

- ensure you have build the sdtl_action_server package
- `source install/setup.bash`
- `ros2 run sdtl_action_server sdtl_action_server`
- `MQTT_HOST=10.23.24.3 MQTT_PORT=1883 ros2 run sdtl_action_server sdtl_action_server`
- `MQTT_HOST=10.23.24.3 MQTT_PORT=1883 ros2 run sdtl_action_server sdtl_action_server --ros-args --log-level debug`

### Sending a goal to the action server

- ros2 action send_goal pedestrian_traffic_light sdtl_interfaces/action/SDTLPedestrianTrafficLight "{route: [{latitude: 12.12, longitude: 13.13}]}"
- ros2 action send_goal --feedback pedestrian_traffic_light sdtl_interfaces/action/SDTLPedestrianTrafficLight "{route: [{latitude: 12.12, longitude: 43.43}, {latitude: 13.13, longitude: 44.44}]}"
- ros2 action send_goal --feedback pedestrian_traffic_light sdtl_interfaces/action/SDTLPedestrianTrafficLight "{route: [{latitude: 53.5559309, longitude: 9.9772394}, {latitude: 53.5559558, longitude: 9.9771795}, {latitude: 53.5559807, longitude: 9.9771196}, {latitude: 53.5560056, longitude: 9.9770596}, {latitude: 53.5560305, longitude: 9.9769997}, {latitude: 53.5560554, longitude: 9.9769398}, {latitude: 53.5560778, longitude: 9.9769635}, {latitude: 53.5561001, longitude: 9.9769872}, {latitude: 53.556126, longitude: 9.9770146}, {latitude: 53.5561519, longitude: 9.977042}, {latitude: 53.5561826, longitude: 9.9770747}, {latitude: 53.5562133, longitude: 9.9771073}, {latitude: 53.5562336, longitude: 9.9771345}, {latitude: 53.5562573, longitude: 9.9771066}, {latitude: 53.556281, longitude: 9.9770786}, {latitude: 53.5563022, longitude: 9.9770227}]}"

### c++ libs can not be found in container

Inside the container make sure you add file under `/home/ws/.vscode/c_cpp_properties.json` and add following configuration in order to link the .hpp files generated for the interfaces

    ```
        {
            "configurations": [
                {
                    "browse": {
                        "databaseFilename": "${default}",
                        "limitSymbolsToIncludedHeaders": false
                    },
                    "includePath": [
                        "/usr/include/**",
                        "/opt/ros/foxy/include/**",
                        "/home/ws/src/sdtl_action_client/include/**",
                        "/home/ws/src/sdtl_interfaces/include/**",
                        "${workspaceFolder}/install/sdtl_interfaces/include"
                    ],
                    "name": "ROS",
                    "intelliSenseMode": "gcc-arm64", # this property was set for an apple silicon. change this if needed to "${default}"
                    "compilerPath": "/usr/bin/gcc",
                    "cStandard": "gnu11",
                    "cppStandard": "c++14"
                }
            ],
            "version": 4
        }
    ```

# TODO

- mk5 widthout charge or shut down
- mk5 without gps signal
- mk5 with bad signal
- TODO: improve proccessRoute
- delete intersections that not longer needed. or implement a Quee. the last updated or created
