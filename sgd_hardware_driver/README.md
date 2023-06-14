# Hardware Driver

In diesem Unterordner sind alle Hardware Treiber, die für den Betrieb des Shared Guide Dog benötigt werden, enthalten.

## Capacitive Touch

Sensormodell: `Eigenbau`

Der Capacitive Touch Sensor ist an den Handgriffen des Shared Guide Dog befestigt und erkennt, ob der Nutzer/die Nutzerin die Handgriffe festhält. Ist dies nicht der Fall, wird der Shared Guide Dog gestoppt.

**Hinweis:** Der Shared Guide Dog besitzt derzeit keine Touch-Sensoren an den Handgriffen

| Topic | Message Type |
| ----- | ------------ |
| /cap_touch | sgd_msgs/Touch |

## FrSky RX8R

Sensormodell: `FrSky RX8R`
[Produktwebsite](https://www.frsky-rc.com/product/rx8r/)

Der FrSky RX8R Empfänger ermöglicht die Steuerung des Shared Guide Dog mit einer Fernbedienung. Derzeit wird eine FrSky Taranis X9D Plus ([Link](https://www.frsky-rc.com/product/taranis-x9d-plus-2019/)) verwendet. Über 8 Kanäle kann der Shared Guide Dog bewegt und die Lichter gesteuert werden. Die Fernbedienung kann im Master Modus betrieben werden und überschreibt damit alle anderen Bewegungsbefehle und Schutzmechanismen.

| Topic | Message Type |
| ----- | ------------ |
| /cmd_vel_master | geometry_msgs/Twist |
| /cmd_vel_frsky  | geometry_msgs/Twist |
| /lights | sgd_msgs/Light |

## GPS

Sensor: `SparkFun GPS-RTK-SMA Breakout - ZED-F9P`
[Produktwebsite](https://www.sparkfun.com/products/16481)

Die Position wird im lokalen und im globalen WGS84 Koordinatensystem ausgegeben. Zusätzlich wird die UTC Zeit bereitgestellt.

Der Receiver kann RTCM Korrekturdaten verarbeiten. Um den Ntrip Client zu aktivieren, müssen in den Parametern ein Ntrip Server, Port, Mountpoint und die Authentifizierung angegeben werden.

Für die Nutzung von SAPOS und anderen Ntrip Servern ist eine Authentifizierung notwendig. Der Nutzername und Passwort können mit dem Hilfsprogramm ntrip_auth in den benötigten Authentifizierungsstring umgewandelt werden. Eine direkte Eingabe von Nutzername und Passwort ist nicht vorgesehen.

| Published Topic | Message Type |
| --------------- | ------------ |
| /gps            | sensor_msgs/NavSatFix |
| /gps/local      | geometry_msgs/PoseWithCovarianceStamped |
| /clock/utc      | builtin_interfaces/msg/Time |
| /visualize/gnss | visualization_msgs/msg/Marker |


### Parameters

| Parameter Name  | Type           | Default Value    | Description    |
| --------------- | -------------- | ---------------- | -------------- |
| log_dir         | String         | ".ros/log/"      | PLOG log directory |
| log_severity    | String         | "I"              | PLOG logging severity |
| port            | String         | "/dev/novalue"   | Topic on which the computed global plan is published |
| parser_file     | String         | ""               | Topic on which to subscribe to clicked point (from rviz) |
| parser_type     | String         | "nmea"           | Service that provides map info |
| publish_local_pose | Boolean     | true             | Service to calculate a new global plan |
| publish_tf      | Boolean        | true             | Yaml to read information about the map from |
| local_pose_topic | String        | "gps/local"      | Name of the global frame |
| gps_sim_topic   | String         | "gps/sim"        | Name of the local (map) frame |
| utc_topic       | String         | "clock/utc"      | Name of the robot base frame |
| gps_topic       | String         | "gps"            | Name of the robot base frame |
| visualize_gnss_topic | String    | "visualize/gnss" | Name of the robot base frame |
| transform_to_base_link | Boolean | true             | Name of the robot base frame |
| base_link_frame_id | String      | "base_link"      | Name of the robot base frame |
| odom_frame_id   | String         | "odeom"          | Name of the robot base frame |
| ntrip_server    | String         | ""               | See section 'How to use SAPOS' |
| ntrip_port      | String         | ""               | See section 'How to use SAPOS' |
| ntrip_mountpoint | String        | ""               | See section 'How to use SAPOS' |
| ntrip_auth      | String         | ""               | See section 'How to use SAPOS' |
| ntrip_send_nmea | String         | ""               | See section 'How to use SAPOS' |

### How to use SAPOS


| Parameter Name  | Type           | Default Value    | Description    |
| --------------- | -------------- | ---------------- | -------------- |
| ntrip_server    | String         | ""               | Name of the robot base frame |
| ntrip_port      | String         | ""               | Name of the robot base frame |
| ntrip_mountpoint | String        | ""               | Name of the robot base frame |
| ntrip_auth      | String         | ""               | Name of the robot base frame |
| ntrip_send_nmea | String         | ""               | Name of the robot base frame |

### Visualisierung


Color depends on gnss fix status:

| value | name       | color  | hex     |
| ----- | ----       | -----  | -----   |
| 0     | not fixed  | grey   | #8a8a8a |
| 1     | standalone | pink   | #00ffff |
| 2     | DGPS       | red    | #c80000 |
| 3     | GPS PPS    | yellow | #ffff00 |
| 4     | RTK fixed  | green  | #00c800 |
| 5     | RTK float  | orange | #ff8700 |


## IMU

Sensormodell: `Bosch BNO055`
[Produktwebsite](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)

Die IMU liefert eine absolute Ausrichtung des Shared Guide Dog und die Beschleunigung und Drehrate.

| Topic | Message Type |
| ----- | ------------ |
| /imu  | sensor_msgs/Imu |

## Laser 1D

Sensormodell: `Pololu VL53L1X`
[Produktwebsite](https://www.pololu.com/product/3415)

Der 1D Lasersensor ist zwischen den Handgriffen befestigt und überwacht die Distanz zwischen dem Shared Guide Dog und dem Nutzer. Wird der Abstand zu groß, verringert sich die Geschwindigkeit, wird der Abstand kleiner, erhöht sich die Geschwindigkeit.

| Parameter | Standardwert | Beschreibung |
| --------- | ------------ | ------------ |
| 

| Topic | Message Type |
| ----- | ------------ |
| /cmd_vel_contr  | geometry_msgs/Twist |

## LED Strips

[Produktwebsite](https://www.adafruit.com/product/3811)

Die LED Strips sind seitlich am Fahrzeug angebracht. Sie warnen und signalisieren der Umwelt die Intention des Shared Guide Dog.

## Lidar

Sensormodell: `Sick TiM571-2050101`
[Produktwebsite](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim5xx/tim571-2050101/p/p412444)

Der 2D Lidarsensor überwacht die Region vor und neben dem Shared Guide Dog. Er wird für die Hinderniserkennung und Lokalisierung eingesetzt. Für den Lidar-Sensor wird das offizielle sick_scan2 package verwendet.

## Ultra-Wideband (UWB)

Sensormodell: `Decawave DW1000`

Das UWB-System liefert, ähnlich eines GNSS Systems, die Position eines Tags in Relation zu mehreren Anchors. Die Ausgabe der Position erfolgt standardmäßig im globalen WGS84 Koordinatensystem. Zusätzlich kann die Ausgabe in lokalen Koordinaten eingeschaltet werden.

| Parameter | Standardwert | Beschreibung |
| --------- | ------------ | ------------ |
| 

| Topic | Message Type |
| ----- | ------------ |
| /uwb  | sensor_msgs/NavSatFix |
| /uwb_local | geometry_msgs/PoseWithCovarianceStamped |

## Motortreiber


