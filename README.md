# SGD4.0-ROS2

Im Rahmen des Forschungsvorhabens "Blindenhund 4.0" wird an der HAW Hamburg in interdisziplinärer Zusammenarbeit ein Blindenhund zur Unterstützung der menschlichen Navigation im urbanen Nahfeld entwickelt. Basierend auf Fähigkeiten Fahrerloser Transportsysteme (z.B. Laser, Ultraschall, 3D-Kamera) hilft er bei Einschränkungen in der natürlichen menschlichen Wahrnehmung ebenso wie bei körperlichen Einschränkungen. Besonders ältere und blinde Menschen profitieren durch erhöhte Selbständigkeit und Mobilität.

# ROS2 Installation

Installationsanleitung für die Verwendung von ROS2 Projekt *Shared Guide Dog 4.0*.

## Requirements

Anforderung | Version
----------- | --------------
Betriebssystem | Ubuntu 20.04 LTS 64 bit

## Verwendete Pakete / Software

Paket | Link | required
------|------|---------
ROS2 Foxy | - | required
Navigation 2 | - | required
Gazebo 11 | [Gazebo](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) | optional
sick_scan2 | [sick_scan2](https://github.com/SICKAG/sick_scan2) | required

## Installation ROS2 Foxy

Installation wie [hier](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/) beschrieben.

Um ROS bei jedem neuen Terminalfenster automatisch zu sourcen, kann an das Ende der .bashrc folgender Codeblock eingefügt werden.

```sh
if [ -f /opt/ros/foxy/setup.bash ]; then
  . /opt/ros/foxy/setup.bash
fi
```

## Installation ROS2 for Blindenhund

Für die Entwicklung des Blindenhunds wird ein neuer Workspace angelegt.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

Anschließend können die Dateien von GitHub geklont und gebaut werden.

```
git clone https://github.com/PStahr-HAW/ros2-sgd4.0.git

colcon build --symlink-install
cd ~/dev_ws/src/ros2_sgd4.0/sgd_gazebo_sim/build
cmake ..
make

source ~/dev_ws/install/setup.bash
```

## Installation Navigation 2

Es gibt zwei Möglichkeiten Navigation 2 zu installieren. Entweder in einem separaten Workspace oder im dev_ws (angelegt im vorigen Kapitel).

Installation wie [hier](https://navigation.ros.org/build_instructions/index.html#build-nav2-for-released-distribution)
beschrieben.

Oder mit folgenden Commands:

```
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel
cd ~/nav2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
colcon build --symlink-install
```

## Installation Gazebo 11
Gazebo wird für die Arbeit mit dem Blindenhund nicht benötigt, ist jedoch sinnvoll, um neue Funktionen zuerst simulieren zu können und sie erst bei erfolgreicher Simulation auf dem Blindenhund zu testen.

Installation wie [hier](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) beschrieben.

Dann noch benötigte Packages zur Kommunikation mit ROS2. Es ist sinnvoll den neuen Workspace nicht *ws* zu nennen, sondern *gazebo_ws*.
[install gazebo ros2](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

Die Testing Section sollte durchgeführt werden, um sicher zu stellen, dass alle Packages installiert sind. Beim Testen sollte sowohl der Pfad aus der Anleitung wie auch der Pfad zum Workspace funktionieren.

```
Gazebo --verbose  ~/gazebo_ws/src/gazebo_ros_pkgs/gazebo_plugins/worlds/gazebo_diff_drive_demo.world
```

Die bereitgestellte Simulationsumgebung enthält auch das Modell des Blindenhunds. Damit es von Gazebo gefunden werden kann, muss die Variable $GAZEBO_MODEL_PATH angepasst werden. 

```
export GAZEBO_MODEL_PATH=~/dev_ws/src/ros2-sgd4.0/sgd_gazebo_sim/models:$GAZEBO_MODEL_PATH
```

Um den Pfad automatisch bei jedem neuen Terminalfenster zu setzen, kann der Befehl auch an das Ende der .bashrc gesetzt werden.

## Installation sick_scan_base
Installation der packages wie [hier](https://github.com/SICKAG/sick_scan2) beschrieben. ldmrs Support ist nicht erforderlich.

Nach der Installation muss die Datei config/sick_tim5xx.yaml mit den folgenden Werten angepasst werden.

```
hostname : "141.22.33.150"
frame_id : "laser"
scanner_name : "sick_tim_5xx"
port : 2112
min_ang : -2.35619449
max_ang : 2.35619449
```

Im launch file launch/sick_tim_5xx.launch.py muss sichergestellt werden, dass node_executable auskommentiert ist, executable nicht. 

```
# node_executable='sick_generic_caller',
executable='sick_generic_caller',
```

In den Ubuntu Einstellungen muss nun noch eine neue Kabelgebundene Netzwerkverbindung erstellt und eingeschaltet werden.

![ip config 1](/doc/ip_config1.png)

![ip config 2](/doc/ip_config2.png)


# Useful git commands

**git clone**
Usage: `git clone [url]`

Obtain a new repository from an existing URL

**git add**
Usage: `git add [file]`

Add a new file to the staging area. Use `git add *` to add all files to the staging area.

**git commit**
Usage: `git commit -m "[commit message]"`

Record or snapshot staged files permanently in the version history.

**git status**
Usage: `git status`

List all files that have to be commited.


Info: Git commands are taken from https://dzone.com/articles/top-20-git-commands-with-examples
