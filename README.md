# SGD4.0-ROS2

Im Rahmen des Forschungsvorhabens "Blindenhund 4.0" wird an der HAW Hamburg in interdisziplinärer Zusammenarbeit ein Blindenhund zur Unterstützung der menschlichen Navigation im urbanen Nahfeld entwickelt. Basierend auf Fähigkeiten Fahrerloser Transportsysteme (z.B. Laser, Ultraschall, 3D-Kamera) hilft er bei Einschränkungen in der natürlichen menschlichen Wahrnehmung ebenso wie bei körperlichen Einschränkungen. Besonders ältere und blinde Menschen profitieren durch erhöhte Selbständigkeit und Mobilität.

# ROS2 Startup

Zum Starten von ROS2 für den Shared Guide Dog 4.0 wird ein Launch-Script verwendet. Dieses befindet sich im Package *sgd_bringup*. Daneben sind weitere Dateien in diesem Package zu finden. Unter anderem die Konfigurationsdateien, Maps, die URDF Beschreibung des Roboters und das Simulationsmodell der Umgebung. Der Shared Guide Dog kann über den Befehl

```
ros2 launch sgd_bringup sgd_startup.launch.py sim:=[True|False] slam:=[True|False]
```

gestartet werden. Die beiden Parameter *sim* und *slam* sind optional. Standardmäßig ist `sim:=False` und `slam:=False`.

# ROS2 Installation

Installationsanleitung für die Verwendung von ROS2 im Projekt *Shared Guide Dog 4.0*.

## Requirements

Anforderung | Version
----------- | --------------
Betriebssystem | Ubuntu 20.04 LTS 64 bit

## Verwendete Pakete / Software

Paket | Link | required
------|------|---------
ROS2 Foxy | - | required
Navigation 2 | - | required
sick_scan2 | [sick_scan2](https://github.com/SICKAG/sick_scan2) | required
robot_localization | [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/ros2) | required
Gazebo 11 | [Gazebo](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) | optional

## Installation ROS2 Foxy

Installation wie [hier](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/) beschrieben.

Um ROS bei jedem neuen Terminalfenster automatisch zu sourcen, kann an das Ende der .bashrc folgender Codeblock eingefügt werden.

```sh
if [ -f /opt/ros/foxy/local_setup.bash ]; then
  . /opt/ros/foxy/local_setup.bash
fi
```

## Installation Navigation 2

Es gibt zwei Möglichkeiten Navigation 2 zu installieren. Entweder in einem separaten Workspace oder im dev_ws (wird im nächsten Kapitel wieder verwendet).

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

source ~/nav2_ws/install/local_setup.bash
```

Um Navigation 2 bei jedem Terminalstart zu sourcen, den folgenden Codeblock ans Ende der .bashrc einfügen.

```sh
if [ -f ~/nav2_ws/install/local_setup.bash ]; then
  source ~/nav2_ws/install/local_setup.bash
fi
```

## Installation ROS2 for Blindenhund

Für die Entwicklung des Blindenhunds wird ein neuer Workspace angelegt.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

Anschließend können die Dateien von GitHub geklont und gebaut werden. Beim Bauen muss unbedingt darauf geachtet werden, dass nur aus dem `~/dev_ws` Verzeichnis gebaut wird.

```
cd ~/dev_ws
git clone https://github.com/PStahr-HAW/ros2-sgd4.0.git
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy

colcon build --symlink-install
```

Wenn der build-Befehl scheitert, kann der folgende Workaround versucht werden:

```
colcon build --symlink-install --packages-select sgd_msgs
colcon build --symlink-install --packages-select sgd_util
colcon build --symlink-install
```

Anschließend muss noch das Geazebo_Sim Package gebaut werden.

```
cd ~/dev_ws/src/ros2_sgd4.0/sgd_gazebo_sim/build
cmake ..
make

source ~/dev_ws/install/local_setup.bash
```

Um USB Ports ohne root Rechte nutzen zu können, müssen die folgenden Befehle ausgeführt werden.
```
sudo apt remove modemmanager
sudo adduser $USER dialout
```

Um ROS2 für den Blindenhund bei jedem Terminalstart zu sourcen, den folgenden Codeblock ans Ende der .bashrc einfügen.

```sh
if [ -f ~/dev_ws/install/local_setup.bash ]; then
  source ~/dev_ws/install/local_setup.bash
fi
```

## Installation sick_scan2
Installation der packages wie [hier](https://github.com/SICKAG/sick_scan2) beschrieben. ldmrs Support ist nicht erforderlich.

Nach der Installation muss die Datei src/sick_scan2/config/sick_tim_5xx.yaml mit den folgenden Werten angepasst werden.

```
hostname : "141.22.33.150"
frame_id : "scan"
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

Nach den beiden Anpassungen muss das Projekt neu gebaut werden. Zuletzt wird in den Ubuntu Einstellungen noch eine neue Kabelgebundene Netzwerkverbindung erstellt und eingeschaltet.

![ip config 1](/doc/ip_config1.png)

![ip config 2](/doc/ip_config2.png)

## Installation Robot Localization Package

Bevor das Robot Localization Package heruntergeladen und installiert werden kann, muss GeographicLib [heruntergeladen](https://geographiclib.sourceforge.io/html/index.html) und [installiert](https://geographiclib.sourceforge.io/html/install.html) werden.

Anschließend wird ein neuer Workspace `robot_localization_ws` im *home* Ordner erstellt. In diesen Order werden das robot_localization package und alle dependencies geklont.

Paket | Link
------|-----
Robot Localization | [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)
Geographic Info | [geographic_info](https://github.com/ros-geographic-info/geographic_info/tree/ros2)
Diagnostics | [Diagnostics](https://github.com/ros/diagnostics/tree/foxy)
GeographicLib | [GeographicLib](https://geographiclib.sourceforge.io/html/index.html)

```
mkdir -p ~/robot_localization_ws/src
cd ~/robot_localization_ws/src
git clone --branch ros2 https://github.com/ros-geographic-info/geographic_info.git
git clone --branch foxy https://github.com/ros/diagnostics.git
cd ..
colcon build --symlink-install

cd src
git clone --branch foxy-devel https://github.com/cra-ros-pkg/robot_localization.git
```

Damit das Robot Localization Package fehlerfrei gebaut werden kann, muss das *STATIC* in Zeile 37 in robot_localization/CMakeLists.txt gelöscht werden, sodass die Zeile dann wie folgt aussieht:

```
find_package(GeographicLib REQUIRED COMPONENTS)
```

Anschließend speichern der Datei und bauen des Packages.

```
cd ~/robot_localization_ws
colcon build --symlink-install --packages-select robot_localization
```


Zum Schluss noch das Einfügen in die .bashrc.

```sh
if [ -f ~/robot_localization_ws/install/setup.bash ]; then
  ~/robot_localization_ws/install/setup.bash
fi
```

## Installation Gazebo 11
Gazebo wird für die Arbeit mit dem Blindenhund nicht benötigt, ist jedoch sinnvoll, um neue Funktionen zuerst simulieren zu können und sie erst bei erfolgreicher Simulation auf dem Blindenhund zu testen. Bei der Installation von ROS2 sollte Gazebo 11 installiert worden sein, das kann mit dem Befehl `gazebo` in einem Terminal geprüft werden.

Falls sich Gazebo nicht öffnet, muss es manuell installiert werden wie [hier](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) beschrieben und zusätzlich werden weitere Packages benötigt. Es ist sinnvoll den neuen Workspace nicht *ws* zu nennen, sondern *gazebo_ws*.
[install gazebo ros2](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

Die bereitgestellte Simulationsumgebung enthält auch das Modell des Blindenhunds. Damit es von Gazebo gefunden werden kann, muss die Variable $GAZEBO_MODEL_PATH angepasst werden. Um den Pfad automatisch bei jedem neuen Terminalfenster zu setzen, kann der Befehl auch an das Ende der .bashrc gesetzt werden.

```
export GAZEBO_MODEL_PATH=~/dev_ws/src/ros2-sgd4.0/sgd_gazebo_sim/models:$GAZEBO_MODEL_PATH
```

Ein Test der Simulationsumgebung kann mit dem folgenden Befehl durchgeführt werden. Es dürfen keine Fehler auftreten und die Welt muss vollständig geladen werden.

```
gazebo --verbose ~/dev_ws/src/ros2-sgd4.0/sgd_bringup/worlds/model_static.model
```

![Gazebo example](/doc/gazebo_example.png)


# Testing im Shared Guide Dog Projekt

- Trennung von ROS Code und eigenem Code. Das bedeutet, dass in einer ROS Node keine Berechnungen durchgeführt werden, sondern diese in externe Files ausgelagert werden.
- Durch die Trennung können die Algorithmen mit GTest einfach getestet werden.

# Use COLCON inside Visual Studio Code

Install the package [Colcon Tasks](https://marketplace.visualstudio.com/items?itemName=deitry.colcon-helper) via Extensions Tab (Ctrl+Shift+X) and follow the instructions in the *How to use* section.

In settings tab (Ctrl+,) search for *colcon* and change `Colcon: Colcon Cwd` to `~/home/dev_ws` and `Colcon: Install Type` to `symlinked`.

The last step is to change the command in `tasks.json` to `"colcon"`.

# Useful git commands

**git clone**
Usage: `git clone [url]`
Alternative: `git clone --branch [branch] [url]`

Obtain a new repository from an existing URL

**git checkout**
Usage: `git checkout [branch]`

Checkout another branch from current repository.

**git add**
Usage: `git add [file]`

Add a new file to the staging area. Use `git add *` to add all files to the staging area.

**git commit**
Usage: `git commit -m "[commit message]"`

Record or snapshot staged files permanently in the version history.

**git push**
Usage: `git push`
Alternative: `git push <origin> <branch>`

Send committed changes to repository.

**git status**
Usage: `git status`

List all files that have to be commited.


Info: Git commands are taken from https://dzone.com/articles/top-20-git-commands-with-examples
