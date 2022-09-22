![Ubuntu version](https://img.shields.io/badge/Ubuntu-20.04-orange)
![ROS version](https://img.shields.io/badge/ROS2-Foxy-brightgreen)

# SGD4.0-ROS2

<p align="center">
  <img height="300" src="doc/sgd_logo_w_background.png" />
</p>



Im Rahmen des Forschungsvorhabens "Blindenhund 4.0" wird an der HAW Hamburg in interdisziplinärer Zusammenarbeit ein Blindenhund zur Unterstützung der menschlichen Navigation im urbanen Nahfeld entwickelt. Basierend auf Fähigkeiten Fahrerloser Transportsysteme (z.B. Laser, Ultraschall, 3D-Kamera) hilft er bei Einschränkungen in der natürlichen menschlichen Wahrnehmung ebenso wie bei körperlichen Einschränkungen. Besonders ältere und blinde Menschen profitieren durch erhöhte Selbständigkeit und Mobilität.

# Contents

- [Startup](README.md#ros2-startup)
- [preempt_rt Patch](README.md#preempt_rt-patch)
- [Installation](README.md#ros2-installation)
- [Testing](README.md#testing-im-shared-guide-dog-projekt)
- [Debugging](README.md#debugging)
- [Git commands](README.md#useful-git-commands)

# ROS2 Startup

Zum Starten von ROS2 für den Shared Guide Dog 4.0 wird ein Launch-Script verwendet. Dieses befindet sich im Package *sgd_bringup*. Daneben sind weitere Dateien in diesem Package zu finden. Unter anderem die Konfigurationsdateien, Maps, die URDF Beschreibung des Roboters und das Simulationsmodell der Umgebung. Der Shared Guide Dog kann über den Befehl

```
ros2 launch sgd_bringup sgd_startup.launch.py sim:=[True|False] slam:=[True|False] log_severity:=["E"|"W"|"I"|"D"|"V"]
```

gestartet werden. Die beiden Parameter *sim* und *slam* sind optional. Standardmäßig ist `sim:=False` und `slam:=False`. Die log_severity ist auf `I` eingestellt.

# preempt_rt Patch

## useful links
- [Ubuntu Kernel/Build](https://wiki.ubuntu.com/Kernel/BuildYourOwnKernel)
- [HOWTO setup Linux with PREEMPT_RT properly](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup)
- [Building a real-time Linux kernel](https://docs.ros.org/en/foxy/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html)

## Installation

Install required software
```
sudo apt-get build-dep linux linux-image-$(uname -r)
sudo apt-get install libncurses-dev gawk flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf llvm zstd
```

Create a new directory
```
mkdir ~/kernel
cd ~/kernel
```

then download the linux kernel (v5.15.69) and preempt rt patch for the downloaded version. 

```
wget https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-5.15.69.tar.xz
wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.65-rt49.patch.xz
```
Caution: Do not download the file patches-5.15-...!

After the downloading is completed unpack linux kernel
```
xz -cd linux-5.15.69.tar.xz | tar xvf -
```

and patch the kernel.
```
cd linux-5.19.10/
xzcat ../patch-5.15.65-rt49.patch.xz | patch -p1
```

## Configuration

Copy the config of the ubuntu install
```
cp /boot/config-5.15.0-48-generic .config
```

Open Software & Updates. in the Ubuntu Software menu tick the ‘Source code’ box

Enable all Ubuntu configurations and preempt_rt in the kernel
```
yes '' | make oldconfig
make menuconfig
```

set the following
```
# Enable CONFIG_PREEMPT_RT
 -> General Setup
  -> Preemption Model (Fully Preemptible Kernel (Real-Time))
   (X) Fully Preemptible Kernel (Real-Time)

# Enable CONFIG_HIGH_RES_TIMERS
 -> General setup
  -> Timers subsystem
   [*] High Resolution Timer Support

# Enable CONFIG_NO_HZ_FULL
 -> General setup
  -> Timers subsystem
   -> Timer tick handling (Full dynticks system (tickless))
    (X) Full dynticks system (tickless)

# Set CONFIG_HZ_1000 (note: this is no longer in the General Setup menu, go back twice)
 -> Processor type and features
  -> Timer frequency (1000 HZ)
   (X) 1000 HZ

# Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE [=y]
 ->  Power management and ACPI options
  -> CPU Frequency scaling
   -> Default CPUFreq governor (<choice> [=y])
     (X) performance
```

Execute the following commands
```
chmod a+x debian/rules
chmod a+x debian/scripts/*
chmod a+x debian/scripts/misc/*
LANG=C fakeroot debian/rules clean
LANG=C fakeroot debian/rules editconfigs
```

## Build the kernel

Build kernel, this may take some time (~3h)
```
LANG=C sudo fakeroot make -j `nproc` deb-pkg
```

After the build is finished check the debian packages

```
ls ../*deb
../linux-headers-5.4.78-rt41_5.4.78-rt44-1_amd64.deb  ../linux-image-5.4.78-rt44-dbg_5.4.78-rt44-1_amd64.deb
../linux-image-5.4.78-rt41_5.4.78-rt44-1_amd64.deb    ../linux-libc-dev_5.4.78-rt44-1_amd64.deb
```

Then we install all kernel debian packages

```
sudo dpkg -i ../*.deb
```

Now the real time kernel should be installed. Reboot the system and check the new kernel version.

```
sudo reboot
uname -a
Linux ros2host 5.4.78-rt44 #1 SMP PREEMPT_RT Fri Nov 6 10:37:59 CET 2020 x86_64 xx
```

If the loading of the kernel terminates with the follwing error message
```
Loading Linux 5.15.69-rt49...
error: /boot/vmlinux-5.15.69-rt49 has invalid signature.
Loading initial ramdisk...
error: you need to load the kernel first.
```
check if Secure Boot is turned off.

## Possible errors when building

```
make[2]: *** [debian/rules:7: build-arch] Error 2
dpkg-buildpackage: error: debian/rules binary subprocess returned exit status 2
make[1]: *** [scripts/Makefile.package:77: deb-pkg] Error 2
make: *** [Makefile:1533: deb-pkg] Error 2
```

Change in file .config ([gitlab.com](https://gitlab.com/CalcProgrammer1/OpenRGB/-/issues/950)):
```
CONFIG_SYSTEM_TRUSTED_KEYS = ""
```

```
make[1]: *** No rule to make target 'debian/canonical-revoked-certs.pem' , needed by certs/x509_revocation_list'. STOP.
make: ***[Makefile:1851:certs] Error 2
```

Execute the command ([askubuntu.com](https://askubuntu.com/questions/1362455/i-am-installing-kernel-in-my-ubuntu-but-getting-an-error))
```
scripts/config --disable SYSTEM_REVOCATION_KEYS
```

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

## 3rd party libraries

- [GoogleTest](https://github.com/google/googletest)
- [json](https://github.com/nlohmann/json)
- [Plog](https://github.com/SergiusTheBest/plog)
- [ROS Wrapper for Intel RealSense](https://github.com/IntelRealSense/realsense-ros)

## Installation ROS2 Foxy

Installation wie [hier](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/) beschrieben.

Um ROS bei jedem neuen Terminalfenster automatisch zu sourcen, kann an das Ende der .bashrc folgender Codeblock eingefügt werden.

```sh
. ~/ros2_foxy/install/local_setup.bash
. /opt/ros/foxy/local_setup.bash  # for ros dependencies
```

## Installation Navigation 2

Installation wie [hier](https://navigation.ros.org/build_instructions/index.html#build-nav2-for-released-distribution)
beschrieben.

Um Navigation 2 bei jedem Terminalstart zu sourcen, den folgenden Codeblock ans Ende der .bashrc einfügen.

```sh
. ~/nav2_ws/install/local_setup.bash
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

Bevor das Robot Localization Package heruntergeladen und installiert werden kann, muss [GeographicLib](https://geographiclib.sourceforge.io/html/index.html) heruntergeladen und [installiert](https://geographiclib.sourceforge.io/html/install.html) werden.


In diesen Order werden das robot_localization package und alle dependencies geklont.

Paket | Link
------|-----
Robot Localization | [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)
Geographic Info | [geographic_info](https://github.com/ros-geographic-info/geographic_info/tree/ros2)
Diagnostics | [Diagnostics](https://github.com/ros/diagnostics/tree/foxy)
GeographicLib | [GeographicLib](https://geographiclib.sourceforge.io/html/index.html)

Installation von GeographicLib:

```bash
mkdir -p ~/localization_ws/src
cd ~/localization_ws/src
git clone https://github.com/geographiclib/geographiclib.git --branch release
touch COLCON_IGNORE     # tells colcon to ignore this folder
mkdir BUILD
cd BUILD
cmake ..
make
make test
sudo make install
```

Anschließend werden die weiteren Dependencies in das src Verzeichnis geklont.

```
cd ~/localization_ws/src
git clone --branch ros2 https://github.com/ros-geographic-info/geographic_info.git
git clone --branch foxy https://github.com/ros/diagnostics.git
cd ..
colcon build --symlink-install

source ~/localization_ws/install/local_setup.bash

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
. ~/robot_localization_ws/install/setup.bash
```

## Installation ROS2 for Blindenhund

### Installation der 3rd party dependencies

GoogleTest ist bereits in der ROS2 Installation enthalten. Nlohmann/json und SergiusTheBest/plog werden über Cmakes FetchContent Funktion eingebunden.

#### Intel RealSense Wrapper for ROS2

Installation Intel® RealSense™ SDK 2.0 [Original-Anleitung](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```
Optional können die Developer und Debug Packages installiert werden. 
```
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

Anschließend wird der ROS RealSense Wrapper installiert:
```
mkdir -p ~/3rd_party_ws/src
cd ~/3rd_party_ws/src/

git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
cd ~/3rd_party_ws

sudo apt-get install python3-rosdep -y
rosdep update
rosdep install -i --from-path src --rosdistro $_ros_dist --skip-keys=librealsense2 -y    # may print an error, but build should succeed

colcon build
```

To source on each new terminal add
```
. ~/3rd_party_ws/install/local_setup.bash
```

### Installation ros2-sgd4.0

Für die Entwicklung des Blindenhunds wird ein neuer Workspace angelegt.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

Anschließend können die Dateien von GitHub geklont und gebaut werden. Eine Anleitung wie ein ssh key zum Github Account hinzugefügt werden kann, gibt es [hier]([https://www.w3docs.com/snippets/git/how-to-generate-ssh-key-for-git.html](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)). Beim Bauen muss unbedingt darauf geachtet werden, dass nur aus dem `~/dev_ws` Verzeichnis gebaut wird.

If you are using ssh key for authorization type:
```
git clone git@github.com:PStahr-HAW/ros2-sgd4.0.git
```
otherwise type:
```
git clone https://github.com/PStahr-HAW/ros2-sgd4.0.git
```

Install all dependencies and build.
```
cd ~/dev_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy

colcon build --symlink-install
. ~/dev_ws/install/local_setup.bash
```

Um USB Ports ohne root Rechte nutzen zu können, müssen die folgenden Befehle ausgeführt werden.
```
sudo apt remove modemmanager
sudo adduser $USER dialout
```

Um ROS2 für den Blindenhund bei jedem Terminalstart zu sourcen, den folgenden Codeblock ans Ende der .bashrc einfügen.

```sh
. ~/dev_ws/install/local_setup.bash
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
gazebo --verbose ~/dev_ws/src/ros2-sgd4.0/sgd_bringup/worlds/lohmuehlenpark.model
```

![Gazebo example](/doc/gazebo_example.png)


# Testing im Shared Guide Dog Projekt

## Idee

Die ROS2 Packages werden so aufgebaut, dass eine Trennung zwischen ROS-spezifischem Code und anderem Code stattfindet. Somit kann ein Test der Algorithmen ohne eine Abhängigkeit von ROS2 durchgeführt werden.

![ROS2 Testing mit GTest](/doc/ros2_testing.png)

Als Beispielpaket kann *sgd_safety* verwendet werden.

## Anpassungen in der CMakeLists.txt

Damit ein Test mit colcon durchgeführt werden kann, muss die CMakeLists.txt angepasst werden. 

Laden des GTest Pakets:
```
# find dependencies
find_package(...)
...

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

... weiter mit der alten CMakeLists.txt
```

Erstellen der Tests mit den Abhängigkeiten:
```
### testing
enable_testing()

set(test_name test_name)
add_executable(${test_name} <path_to_test_cpp> <optional path_to_src_cpp>)
target_link_libraries(${test_name} gtest_main <optional included_libs>)

# if you have test data (e.g. a csv file with data) put it into test/data and uncomment the following line.
# The files will be copied to the build folder from where they can be accessed
# file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

include(GoogleTest)
gtest_discover_tests(${test_name})
```

## Start der Tests und Anzeige der Testergebnisse

Das Starten der Tests erfolgt mit
```
colcon test --packages-select package_name
```
die Angabe eines Pakets ist optional. Wird kein Paket angegeben, werden alle Pakete getestet.

Eine Kurzübersicht über die Testergebnisse kann mit 
```
colcon test-result
```
erzeugt werden. Ausführlichere Berichte über bestandene und Fehlerhafte Tests sind im build_Ordner des Paktes unter *Testing* zu finden.

# Use COLCON inside Visual Studio Code

Install the package [Colcon Tasks](https://marketplace.visualstudio.com/items?itemName=deitry.colcon-helper) via Extensions Tab (Ctrl+Shift+X) and follow the instructions in the *How to use* section.

In settings tab (Ctrl+,) search for *colcon* and change `Colcon: Colcon Cwd` to `~/home/dev_ws` and `Colcon: Install Type` to `symlinked`.

The last step is to change the command in `tasks.json` to `"colcon"`.

# Debugging

The following section is based on [this](https://navigation.ros.org/tutorials/docs/get_backtrace.html) tutorial.

## Preparations

Add the following line to the CMakeLists.txt of the node you want to debug.

```
add_compile_options(-g)
```

After that simply rebuild the package with `colcon build --packages-select <package-name>`. After debugging the node, the option should be removed again, as it causes performance losses.


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
