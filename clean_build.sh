#!/bin/bash
echo "The script you are running has basename `basename "$0"`, dirname `dirname "$0"`"

VAR1=`pwd`

cd ~/dev_ws/build
rm -r *

cd ~/dev_ws/install
rm -r *

echo "All install and build files deleted"

cd ~/dev_ws
colcon build --symlink-install

cd $VAR1
