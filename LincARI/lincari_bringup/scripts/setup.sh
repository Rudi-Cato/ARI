#!/usr/bin/env sh
export TMULE=1

this_file=$(readlink -f "$BASH_SOURCE")
this_dir=$(dirname $this_file)
ws_root=$(readlink -f "$this_dir/../../../")
export DISPLAY=:0
export ROBOT_NAME=ARI
export ARI_WS="$ws_root"
export MONGO_DIR="$ws_root/db"
export ROBOT_IP="10.68.0.1"
export ROS_MASTER_URI="http://$ROBOT_IP:11311"


# if [ -r "$HOME/.lindseyrc" ]; then source ""$HOME/.lindseyrc""; fi

# if [ -r "$LINDSEY_WS" ]; then echo "YES"; source "$LINDSEY_WS/devel/setup.bash"; else source /opt/ros/kinetic/setup.bash; fi

echo $ARI_WS
