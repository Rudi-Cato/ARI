
TIAGO_NUM=35
#echo $TIAGO_NUM

# Change ROS Master
echo "Connecting through Ethernet"
export ROS_MASTER=ari-${TIAGO_NUM}c
export ROS_MASTER_URI=http://${ROS_MASTER}:11311


# wait until the master is pingable
echo "Wait until ros master is reachable..."
ping_cancelled=false    # Keep track of whether the loop was cancelled, or succeeded
until ping -c1 ${ROS_MASTER} >/dev/null 2>&1; do :; done &
trap "kill $!; ping_cancelled=true" SIGINT
wait $!          # Wait for the loop to exit, one way or another
trap - SIGINT    # Remove the trap, now we're done with it
echo "Done pinging ros master."

# check the ros master is running
# rostopic list &>/dev/null
# RETVAL=$?
# if [ $RETVAL -ne 0 ]; then
#     echo "[ERROR] connection with ROS MASTER not enstablished"
# else
#     echo "[OK] Connected to ROS MASTER"
# fi

# Add forwarding address to DNS
#echo $ROBOT_IFACE
#THIS_IP=`ifconfig ${ROBOT_IFACE} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
ROBOT_IP=`getent hosts ${ROS_MASTER} | awk '{ print $1 }'`
THIS_IP=`ip route get ${ROBOT_IP} | grep "src" | sed 's/.*src \([0-9\.]*\).*/\1/'`
export ROS_HOSTNAME=${HOSTNAME}
export ROS_IP=${THIS_IP}
# add to bashrc
echo "export ROS_MASTER_URI=${ROS_MASTER_URI}" >> ~/.bashrc
echo "export ROS_HOSTNAME=${ROS_HOSTNAME}" >> ~/.bashrc


# echo "${ROS_MASTER} ari-${TIAGO_NUM}c" >> /etc/hosts
echo "executing "palroot" ssh root@${ROS_MASTER} "addLocalDns -u \"${ROS_HOSTNAME}\" -i \"${ROS_IP}\"""
sshpass -p "palroot" ssh root@${ROS_MASTER} "addLocalDns -u \"${ROS_HOSTNAME}\" -i \"${ROS_IP}\""
