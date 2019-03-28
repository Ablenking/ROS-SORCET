#edit by Zemin Lin 2019/03/18
#! /bin/sh
# Please run with sudo.
# This bash will launch all the components of the system, in the order of:
#sudo -s
#sudo rm -rf /root/.ros/log
#su uav

user_name=uav

cd /home/uav/catkin_ws
#cd ~/catkin_ws
rosclean purge -y

source /opt/ros/kinetic/setup.bash
source /catkin_ws/devel/setup.bash

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/uav/catkin_ws/
#export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOME=/home/uav/catkin_ws/logfiles
#export ROS_HOSTNAME=UAV-10
#export ROS_MASTER_URI=http://192.168.1.100:11311

# kill all screen session 
pkill screen
# delay between launching various modules
module_delay=1

# check whether is running as sudo
if [ "$EUID" -eq 0 ]
    then echo "Please DO NOT run as root."
    exit
fi

if [ "${STY}" != "" ]
    then echo "You are running the script in a screen environment. Please quit the screen."
    exit
fi

output="$(screen -ls)"
if [ [ $output != *"No Sockets found"* ] ]; then
    echo "There are some screen sessions alive. Please run 'pkill screen' before launching uavos."
    exit
fi


# enable serial ports
sudo chmod a+rw /dev/ttyUSB0
#sudo chmod a+rw /dev/ttyUSB1
#sudo chmod a+rw /dev/ttyACM0

echo "The system is booting..."

##roscore
screen -d -m -S roscore bash -c "source ~/catkin_ws/devel/setup.bash; roscore; exec bash -i"
sleep 10
echo "roscore ready."


# --------------------------------------------
# prepareation

#source devel/setup.bash
# 0. px4
screen -d -m -S mavros bash -c "source ~/catkin_ws/devel/setup.bash; roslaunch mavros px4.launch; exec bash -i"
sleep ${module_delay}
echo "mavros ready."

# 1. offboard demo
screen -d -m -S mavros bash -c "source ~/catkin_ws/devel/setup.bash; rosrun offboard_demo offboard_demo_node; exec bash -i"
sleep ${module_delay}
echo "offboard demo ready."

#source devel/setup.bash
# 1. uav_state
#screen -d -m -S uav_state bash -c "source ~/catkin_ws/install/setup.bash; roslaunch uav_state uav_state.launch; exec bash -i"
#sleep ${module_delay}
#echo "uav_state ready."

#source devel/setup.bash
# 2. ref_generation
#screen -d -m -S ref_generation bash -c "source ~/catkin_ws/install/setup.bash; roslaunch ref_generation ref_generation.launch; exec bash -i"
#sleep ${module_delay}
#echo "ref_generation ready."


echo "System is started."
