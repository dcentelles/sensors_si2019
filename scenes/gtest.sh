#!/bin/bash

#./gtest.sh
#	 test0
#	 "aloha sfama dcmac"
#	 <SFAMABackoffSlots>

scriptName=$(basename $0)
echo $scriptName
pid=$$

rbasedir=$1
protos=$2
bslots=$3

if [ ! -d $rbasedir ]
then
	echo "creating $rbasedir"
	mkdir -p $rbasedir
fi
basedir=$(readlink -f $rbasedir)
launchdir=$(pwd)

stopntp="timedatectl set-ntp 0"
echo $stopntp
eval $stopntp
sleep 1s

declare -A protonames=( ["sfama"]="Slotted-FAMA" ["aloha"]="CS-ALOHA" ["dcmac"]="UMCI-MAC" )

for proto in $protos
do
	protodir=$rbasedir/${protonames[$proto]}
	mkdir -p $protodir
	echo "Current protocol: $protodir"
	sleep 1s
	echo "Launch SITL simulator..."
	cp ../config/mav.parm ../modules/ardupilot/Tools/autotest/default_params/sub.parm;
	coproc sitl { ../modules/ardupilot/Tools/autotest/sim_vehicle.py -m "--out=udp:127.0.0.1:14550" -L irslab --console -v ArduSub; }
	sitlpid=${sitl_PID}
	echo $sitlpid
	sleep 1s
	echo "Launch aruco_mapping..."
	{ coproc aruco_mapping { roslaunch aruco_mapping aruco_mapping.launch  num_of_markers:=40 image_transport:=compressed camera_info:=/bluerov2/camera_info image_topic:=/bluerov2/camera marker_size:=0.088; } >&3; } 3>&1
	aruco_mappingpid=${aruco_mapping_PID}
	echo $aruco_mappingpid
	sleep 1s
	echo "Launch vision filter..."
	{ coproc vision_filter { rosrun filters_ros tf_filter; } >&3; } 3>&1
	vision_filterpid=${vision_filter_PID}
	echo $vision_filterpid
	sleep 1s
	./mac_performance.sh $proto $protodir $bslots nodebug

	sleep 1s
	echo "kill sim_vehicle.py"
	kill -9 $sitlpid
	echo "kill mavproxy"
	kill -9 $(ps aux | grep "mavproxy" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1
	echo "kill ardusub sitl"
	kill -9 $(ps aux | grep "ardusub" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1
	echo "kill vision filter"
	kill -9 $vision_filterpid
	kill -9 $(ps aux | grep "tfToEkf" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1
	echo "kill aruco_mapping"
	kill -9 $aruco_mappingpid
	kill -9 $(ps aux | grep "aruco_mapping" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1

	sleep 5s
done

./plot.sh $rbasedir
