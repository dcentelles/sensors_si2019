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
sleep 2s

declare -A protonames=( ["sfama"]="Slotted-FAMA" ["aloha"]="CS-ALOHA" ["dcmac"]="UMCI-MAC" )

for proto in $protos
do
	protodir=$rbasedir/${protonames[$proto]}
	mkdir -p $protodir
	echo "Current protocol: $protodir"
	sleep 20s
	sitldir=/home/diego/programming/ardupilot/ArduSub/
	cd $sitldir
	coproc sitl { ../Tools/autotest/sim_vehicle.py -m "--out=udp:127.0.0.1:14550" -L irslab --console; }
	sitlpid=${sitl_PID}
	echo $sitlpid
	cd -
	./mac_performance.sh $proto $protodir $bslots nodebug

	sleep 10s
	echo "kill sim_vehicle.py"
	kill -9 $sitlpid
	echo "kill mavproxy"
	kill -9 $(ps aux | grep "mavproxy" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1
	echo "kill ardusub sitl"
	kill -9 $(ps aux | grep "ardusub" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1
	sleep 10s
done

./plot.sh $rbasedir
