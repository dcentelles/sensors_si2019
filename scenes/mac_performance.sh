#!/bin/bash

scriptName=$(basename $0)
scriptPath=$(realpath $0)
echo $scriptName
pid=$$

protocol=$1
basedir=$(realpath ${2}/)
maxBackoffSlots=${3}
debug=$4

devDelay=2
maxRange=100

echo "protocol: $protocol"

echo "BASEDIR: $basedir"
bindir="../build/" #TODO: as argument
resultsdir=$basedir/results

rm -rf $resultsdir 
if [ "$debug" == "debug" ]
then
	echo "debug"
	debug=true
else
	echo "no debug"
	debug=false
fi

rm -rf /dev/mqueue/*
mkdir -p $resultsdir

kill -9 $(ps aux | grep "bash .*$scriptName" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1

###################################################
###################################################


###################################################
###################################################

now=$(date +%s)
daterefsecs=$now
dateref=$(date -R -d @$daterefsecs)
datereffile=$basedir/dateref
echo "dateref: $dateref"
echo $dateref > $datereffile
echo "$datereffile content: $(cat $datereffile)"

rosrunproc=$(cat rosrunpid 2> /dev/null)
sim=$(cat simpid 2> /dev/null)
kill -9 $rosrunproc > /dev/null 2> /dev/null
kill -9 $sim > /dev/null 2> /dev/null

#sleep 5s

scenesdir=$(rospack find sensors_si2019)/scenes
uwsimlog=$(realpath $basedir/uwsimnet.log)
uwsimlograw=$(realpath $basedir/uwsim.log.raw)

tmplscene=$scenesdir/hil.xml
echo "TMPL SCENE: $tmplscene"
if [ "$protocol" == "dcmac" ]
then
	tracingscript=UMCIMAC_HILNetSimTracing
	scene=$scenesdir/$protocol.xml
	cp $tmplscene $scene
	cd ../modules/umci
	gitrev=$(git rev-parse --short HEAD)
	cd -
	library=$(rospack find sensors_si2019)/../../devel/lib/libumci_${gitrev}.so
#	library=$(realpath ${library})
	library=$(echo "$library" | sed 's/\//\\\//g')
	echo $library
	pktbuilder="DcMacPacketBuilder"
	libpath="<libPath>$library<\/libPath>"
else
	tracingscript=HILNetSimTracing
	scene=$scenesdir/$protocol.xml
	pktbuilder="VariableLength2BPacketBuilder"
	sed "s/<name><\/name>/<name>$protocol<\/name>/g" $tmplscene > $scene
	libpath=""
fi

echannel=0
disablerf=1
buoy_addr=0
hil_ac_addr=1
hil_rf_addr=0
explorer1_addr=2
explorer2_addr=3
explorer3_addr=4

sed -i "s/buoy_addr/$buoy_addr/g" $scene
sed -i "s/hil_ac_addr/$hil_ac_addr/g" $scene
sed -i "s/hil_rf_addr/$hil_rf_addr/g" $scene
sed -i "s/explorer1_addr/$explorer1_addr/g" $scene
sed -i "s/explorer2_addr/$explorer2_addr/g" $scene
sed -i "s/explorer3_addr/$explorer3_addr/g" $scene


sed -i "s/disablerf/$disablerf/g" $scene
sed -i "s/tracingscript/$tracingscript/g" $scene
sed -i "s/echannel/$echannel/g" $scene
sed -i "s/<maxDistance><\/maxDistance>/<maxDistance>$maxRange<\/maxDistance>/g" $scene
sed -i "s/<maxBackoffSlots><\/maxBackoffSlots>/<maxBackoffSlots>$maxBackoffSlots<\/maxBackoffSlots>/g" $scene
sed -i "s/packetbuilder/${pktbuilder}/g" $scene
sed -i "s/builderlibpath/${libpath}/g" $scene
uwsimlogpath=$(echo "$uwsimlog" | sed 's/\//\\\//g')
sed -i "s/<logToFile><\/logToFile>/<logToFile>$uwsimlogpath<\/logToFile>/g" $scene


if [ "$protocol" == "dcmac" ]
then
	roslaunch sensors_si2019 hil.launch debug:=$debug scene:=$scene 2>&1 | tee $uwsimlograw & 
else
#	export NS_LOG="AquaSimMac=all|prefix_time:AquaSimSFama=all|prefix_time:AquaSimAloha=all|prefix_time"
       	roslaunch sensors_si2019 hil.launch debug:=$debug scene:=$scene 2>&1 | tee $uwsimlograw & 
fi


sleep 5s
rosrunproc=$!

sim=$(ps aux | grep "uwsim_binary" | awk -v mpid=$pid '
BEGIN{\
 	found=0; 
}
{\
	if(mpid != $2)
	{ 
		where = match($0, "grep")
		if(where == 0)
		{
			if(!found)
			{
				pid=$2;
				found=1;
			}
			else
			{
				if($2 < pid)
					pid=$2;
			}
		}
	}
}
END{\
	print pid
}'
)
echo "ROSRUN: $rosrunproc ; SIM: $sim"

echo $rosrunproc > rosrunpid
echo $sim > simpid
sleep 370s


echo "SIGTERM programs..."
kill -s TERM $rosrunproc > /dev/null 2> /dev/null
kill -s TERM $sim > /dev/null 2> /dev/null

sleep 5s

echo "SIGINT programs..."
kill -s INT $rosrunproc > /dev/null 2> /dev/null
kill -s INT $sim > /dev/null 2> /dev/null

sleep 5s

echo "kill -9 programs..."
kill -9 $rosrunproc > /dev/null 2> /dev/null
kill -9 $sim > /dev/null 2> /dev/null
kill -9 $(ps aux | grep "bash .*$scriptName" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1

sleep 5s

mv $uwsimlog $resultsdir
mv $uwsimlograw $resultsdir
mv $scene $basedir
mv $datereffile $resultsdir
cp $scriptPath $basedir
echo $* > $basedir/notes.txt

echo "Exit"

