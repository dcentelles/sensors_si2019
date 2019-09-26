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
rawlogdir=$basedir/rawlog

rm -rf $resultsdir $rawlogdir
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
mkdir -p $rawlogdir


kill -9 $(ps aux | grep "bash .*$scriptName" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1

###################################################
###################################################

awktime='
BEGIN{\
	"cat \""dateref"\" | date -u -f - +%s" | getline t0
}
{\
	where = match($0, "[0-9]+\\/[0-9]+\\/[0-9]+ [0-9]+:[0-9]+:[0-9]+\\.[0-9]+")
	if(where != 0)
	{
		time=substr($0, RSTART, RLENGTH)
        where = match($0, patt)
	    if(where != 0)
	    {
		    f = "date -u -d \""time"\" +%s" 
		    f | getline seconds
		    close(f)
		    f = "date -u -d \""time"\" +%N"
		    f | getline nanos
		    close(f)
		    seconds = (seconds - t0)
		    tt = seconds + nanos / 1e9
		    f = "date -u -d \""time"\" +%s.%9N"
		    f | getline tt2
		    close(f)
		    printf("%.9f\t%d\t%d\n", tt, $7, $9)
        }
	} 
}
END{\
}
'

awkgap='
BEGIN{\
	samples = 0
	sum = 0
	sum2 = 0
}
{\
	cont += 1
	dif = $2 - old2
	if(dif==0) 
		dif = 1
	if(dif > 0) 
	{ 
		if(cont > 1) 
		{
			gap = ($1 - old1) / dif
			printf("%d\t%.9f\n", $2, gap)
			sum += gap
			sum2 += gap * gap
			samples += 1
		} 
		old1 = $1; old2 = $2
	}
}
END{\
	avg = sum / samples
	var = (sum2 - (avg * sum)) / (samples)
	sd = sqrt(var)
	printf("%.9f\t%.9f\t%.9f\n", avg, sd, var);
}'

awkjitter='
BEGIN{\
	samples = 0
	sum = 0
	sum2 = 0
}
{\
	if (samples > 0)
	{
		gap = ($2 - old2)
		gap = gap < 0 ? -gap : gap
		printf("%d\t%.9f\n", $1, gap)
		sum += gap
		sum2 += gap * gap
	}
	samples += 1
	old2 = $2
}
END{\
	avg = sum / samples
	var = (sum2 - (avg * sum)) / (samples)
	sd = sqrt(var)
	printf("%.9f\t%.9f\t%.9f\n", avg, sd, var);
}'

awkthroughput='
BEGIN{\
	nbytes = 0
}
{\
	nbytes += $3
}
END{\
	printf("%d", nbytes)
}'

awkavg='
BEGIN{\
	samples = 0
	sum = 0
	sum2 = 0
}
{\
	value = $col
	sum += value
	sum2 += value * value
	samples += 1
}
END{\
	avg = sum / samples
	var = (sum2 - (avg * sum)) / (samples)
	sd = sqrt(var)
	printf("%.9f\t%.9f\t%.9f\n", avg, sd, var);
}'

txRaw='
BEGIN{\
    nbytes = 0
    lines = 0
}
{\
    where = match($0, "MAC TX -- .*"devname".* Size: (.*)$", arr)
    if(where != 0)
    {
        nbytes += arr[1]
        lines += 1;
    }
}
END{\
    printf("totalTxBytes=%d\n", nbytes);
    printf("totalTxPackets=%d\n", lines)
}'


txRawDcMac='
BEGIN{\
    nbytes = 0
    lines = 0
}
{\

    where = match($0, "TX -- .*"devname".* Size: (.*)$", arr)
    if(where != 0)
    {
	nbytes += arr[1]
        lines += 1
    }
}
END{\
    printf("totalTxBytes=%d\n", nbytes);
    printf("totalTxPackets=%d\n", lines)
}'

colScript='
BEGIN{\
    nbytes = 0
    lines = 0
}
{\
    where = match($0, "COL -- .*"devname".* Size: (.*)$", arr)
    if(where != 0)
    {
	nbytes += arr[1]
        lines += 1
    }
}
END{\
    printf("totalColBytes=%d\n", nbytes);
    printf("totalColPackets=%d\n", lines)
}'



colScriptDcMac=$colScript

###################################################
###################################################

now=$(date -u +%s)
daterefsecs=$now
dateref=$(date -u -R -d @$daterefsecs)
datereffile=$basedir/dateref
echo "dateref: $dateref"
echo $dateref > $datereffile
echo "$datereffile content: $(cat $datereffile)"

rosrunproc=$(cat rosrunpid 2> /dev/null)
sim=$(cat simpid 2> /dev/null)
kill -9 $rosrunproc > /dev/null 2> /dev/null
kill -9 $sim > /dev/null 2> /dev/null

sleep 5s

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
sleep 10000s

sleep ${testduration}s

echo "SIGINT programs..."
kill -s INT $rosrunproc > /dev/null 2> /dev/null
kill -s INT $sim > /dev/null 2> /dev/null

sleep 10s

echo "SIGTERM programs..."
kill -s TERM $rosrunproc > /dev/null 2> /dev/null
kill -s TERM $sim > /dev/null 2> /dev/null

sleep 10s

echo "kill -9 programs..."
kill -9 $(ps aux | grep "bash .*$scriptName" | awk -v mpid=$pid '{ if(mpid != $2) print $2}') > /dev/null 2>&1
kill -9 $rosrunproc > /dev/null 2> /dev/null
kill -9 $sim > /dev/null 2> /dev/null

#mv $uwsimlog $resultsdir
#mv $uwsimlograw $resultsdir
#mv $scene $resultsdir
#mv $datereffile $resultsdir
#mv $rawlogdir $resultsdir
#cp $scriptPath $resultsdir
#echo $* > $resultsdir/notes.txt
echo "Exit"

