#!/bin/bash

scriptName=$(basename $0)
echo $scriptName
pid=$$

rbasedir=$1

buoy_addr=0
hil_ac_addr=1
explorer1_addr=2
explorer2_addr=3
explorer3_addr=4


##############################
##############################

awktime='
BEGIN{\
	initfound = 0
	t0 = 0
}
{\
	if(!initfound)
	{
		where = match($0, "INIT POSITION REACHED")
		if( where != 0 )
		{
			where = match($0, "[0-9]+\\/[0-9]+\\/[0-9]+ [0-9]+:[0-9]+:[0-9]+\\.[0-9]+")
			if( where != 0 )
			{
				initfound = 1
				time=substr($0, RSTART, RLENGTH)
				f = "date -d \""time"\" +%s.%N"
				f | getline t0
				close(f)
			}
		}
	}
	else{
		where = match($0, "[0-9]+\\/[0-9]+\\/[0-9]+ [0-9]+:[0-9]+:[0-9]+\\.[0-9]+")
		if(where != 0)
		{
		    time=substr($0, RSTART, RLENGTH)
	            where = match($0, patt)
		    if(where != 0)
		    {
			    f = "date -d \""time"\" +%s.%N" 
			    f | getline tt
			    close(f)
			    tt2 = (tt - t0)
			    printf("%.9f\t%f\t%f\n", tt2, $6, $8)
	             }
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

##############################
##############################

dataplotdir=$rbasedir/dataplot
plotsdir=$rbasedir/plots

rm -rf $dataplotdir $plotsdir
sleep 5s
for proto in $rbasedir/*
do
	proto=$(basename "$proto")
	protodataplotdir=$dataplotdir/$proto
	e1tr=$protodataplotdir/derr1.tr
	e2tr=$protodataplotdir/derr2.tr
	e3tr=$protodataplotdir/derr3.tr
	hiltr=$protodataplotdir/hilderr.tr

	logdir=$rbasedir/$proto/results/
	uwsimlog=$logdir/uwsimnet.log
	datereffile=$logdir/dateref
	echo $logdir
	mkdir -p $protodataplotdir

	cat $uwsimlog | awk -v dateref=$datereffile -v patt="E$explorer1_addr DERR" "$awktime" | awk '{ print $1" "$2 }' > $e1tr
	cat $uwsimlog | awk -v dateref=$datereffile -v patt="E$explorer2_addr DERR" "$awktime" | awk '{ print $1" "$2 }' > $e2tr
	cat $uwsimlog | awk -v dateref=$datereffile -v patt="E$explorer3_addr DERR" "$awktime" | awk '{ print $1" "$2 }' > $e3tr
	cat $uwsimlog | awk -v dateref=$datereffile -v patt="HIL DERR" "$awktime" | awk '{ print $1" "$2 }' > $hiltr
	
done

titles=(Explorer1 Explorer2 Explorer3 HIL)
datafiles=(derr1.tr derr2.tr derr3.tr hilderr.tr)
plotnames="explorer1_err.pdf explorer2_err.pdf explorer3_err.pdf hil_err.pdf"
mkdir -p $plotsdir
idx0=0
for plot in $plotnames
do
	echo "Generating $plot..."
	tmpplotfile=$plotsdir/$plot.plot
	cp xyautoscale.plot $tmpplotfile
	echo -e -n "plot" >> $tmpplotfile

	for proto in $dataplotdir/*
	do
		echo -e "\t- proto: $proto"
		proto=$(basename "$proto")
		protodir=$dataplotdir/$proto
		datafile=${datafiles[$idx0]}
		datafile=$protodir/$datafile
		title=$proto
	        newplotline=" \"$datafile\" using 1:2 w lp ps 0.2 title \"$title\", \\"
	        echo -e "$newplotline" >> $tmpplotfile
	done
	title=${titles[$idx0]}
	plotcmd="gnuplot -e \"title='$title' ; ylabel='Error (m)' ; xlabel='Time (s)' \" $tmpplotfile > $plotsdir/$plot"
	eval $plotcmd

	let idx0=idx0+1

	echo "$plot generated"
done


