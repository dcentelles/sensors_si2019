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
			    printf("%.9f\t%s\n", tt2, $0)
	             }
		} 
	}
}
END{\
}
'

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

awkminmax='
BEGIN{\
	min = 999999999999999999999999
	max = -999999999999999999999999
}
{\
	value = $col
	if ( value < min )
		min = value
	if ( value > max )
		max = value
}
END{\
	printf("colmin=%.9f\n", min);
	printf("colmax=%.9f\n", max);
}'



##############################
##############################

dataplotdir=$rbasedir/dataplot
plotsdir=$rbasedir/plots

rm -rf $dataplotdir $plotsdir
for proto in $rbasedir/*
do
	proto=$(basename "$proto")
	protodataplotdir=$dataplotdir/$proto
	e1tr=$protodataplotdir/derr1.tr
	e2tr=$protodataplotdir/derr2.tr
	e3tr=$protodataplotdir/derr3.tr
	hiltr=$protodataplotdir/hilderr.tr
	updatepositiontr=$protodataplotdir/updateposition.tr

	
	logdir=$rbasedir/$proto/results/
	uwsimlog=$logdir/uwsimnet.log
	echo $logdir
	mkdir -p $protodataplotdir

	## COMPUTE TARGET ERROR

	cat $uwsimlog | awk -v patt="E$explorer1_addr DERR" "$awktime" | awk '{ print $1" "$7 }' > $e1tr
	cat $uwsimlog | awk -v patt="E$explorer2_addr DERR" "$awktime" | awk '{ print $1" "$7 }' > $e2tr
	cat $uwsimlog | awk -v patt="E$explorer3_addr DERR" "$awktime" | awk '{ print $1" "$7 }' > $e3tr
	cat $uwsimlog | awk -v patt="HIL DERR" "$awktime" | awk '{ print $1" "$7 }' > $hiltr

	cat $uwsimlog | awk -v patt="BUOY: UPDATE POSITION" "$awktime" | awk '{ print $1 }' > $updatepositiontr


	## COMPUTE COMMUNICATION DATA PLOT

	hil2buoy_tx="HIL: TX TO 0"
	hil2buoy_rx="BUOY: RX FROM 1"
	e12buoy_tx="E2: TX TO 0"
	e12buoy_rx="BUOY: RX FROM 2"
	e22buoy_tx="E3: TX TO 0"
	e22buoy_rx="BUOY: RX FROM 3"
	e32buoy_tx="E4: TX TO 0"
	e32buoy_rx="BUOY: RX FROM 4"
	buoy2hil_tx="BUOY: TX TO 1"
	buoy2hil_rx="HIL: RX FROM 0"

	
	for pair in \
		"$hil2buoy_tx,$hil2buoy_rx,HIL -> BUOY,hil_buoy" \
		"$e12buoy_tx,$e12buoy_rx,E1 -> BUOY,e1_buoy" \
		"$e22buoy_tx,$e22buoy_rx,E2 -> BUOY,e2_buoy" \
		"$e32buoy_tx,$e32buoy_rx,E3 -> BUOY,e3_buoy" \
		"$buoy2hil_tx,$buoy2hil_rx,BUOY -> HIL,buoy_hil"
	do 
	        txpatt="$(cut -d',' -f1 <<< $pair)"
	        rxpatt="$(cut -d',' -f2 <<< $pair)"
	        title="$(cut -d',' -f3 <<< $pair)"
		filename="$(cut -d',' -f4 <<< $pair)"
	        echo "TX PATTERN: $txpatt"
	        echo "RX PATTERN: $rxpatt"
		echo "FLOW: $title"

		txtr=$protodataplotdir/${filename}_tx.tr
		rxtr=$protodataplotdir/${filename}_rx.tr
		end2endtr=$protodataplotdir/${filename}_end2end.tr
		jittertr=$protodataplotdir/${filename}_jitter.tr
		genresults=$protodataplotdir/${filename}_genresults.txt
		parsererrors=$protodataplotdir/${filename}_parsererrors.txt

		#generate plot files time - seq.num

		cat $uwsimlog | awk -v patt="$txpatt" "$awktime" | awk ' {printf("%.9f\t%s\n", $1, $9) }' > $txtr
		cat $uwsimlog | awk -v patt="$rxpatt" "$awktime" | awk ' {printf("%.9f\t%s\n", $1, $9) }' > $rxtr

		########## END 2 END ###################
		curlinenum=1
		while read line
		do
			t1=$(echo "$line" | cut -f 1 -d$'\t')
			seq=$(echo "$line" | cut -f 2 -d$'\t')
			#echo "T1: $t1 - SEQ: $seq"
			found=false
			curlinenum_before=$curlinenum
			while [[ $found = false ]]
			do
				lineb=$(sed -n "$curlinenum p" < $txtr)
				#echo "cur line: $lineb"
				if [[ $lineb ]]
				then
					t0=$(echo "$lineb" | cut -f 1 -d$'\t')
					seqb=$(echo "$lineb" | cut -f 2 -d$'\t')
					if [ $seq -eq $seqb ]
					then
						end2end=$(bc <<< "scale=9; $t1 - $t0")
						#echo "T1: $t1 --- T0: $t0 --- END2END: $end2end"
						echo -e "$seq\t$end2end" >> $end2endtr
						found=true
					fi
				else
					echo "THIS IS NOT POSSILE: $curlinenum" >> $parsererrors
					break
				fi
				((curlinenum++))
			done
			if [[ $found = false ]]
			then
				#Es posible que el evento de transmision se haya filtrado (se ha realizado antes de que el robot llegue a la posicion de inicio
				curlinenum=$curlinenum_before
			fi
		done < $rxtr
		echo "End To End:" | tee -a $genresults
		cat $end2endtr | awk -v col=2 "$awkavg" | tee -a $genresults
		cat $end2endtr | awk "$awkjitter" > jitter.tr.tmp
		echo "Jitter:" | tee -a $genresults
		cat jitter.tr.tmp | head -n -1 > $jittertr
		cat jitter.tr.tmp | tail -n 1 | tee -a $genresults
		rm -f *.tmp
	done #FOR PAIR
done #FOR PROTO

xlabeldef="Time (s)"
e2elabel="End to End (s)"
e2exlabel="Packet Seq. num."
titles=(Explorer1 Explorer2 Explorer3 HIL "HIL->BUOY" "E1->BUOY" "E2->BUOY" "E3->BUOY" "BUOY->HIL")
datafiles=(derr1.tr derr2.tr derr3.tr hilderr.tr hil_buoy_end2end.tr e1_buoy_end2end.tr e2_buoy_end2end.tr e3_buoy_end2end.tr buoy_hil_end2end.tr)
plotnames="explorer1_err explorer2_err explorer3_err hil_err hil_buoy_e2e e1_buoy_e2e e2_buoy_e2e e3_buoy_e2e buoy_hil_e2e"
ylabels=("Error (m)" "Error (m)" "Error (m)" "Error (m)"     "$e2elabel"  "$e2elabel"  "$e2elabel"  "$e2elabel"  "$e2elabel") 
xlabels=("$xlabeldef" "$xlabeldef" "$xlabeldef" "$xlabeldef" "$e2exlabel" "$e2exlabel" "$e2exlabel" "$e2exlabel" "$e2exlabel")
vertical=(0 0 0 1 0 0 0 0 0)
mkdir -p $plotsdir

updatePositionCmd="set arrow from $position,graph(0,0) to $position,graph(1,1) nohead lw 3 lc rgb \"red\""
idx0=0
for plot in $plotnames
do
	echo "Generating $plot..."
	tmpplotfile=$plotsdir/$plot.plot
	cp xyautoscale.plot $tmpplotfile
	
	plotvertical=${vertical[$idx0]}
	plotvertical_file=$plotvertical

	if [[ "$plotvertical_file" == 1 ]]
	then
		tmpplotfile_verticals=$plotsdir/${plot}_verticals.plot
		cp xyautoscale.plot $tmpplotfile_verticals
	fi

	firstplot=1

	for proto in $dataplotdir/*
	do
		echo -e "\t- proto: $proto"
		proto=$(basename "$proto")
		protodir=$dataplotdir/$proto
		datafile=${datafiles[$idx0]}
		datafile=$protodir/$datafile
		title=$proto
		if [[ "$plotvertical" == 1 ]]
		then
			awk '{print "set arrow from "$1",graph(0,0) to "$1",graph(1,1) nohead lw 3 lc rgb \"red\"" }' $protodir/updateposition.tr >> $tmpplotfile_verticals
			plotvertical=0	
		fi
		if [[ "$firstplot" == 1 ]]
		then
			echo -e -n "plot" >> $tmpplotfile
			if [[ "$plotvertical_file" == 1 ]]
			then
				echo -e -n "plot" >> $tmpplotfile_verticals
			fi
			firstplot=0
		fi
	        newplotline=" \"$datafile\" using 1:2 w lp ps 0.2 title \"$title\", \\"
	        echo -e "$newplotline" >> $tmpplotfile
		if [[ "$plotvertical_file" == 1 ]]
		then
	        	echo -e "$newplotline" >> $tmpplotfile_verticals
		fi
	done
	title=${titles[$idx0]}
	ylabel=${ylabels[$idx0]}
	xlabel=${xlabels[$idx0]}
	plotcmd="gnuplot -e \"title='$title' ; ylabel='$ylabel' ; xlabel='$xlabel' \" $tmpplotfile > $plotsdir/${plot}.pdf"
	eval $plotcmd

	if [[ "$plotvertical_file" == 1 ]]
	then
		plotcmd="gnuplot -e \"title='$title' ; ylabel='$ylabel' ; xlabel='$xlabel' \" $tmpplotfile_verticals > $plotsdir/${plot}_verticals.pdf"
		eval $plotcmd
	fi

	let idx0=idx0+1

	echo "$plot generated"
done


