#!/bin/bash

#variable bash para el fichero de lock
lockfile=/home/pi/zowi/python/tmp/zowi.lock
EJECUCION=$((0))
PID_OLD=$((0))	
	
	
while true
do
	#Modificacion matar avrdude
		PID=`ps -A | grep avrdude | awk '{print $1}'`
		if [ "$PID" != "$PID_OLD" ];then
			EJECUCION=$((0))
		fi
		if [ "$PID" != "" ];then
			EJECUCION=$((EJECUCION+1))
		fi
		if [ $EJECUCION -gt 4 ];then
			sudo kill -9 $PID
			EJECUCION=$((0))
		fi
		PID_OLD=$((PID))

	
		#Tiempo para reiniciar el script
	sleep 4
done