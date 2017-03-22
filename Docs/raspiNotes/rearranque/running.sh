#!/bin/bash

while true
do
	LITEMS=`ls /var/lock | wc -l`
	if [[ "$LITEMS" != "1" ]]
	then
		'/home/pi/zowi/python/start_script.sh'
	fi
	sleep 5
done
