#!/bin/bash

lockfile=/home/pi/zowi/python/tmp/zowi.lock

if (set -o noclobber; echo "$$" > "$lockfile") 2> /dev/null;then
	trap 'rm -f "$lockfile"; exit $?' INT TERM EXIT
	echo "Ejecutando script"
	python /home/pi/zowi/python/main.mysql.py
	rm -f "$lockfile"
	trap - INT TERM EXIT
else
	echo "Ya se esta ejecutando el script"
	echo "corriendo con el PID: $(cat $lockfile)"
fi
