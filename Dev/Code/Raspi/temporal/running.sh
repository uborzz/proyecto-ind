#!/bin/bash

#variable bash para el fichero de lock
lockfile=/home/pi/zowi/python/tmp/zowi.lock

#borramos el directorio tmp por seguridad
rm -rf /home/pi/zowi/python/tmp/
#creamos el directorio tmp nuevo (vacio)
mkdir /home/pi/zowi/python/tmp/

while true
do
	#Si no hay un fichero en el directorio tmp (zowi.lock) lanzamos el script python
	LITEMS=`ls /home/pi/zowi/python/tmp/ | wc -l`
	if [[ "$LITEMS" == "0" ]]
	then
		#Comprueba si se está ejecutando el sript python(si no se ejecuta lo lanza)
		if (set -o noclobber; echo "$$" > "$lockfile") 2> /dev/null;
		then
			trap 'rm -f "$lockfile"; exit $?' INT TERM EXIT
			echo "Ejecutamos script"
			python /home/pi/zowi/python/main.mysql.py
			#Borramos el fichero lock por cierre del script de python
			rm -f "$lockfile"
			trap - INT TERM EXIT
		else
			echo "Ya se esta ejecutando el script"
			echo "PID de running.sh: $(cat $lockfile)"
		fi
	fi

	
	#Tiempo para reiniciar el script
	sleep 4
done
