#!/bin/bash

ARDUINOVERSION="1.5.8"

TEMPFOLDER=$(mktemp -d)
ARCH="32"

if uname -p | grep -q 64
then
    ARCH="64"
fi

ARDUINOARCHIVE="arduino-${ARDUINOVERSION}-linux${ARCH}.tgz"


function printRunning(){
    echo -n "$1  "
    CHARS=("/" "-" "\\" "|")
    CURRIND=0
    while true
    do
        echo -en "\b${CHARS[$CURRIND]}"
        CURRIND=$(( ($CURRIND + 1) % 4 ))
        sleep 0.1
    done
}

printRunning "Getting Arduino archive: ${ARDUINOARCHIVE}..." &
LASTPID=$!
wget -q -O${TEMPFOLDER}/${ARDUINOARCHIVE} http://arduino.cc/download.php?f=/${ARDUINOARCHIVE}
kill $LASTPID
wait $LASTPID 2>/dev/null
echo -e "\bDone"

echo "Extracting archive to /opt..."
sudo tar -zx -C /opt -f ${TEMPFOLDER}/${ARDUINOARCHIVE}
echo "Creating symlink."
sudo ln -s /opt/arduino-${ARDUINOVERSION} /opt/arduino

printRunning "Getting CmdArduino library..." &
LASTPID=$!
wget -q -O${TEMPFOLDER}/CmdArduino.zip https://github.com/joshmarinacci/CmdArduino/archive/master.zip
kill $LASTPID
wait $LASTPID 2>/dev/null
echo -e "\bDone"

echo -n "Extracting libraries... "
unzip -qd ${TEMPFOLDER} ${TEMPFOLDER}/CmdArduino.zip
sudo mv ${TEMPFOLDER}/CmdArduino-master /opt/arduino/libraries/CmdArduino


echo "Done"

rm -Rf ${TEMPFOLDER}
