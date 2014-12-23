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

printRunning "Getting PID library..." &
LASTPID=$!
wget -q -O${TEMPFOLDER}/PID.zip https://github.com/br3ttb/Arduino-PID-Library/zipball/master
kill $LASTPID
wait $LASTPID 2>/dev/null
echo -e "\bDone"

printRunning "Getting PID AutoTune library..." &
LASTPID=$!
wget -q -O${TEMPFOLDER}/PID-AutoTune.zip https://github.com/br3ttb/Arduino-PID-AutoTune-Library/zipball/master
kill $LASTPID
wait $LASTPID 2>/dev/null
echo -e "\bDone"

echo -n "Extracting libraries... "
unzip -qd ${TEMPFOLDER} ${TEMPFOLDER}/PID.zip
unzip -qd ${TEMPFOLDER} ${TEMPFOLDER}/PID-AutoTune.zip
for d in $(find ${TEMPFOLDER}/br3tt* -type d -name PID*_v*)
do
    sudo mv "$d" /opt/arduino/libraries
done
echo "Done"

rm -Rf ${TEMPFOLDER}
