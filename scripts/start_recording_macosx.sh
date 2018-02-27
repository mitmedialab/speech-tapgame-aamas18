#!/bin/bash
#Quit Dropbox and Google Drive
osascript -e 'quit app "Dropbox"'
osascript -e 'quit app "Google Drive"'
#Quit Photo Booth
osascript -e 'quit app "Photo Booth"'

# if your script hangs, comment the following two lines
p=`lsof | grep "VDC.plugin" | awk '{print $2}'`
kill -9 $p
sleep 0.5s

#check $1
list="p00 p01 p02 p03 p04 p05 p06 p07 p08 p09 p10 p11"

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./start_recording_macosx.sh <participant_id> <experimenter>"
  exit
fi

#check $2
list='huili safinah maggie'
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: session [$2] does not exist"
  echo "Usage: ./start_recording_macosx.sh <participant_id> <experimenter>"
  exit
fi

# 1. get device indices
DEVICE_LIST=`ffmpeg -f avfoundation -list_devices true -i "" 2>&1 | grep "C930e" | awk '{print $6}' | tr -d []`

# 2. Check if 4 devices found (2 camera, 2 audio)
if [ ${#DEVICE_LIST} -eq 7 ]
then
  
  echo ">>>VIDEO RECORD STARTING. TO QUIT RECORDING, PRESS 'q' IN EACH TAB.<<<";echo

  DEVICE1="\"${DEVICE_LIST:0:1}:${DEVICE_LIST:4:1}\""
  DEVICE2="\"${DEVICE_LIST:2:1}:${DEVICE_LIST:6:1}\""
  echo "VIDEO1: $DEVICE1    VIDEO2: $DEVICE2"
else
  echo ">>>Logitech C930e devices not found. Check devices below.<<<"
  echo ">>>If two webcams are connected, run the following command to reset.<<<";echo
  echo -e "\033[1msudo killall AppleCameraAssistant;sudo killall VDCAssistant\033[0m";echo;echo
  ffmpeg -f avfoundation -list_devices true -i ""  2>&1 | grep "input"
  exit
fi

# 3. Set variable for output video files
VID_DIR="videos/"
mkdir -p $VID_DIR

DATE=`date +%Y-%m-%d-%H-%M-%S`
OUT1="$VID_DIR$1_$2_vid1_$DATE.mp4"
OUT2="$VID_DIR$1_$2_vid2_$DATE.mp4"
echo "OUT1: $OUT1"
echo "OUT2: $OUT2"


# 4. write ffmpeg script to file.

echo -e "ffmpeg -y -f avfoundation -s 640x480 -r 30 -i $DEVICE1 \\
	-vf \"drawbox=y=0: color=black@1.0: width=iw:height=40: t=20, \\
  drawtext=fontfile=/Library/Fonts/Arial.ttf:  \\
	text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: x=7: y=15: box=1: boxcolor=black@1.0\" \\
  $OUT1" > record_video1.sh

echo -e "ffmpeg -y -f avfoundation -s 640x480 -r 30 -i $DEVICE2 \\
	-vf \"drawbox=y=0: color=black@1.0: width=iw:height=40: t=20, \\
  drawtext=fontfile=/Library/Fonts/Arial.ttf:  \\
	text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: x=7: y=15: box=1: boxcolor=black@1.0\" \\
  $OUT2" > record_video2.sh

chmod +x record_video1.sh record_video2.sh

# 5. execute files in separate tabs.
function new_tab() {
  TAB_NAME=$1
  COMMAND=$2
  echo $COMMAND
  osascript \
    -e "tell application \"Terminal\"" \
    -e "tell application \"System Events\" to keystroke \"t\" using {command down}" \
    -e "do script \"printf '\\\e]1;$TAB_NAME\\\a'; cd $PWD; $COMMAND\" in front window" \
    -e "end tell" > /dev/null
}

new_tab "VIDEO1" "./record_video1.sh"
new_tab "VIDEO2" "./record_video2.sh"





