#!/bin/bash
# killall vlc
#kill $(ps aux | grep 'vlc' | awk '{print $2}')
# killall camera resources
#p=`lsof | grep "/dev/video" | awk '{print $2}'`
#kill -9 $p
#sleep 0.5s


# 1. get device indices
echo "video list"
#echo `ffmpeg -sources 2>&1`
#VIDEO_LIST=`ffmpeg -sources 2>&1 | grep "/dev/video" | grep "C930e" | awk '{print $1}'`
AUDIO_LIST=`arecord -l | grep "C930e" | awk '{print $2}' | tr -d :` 
VIDEO_LIST=`ffmpeg -sources 2>&1  | grep "/dev/video*[VMware Virtual USB Video Device]" | awk '{print $1}'`

echo $VIDEO_LIST


# 2. Check if 4 devices found (2 camera, 2 audio)

#if [ ${#VIDEO_LIST} -eq 2 ] && [ ${#AUDIO_LIST} -eq 3 ]
#then
  
  echo ">>>VIDEO RECORD STARTING. TO QUIT RECORDING, PRESS 'q' IN EACH TAB.<<<";echo

  VIDEO1="\"${VIDEO_LIST:0:11}\""
  VIDEO2="\"${VIDEO_LIST:12:11}\""
  AUDIO1="${AUDIO_LIST:0:1}"
  AUDIO2="${AUDIO_LIST:2:1}"
  echo "VIDEO1: $VIDEO1    VIDEO2: $VIDEO2"
  echo "AUDIO1: $AUDIO1    AUDIO2: $AUDIO2"
# else
#   echo ">>>Logitech C930e devices not found. Check devices below.<<<"
#   echo ">>>If two webcams are connected, run the following command to reset.<<<";echo
#   echo -e "\033[1msudo modprobe -r uvcvideo && sudo modprobe uvcvideo\033[0m";echo;echo
#   echo
#   echo "Video Device List:"
#   ffmpeg -sources | grep "/dev/video"
#   echo "Audio Device List:"
#   arecord -l | grep "C930e"
#   exit
#fi

# 3. Set variable for output video files
VID_DIR="videos/"
mkdir -p $VID_DIR

DATE=`date +%Y-%m-%d-%H-%M-%S`
OUT1="$VID_DIR$1_$2_vid1_$DATE.mp4"
OUT2="$VID_DIR$1_$2_vid2_$DATE.mp4"
echo "OUT1: $OUT1"
echo "OUT2: $OUT2"


# 4. write ffmpeg script to file.

echo -e "ffmpeg -y -f video4linux2 -s 640x480 -r 30 -i $VIDEO2 \\
   -ac 2   -vf drawtext=\"fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf: \\
text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: fontsize=24: box=1: boxcolor=black@0.5: \\
boxborderw=5: x=(w-text_w)/2: y=0\" \\
  $OUT1" > record_video1.sh

echo -e "ffmpeg -y -f video4linux2 -s 640x480 -r 30 -i $VIDEO1 \\
    -ac 2   -vf drawtext=\"fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf: \\
text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: fontsize=24: box=1: boxcolor=black@0.5: \\
boxborderw=5: x=(w-text_w)/2: y=0\" \\
  $OUT2" > record_video2.sh

#-i hw:$AUDIO1,0
#-i hw:$AUDIO2,0

chmod +x record_video1.sh record_video2.sh

# 5. execute files in separate tabs.

#new_tab "VIDEO1" "./record_video1.sh"
#new_tab "VIDEO2" "./record_video2.sh"


WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

sleep 1; xdotool type --delay 1 --clearmodifiers "./record_video1.sh"; xdotool key Return;


WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

sleep 1; xdotool type --delay 1 --clearmodifiers "./record_video2.sh"; xdotool key Return;


