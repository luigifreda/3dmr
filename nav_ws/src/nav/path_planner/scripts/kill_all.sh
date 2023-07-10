#/bin/bash

rosnode kill -a
sleep 1

#killall -9 xterm
kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')
#sleep 5
#kill $(ps aux | grep monitor | grep -v grep | awk '{print $2}')

kill -9 $(ps aux | grep gazebo_ugv_teleop_key | grep -v grep | awk '{print $2}')
