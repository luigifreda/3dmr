
- for adding a new custom message see v_repConst.h: 
 lines from 1216 on, here there are the constant for the publisher and subscribers (set or get)
 see line 1253 for the new custom message of tracks
 
- for add a new publisher see the file in Ros_server.cpp, in particular 
 1) line 825 for the publisher init
 2) void ROS_server::streamAllData() for the management of the particular publisher
 here all the publishers are managed (included tf broadcasting)
 see line 1723 for the customization and addition of a new publisher with a new kind of message 
 if(publishers[pubI].cmdID==simros_strmcmd_get_tracks_vel)

- for adding a subscriber see the filename 


- NOTE: in order to get a tf_remapper working here 
1) get the tfMessage published on a /tf_old
2) then you can use the tf_remapper itself
