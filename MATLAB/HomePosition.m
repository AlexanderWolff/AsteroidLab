


[log, pose]= RemoteCommand(S, log, 'pose', false);
joint_pos(1) =    0;
joint_pos(2) =  -90;
joint_pos(3) =    0;
joint_pos(4) =    0;
joint_pos(5) =    0;
joint_pos(5) =   90;
[log, pose]= RemoteCommand(S, log, {'move', 'J', deg2rad(joint_pos),[1,1,5,0]}, false);
[log, pose]= RemoteCommand(S, log, 'pose', false);