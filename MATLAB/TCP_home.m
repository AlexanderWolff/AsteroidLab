
[log, pose]= RemoteCommand(S, log, 'pose', false);
TCP_pose = pose.actual_tcp_pose;
TCP_pose(1) = -100*1E-3;
TCP_pose(2) = -100*1E-3;
TCP_pose(3) =  450*1E-3;
TCP_pose(4) =  deg2rad(0);
TCP_pose(5) =  deg2rad(0);
TCP_pose(6) =  deg2rad(0);
[log, pose]= RemoteCommand(S, log, {'move', 'L', TCP_pose,[1,1,5,0]}, false);
[log, pose]= RemoteCommand(S, log, 'pose', false);