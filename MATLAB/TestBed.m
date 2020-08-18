
% 
% [log, pose]= RemoteCommand(S, log, 'pose', true);
% TCP_final_pose = pose.actual_tcp_pose;
% TCP_final_pose(1) = -440*1E-3;
% TCP_final_pose(2) = -150*1E-3;
% TCP_final_pose(3) =  475*1E-3;
% TCP_final_pose(4) =  deg2rad(0);
% TCP_final_pose(5) =  deg2rad(0);
% TCP_final_pose(6) =  deg2rad(0);
% 
% TCP_midway_pose = pose.actual_tcp_pose;
% TCP_midway_pose(1) = -200*1E-3;
% TCP_midway_pose(2) = -130*1E-3;
% TCP_midway_pose(3) =  450*1E-3;
% TCP_midway_pose(4) =  deg2rad(0);
% TCP_midway_pose(5) =  deg2rad(0);
% TCP_midway_pose(6) =  deg2rad(0);
% [log, pose]= RemoteCommand(S, log, {'move', 'C', TCP_final_pose,[0.1,0.1,0.1,1],TCP_midway_pose}, false);
% 

%[log, ~] = RemoteCommand(S, log, {'instruction', 'Warning! Close Windows!'}, true);
% 
% [log, ~]= RemoteCommand(S, log, {'wait', 5}, true);

[log, ~] = RemoteCommand(S, log, 'halt', true);