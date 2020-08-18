global pose


for j = 1:2
    for i = 1:4

        [log, pose]= RemoteCommand(S, log, 'pose', false);
        joint_pos = rad2deg(pose.actual_joint_position);
        joint_pos(1) = joint_pos(1) + 5;
        joint_pos(2) = joint_pos(2) - 5;
        joint_pos(3) = joint_pos(3) + 5;
        joint_pos(4) = joint_pos(4) - 5;
        joint_pos(5) = joint_pos(5) + 5;
        joint_pos(6) = joint_pos(6) - 5;
        [log, pose]= RemoteCommand(S, log, {'move', 'J', deg2rad(joint_pos),[1,0.5,0,0]}, false);
    end

    for i = 1:4
        [log, pose]= RemoteCommand(S, log, 'pose', false);
        joint_pos = rad2deg(pose.actual_joint_position);
        joint_pos(1) = joint_pos(1) - 5;
        joint_pos(2) = joint_pos(2) + 5;
        joint_pos(3) = joint_pos(3) - 5;
        joint_pos(4) = joint_pos(4) + 5;
        joint_pos(5) = joint_pos(5) - 5;
        joint_pos(6) = joint_pos(6) + 5;
        [log, pose]= RemoteCommand(S, log, {'move', 'J',  deg2rad(joint_pos),[1,0.5,0,0]}, false);
    end
    
    for i = 1:4
        [log, pose]= RemoteCommand(S, log, 'pose', false);
        joint_pos = rad2deg(pose.actual_joint_position);
        joint_pos(1) = joint_pos(1) - 5;
        joint_pos(2) = joint_pos(2) + 5;
        joint_pos(3) = joint_pos(3) - 5;
        joint_pos(4) = joint_pos(4) + 5;
        joint_pos(5) = joint_pos(5) - 5;
        joint_pos(6) = joint_pos(6) + 5;
        [log, pose]= RemoteCommand(S, log, {'move', 'J',  deg2rad(joint_pos),[1,0.5,0,0]}, false);
    end
    
    for i = 1:4

        [log, pose]= RemoteCommand(S, log, 'pose', false);
        joint_pos = rad2deg(pose.actual_joint_position);
        joint_pos(1) = joint_pos(1) + 5;
        joint_pos(2) = joint_pos(2) - 5;
        joint_pos(3) = joint_pos(3) + 5;
        joint_pos(4) = joint_pos(4) - 5;
        joint_pos(5) = joint_pos(5) + 5;
        joint_pos(6) = joint_pos(6) - 5;
        [log, pose]= RemoteCommand(S, log, {'move', 'J',  deg2rad(joint_pos),[1,0.5,0,0]}, false);
    end
    
end