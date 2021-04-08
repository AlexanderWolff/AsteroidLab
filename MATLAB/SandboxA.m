%t = tcpclient('192.168.0.11',30002);

%directory = "/home/wolff/Documents/ASTEROIDLAB/UR5RobotControlV3.2";
%command = fileread(directory+"/Program_Template.script");
%command = fileread(directory+"/Robot_Dance.script");

% t = tcpclient('192.168.0.11',30002);
% directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
% command = fileread(directory+"/Program_Control.script");
% write(t, unicode2native(command,'US-ASCII'));

% 
% x = [tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      tcpa_joints,...
%      tcpb_joints,...
%      jointsb,...
%      jointsa];
% 
% 
% UR3.Move_Joints(x,[2,2])

% scale in mm
% scale = 100;
% 
% t = linspace(0,2*pi,1000);
% x = 16*(sin(t)).^3;
% y = 13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t);
% x = (x/max(x))*scale;
% y = (y/max(y))*scale;
% plot(x,y)
% 
% actual_j = UR3.Get_Joint_Pos;
% 
% % theta (rad), r (m), d(m), alpha (rad)
% p= [ 0,        0, 0.15190, pi/2; ...
%      0, -0.24365,       0,    0; ...
%      0, -0.21325,       0,    0; ...
%      0,        0, 0.11235, pi/2; ...
%      0,        0, 0.08535,-pi/2; ...
%      0,        0, 0.08190,    0];
% 
% R = cell(1,6);
% T = cell(1,6);
% H = cell(1,6);
%  
% for i = 1:6
%     theta = actual_j(i) + p(i,1);
%     r = p(i,2);
%     d = p(i,3);
%     alpha = p(i,4);
% 
%     R{i} = [ cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha);...
%              sin(theta),  cos(theta)*cos(alpha),-cos(theta)*sin(alpha);...
%                       0,             sin(alpha),            cos(alpha)];
% 
%     T{i} = [r*cos(theta); r*sin(theta); d];
%     
%     H{i} = [[R{i};zeros(1,3)],[T{iUr3.e};1]];
% end
% 
% start = [0;0;0]
% x = H{i};
% y = H{i};
% for i = 2:6
%     
%     % Rotate
%     x = x*H{i}
%     y = H{i}*y
% 
% end

% actual_joints = UR3.Get_Joint_Pos;
% actual_pose = UR3.Get_TCP_Pos;
% actual_xyz = RMatrix.fromAngleAxisMag(actual_pose(4:6)).toXYZ;
% 
% sim = Simulation('UR3');
% F = sim.trans(rad2deg(actual_joints));
% sim_xyz = F{6}.toXYZ;
% error = sim_xyz-actual_xyz;
% 
% % Orientation Error
% orientation_actual_sim_error_rad = [actual_xyz;sim_xyz;error]
% %actual_sim_error_def = rad2deg(actual_sim_error_rad)
% 
% 
% % Position Error
% actual_xyz = actual_pose(1:3)';
% sim_xyz = F{7}.T';
% error = sim_xyz-actual_xyz;
% position_actual_sim_error_rad = [actual_xyz;sim_xyz;error]
% 
% actual = Homogeneous.fromT(actual_xyz);
% actual.setR( RMatrix.fromAngleAxisMag(actual_pose(4:6)).R );
% actual.plotL('actual');

% Distances in m
% hypo = (1580-125/2-146/2)*1e-3;%1480.4e-3;
% adja = (1343+7.3)*1e-3;%150.15e-2;%
% oppo = (668-87.5)*1e-3;%46.15e-2;%
% 
% theta = zeros(3,1);
% theta(1) = asin(oppo/hypo);
% theta(2) = acos(adja/hypo);
% theta(3) = atan2(oppo,adja);
% rad2deg(theta)
% 
% h = zeros(3,1);
% a = zeros(3,1);
% o = zeros(3,1);
% 
% h(1) = sqrt( adja^2 + oppo^2 )

% Table Dimensions in M (from UR3 Base)


sim = Simulation('UR3');



%% Target
Target = Homogeneous.fromT([-0.2,-0.2,0.2] );%((rand(1,3)/3)));
random = rad2deg((rand(1,3)-0.5)*2*pi)

% % Alignment
% random = [64.3447   92.7864   87.5277];
Target.setR(RMatrix.fromXYZ(random).R);

%% Check that Target is within Workspace
% TODO

%% Begin Inverse Kinematics
% Initialise Pose
joints = [0,0,0,0,90,0];
sim.ForwardKinematics(joints);


[joint_history, distances] = sim.InverseK_Position(Target, false);
disp(distances')
plotJoints_Time(sim, Target, joint_history)
% 
% % Home to position
% for i = 1:1
%     
%     [joint_history, distances] = sim.InverseK_Position(Target, false);
%     plotJoints_Time(sim, Target, joint_history)
%     sim.align_orientation(Target)
%     closePlotSim(sim,Target);
% end
% 
% 
% closePlotSim(sim,Target);
% axis equal

% for i = 1:10
%     
%     % Determine if Target and Tool are aligned in orientation
%     
%     % Iterate
%     [joint_history, distances] = sim.InverseK_Position(Target, false);
%     sim.align_Joint6(Target);
%     sim.align_Joint4(Target);
% end

%-82.8092 90 51.8419

%plotJoints_Time(sim, Target, joint_history)

% %% Align with Target in orientation
% Target_F0 = Target;
% sim.ForwardKinematics(joint_history(end,:))
% 
% % Get F4, F5, F6, F7, FTool and Target in terms of F4
% F6_F5 = sim.transform.chain{6};
% F7_F6 = sim.transform.chain{7}; 
% FTool_F7 = sim.transform.chain{8}; 
% F4_F0 = sim.transform.local{4};
% F0_F4 = F4_F0.inv;
% 
% F4_F4 = Homogeneous.Empty;
% F5_F4 = sim.transform.chain{5};
% F6_F4 = F6_F5.transform(F5_F4);
% F7_F4 = F7_F6.transform(F6_F4);
% FTool_F4 = FTool_F7.transform(F7_F4);
% Target_F4 = Target_F0.transform(F0_F4);
% 
% scale = 0.1;
% TargetX = Homogeneous.fromT([scale,0,0]);
% TargetX = TargetX.transform(Target_F4);
% TargetZ = Homogeneous.fromT([0,0,scale]);
% TargetZ = TargetZ.transform(Target_F4);
% 
% ToolX = Homogeneous.fromT([scale,0,0]);
% ToolX = ToolX.transform(FTool_F4);
% ToolZ = Homogeneous.fromT([0,0,scale]);
% ToolZ = ToolZ.transform(FTool_F4);
% 
% %% Align X axis along Z axis of F7
% sim.align_Joint6(Target_F0);
% 
% closePlotSim(sim,Target);
% 
% %% Make Z axis of TOol parallel to Z axis of Target in X-Y by moving F4
% sim.align_Joint4(Target_F0);

%closePlotSim(sim,Target);




%% Check that solution is feasible (not intersecting robot/obstacles)
































