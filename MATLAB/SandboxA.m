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

sim1 = Simulation('UR3');
sim2 = Simulation('UR5');

offset = Homogeneous.fromT([-1480.4e-3,0,0]);
offset.setR(RMatrix.fromXYZ(deg2rad([0,0,180])).R);
sim2.Set_Base_Offset(offset);

sim1.ForwardKinematics([0,-90,0,-90,0,0])
sim2.ForwardKinematics([0,-90,0,-90,0,0])

figure('name', 'Robots in Physical Space');
hold on;

sim1.gDisplay;
sim2.gDisplay;

xlim([-2,2]);
axis equal;

tool1_F0 = sim1.transform.local{7};
tool2_local = sim2.transform.local{7};
tool2_F0 = tool2_local.transform(offset);
figure('name', 'Targets in Global Frame (base of UR3)');
hold on;
tool1_F0.plotL('Asteroid');
tool2_F0.plotL('Camera');
xlim([-2,2]);
axis equal;




























