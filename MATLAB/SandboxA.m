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
%     H{i} = [[R{i};zeros(1,3)],[T{i};1]];
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


O = Homogeneous.Empty;

T_POtP1 = Homogeneous.fromT([-0.5,0,0]');
T_P1tP2 = Homogeneous.fromT([-0.2,0,0]');
T_P2tP3 = Homogeneous.fromT([-0.3,0,0]');

T_P1tPO = T_POtP1.inv;
T_P2tP1 = T_P1tP2.inv;
T_P3tP2 = T_P2tP3.inv;

R_O  = Homogeneous.fromR(RMatrix.fromXYZ(deg2rad([0,0,10])).R);

hold on
O.plotk();

%% P1 in O frame
% In P1 frame
P1_P1 = O;

% In O frame
P1_O = P1_P1.transform(T_P1tPO);

P1_O.plotL('P1')

%% P2 in O frame
% In P2 frame
P2_P2 = O.transform(R_O);

% In P1 frame
P2_P1 = P2_P2.transform(T_P2tP1);

% In O frame
P2_O = P2_P1.transform(P1);
P2_O.plotL('P2')



%% P3 in O frame
% In P3 frame
P3_P3 = O;

% In P2 frame
P3_P2 = P3_P3.transform(T_P3tP2);

% In P1 frame
P3_P1 = P3_P2.transform(P2_P1);

% In PO frame
P3_PO = P3_P1.transform(P1_O);
P3_PO.plotL('P3_')

% In O frame
P3_O = P3_P2.transform(P2_O);
P3_O.plotL('P3')


%%

xlim([-0.2,2]);
axis equal 






