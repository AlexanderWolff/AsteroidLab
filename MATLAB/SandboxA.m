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
scale = 100;

t = linspace(0,2*pi,1000);
x = 16*(sin(t)).^3;
y = 13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t);
x = (x/max(x))*scale;
y = (y/max(y))*scale;
plot(x,y)



















