
%% Example

% L1 = Link('d', 0, 'a', 1, 'alpha', pi/2);
% 
% %For a given joint angle, say q=0.2 rad, we can determine the link transform
% % matrix
% %L1.A(0.2)
% 
% L2 = Link('d', 0, 'a', 1, 'alpha', 0);
% 
% % Now we need to join these into a serial-link robot manipulator
% bot = SerialLink([L1 L2], 'name', 'my robot');
% 
% %Given the joint angles q1 = 0.1 and q2 = 0.2 we can determine the pose of the
% % robot's end-effector
% bot.fkine([0.1 0.2])

% Display
%bot.plot([0.1 0.2])

%% UR3 Parameters

% theta (rad), a (m), d(m), alpha (rad)
UR3_parameters_kinematics = [0,        0, 0.15190, pi/2; ...
                             0, -0.24365,       0,    0; ...
                             0, -0.21325,       0,    0; ...
                             0,        0, 0.11235, pi/2; ...
                             0,        0, 0.08535,-pi/2; ...
                             0,        0, 0.08190,    0];
                         
% mass (kg), centre of mass (m)
UR3_parameters_dynamics = { 2.00, [   0, -0.02,      0]; ...
                            3.42, [0.13,     0, 0.1157]; ...
                            1.26, [0.05,     0, 0.0238]; ...
                            0.80, [   0,     0,   0.01]; ...
                            0.80, [   0,     0,   0.01]; ...
                            0.35, [   0,     0,  -0.02]};
                        
                        
%% UR5 Parameters

% theta (rad), a (m), d(m), alpha (rad)
UR5_parameters_kinematics = [0,       0, 0.089159, pi/2; ...
                             0,-0.42500,        0,    0; ...
                             0,-0.39225,        0,    0; ...
                             0,       0, 0.109150, pi/2; ...
                             0,       0, 0.094650,-pi/2; ...
                             0,       0, 0.082300,    0];

% mass (kg), centre of mass (m)
UR5_parameters_dynamics = { 3.7000, [      0,-0.02561, 0.001930]; ...
                            8.3930, [0.21255,       0, 0.113360]; ...
                            2.3300, [0.15000,       0, 0.026500]; ...
                            1.2190, [      0,-0.00180, 0.016340]; ...
                            1.2190, [      0, 0.00180, 0.016340]; ...
                            0.1879, [      0,       0,-0.001159]};
                        
%% UR3 Test

P = UR3_parameters_kinematics;
L = cell(1,6);

for i = 1:6
    L{i} = Link('d', P(i,3), 'a', P(i,2), 'alpha', P(i,4));
end

UR3_sim = SerialLink([L{1},L{2},L{3},L{4},L{5},L{6}], 'name', 'UR3');

%UR3_robot.fkine([0.1, 0.2, 0, 0, 0, 0])
%example_pose = [-0.41, -90.22, 25.79, -90.43, 0.62, 0.24];

%UR3_robot.plot(UR3.robot_packets{end-1}.info{2}')
% setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04];
% setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04];

start_pos = UR3.Get_Joint_Pos();
start_tcp = UR3.Get_TCP_Pos();

%UR3_sim.plot(start_pos')


%% Heart Curve
% scale in m
scale = 0.1;

t = linspace(0,2*pi,20);
x = 16*(sin(t)).^3;
y = 13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t);
x = (x/max(x))*scale;
y = (y/max(y))*scale;
%figure
%plot(x,y,'-',x,y, 'x')

tcp = UR3_sim.fkine( start_pos );
s_tcp_t = tcp.t';
s_tcp_r = [tcp.n, tcp.o, tcp.a];
s_tcp_a = rotationMatrixToVector(s_tcp_r');
s_tcp = [s_tcp_t, s_tcp_a]';

%disp('Ground Truth, Reconstructed, Error')
%[start_tcp, s_tcp, abs(start_tcp-s_tcp)]

% Orientation Error
actual_xyz = RMatrix.fromAngleAxisMag(start_tcp(4:6)).toXYZ;
sim_xyz = s_tcp_a;
error = sim_xyz-actual_xyz;
orientation_actual_sim_error_rad = [actual_xyz;sim_xyz;error]

% Position Error
actual_xyz = start_tcp(1:3)';
sim_xyz = s_tcp_t;
error = sim_xyz-actual_xyz;
position_actual_sim_error_rad = [actual_xyz;sim_xyz;error]

% t = [0:.056:2]; 	% generate a time vector
% q = jtraj(qz, qr, t); % compute the joint coordinate trajectory
% T = p560.fkine(q);
% p = T.transl;
% qi = p560.ikine(T);
% qi = p560.ikine(T, 'pinv');
% qi = p560.ikine6s(T)
% p560.fkine(qi)
% p560.ikine6s(T, 'rdf')
% 
% % trajectory
% T1 = transl(0.6, -0.5, 0.0) % define the start point
% T2 = transl(0.4, 0.5, 0.2)	% and destination
% T = ctraj(T1, T2, 50) 	% compute a Cartesian path
% 
% % Find Trajectory inverse k
% q = p560.ikine6s(T); 
% clf
% p560.plot(q)





























