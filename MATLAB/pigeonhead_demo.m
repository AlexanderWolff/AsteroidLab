

sim = Simulation('UR3');

% Initialise Pose
joints = randi(360,1,6)-180%[0,-60,60,10,60,40];
sim.ForwardKinematics(joints);
Target = Homogeneous.fromT([-0.2,-0.3,0.4] );
plotSim(sim, Target, '');

%% Pigeonhead Motion
% Moving Joint 0 will rotate wrist to maintain orientation


% plot sphere around target (radius = frame 5 workspace)

A = sim.transform.local{5}.T;
B = sim.transform.local{8}.T;

dx = abs(B(1) - A(1));
dy = abs(B(2) - A(2));
dz = abs(B(3) - A(3));
distance = sqrt(dx^2 + dy^2 + dz^2);

disp(distance)


subplot(2,2,1);
plot3([A(1), B(1)],[A(2), B(2)],[A(3), B(3)], ':r')

return

% plot vector showing right ascension and declination
% start at target , stop at edge of sphere


% plot vector between edge of sphere and robot origin (joint 2)


% change right ascension/declination


% find change in where sphere intersects and therefore sphere-robot origin 
% vector new length and change in declination/right ascension







%% Begin Inverse Kinematics


history = 10;
joint_history = zeros(history,length(joints));
joint_history(1,:) = sim.joints.deg;



disp('START')
for i = 2:history
    
    % Current Joints
    j1 = joint_history(i-1,1);
    j2 = joint_history(i-1,2);
    j3 = joint_history(i-1,3);
    j4 = joint_history(i-1,4);
    j5 = joint_history(i-1,5);
    j6 = joint_history(i-1,6);

    %% J4-J3-J2 Triangle
    F2 = sim.transform.local{2};
    F3 = sim.transform.local{3};
    A = sim.transform.local{4};

    J2 = F2.T;
    J3 = F3.T;
    J4 = A.T;
    
    % L1 : J2 and J3
    L1 = sqrt( (J2(1)-J3(1))^2 + (J2(2)-J3(2))^2 + (J2(3)-J3(3))^2 );

    % L2 : J3 and J4
    L2 = sqrt( (J3(1)-J4(1))^2 + (J3(2)-J4(2))^2 + (J3(3)-J4(3))^2 );

    % Shoulder : J2
    Js = j2;

    % Elbow : J3
    Je = j3;

    % Current Altitude of Wrist : Distance J2 to J4
    Altitude.wrist = sqrt( (J2(1)-J4(1))^2 + (J2(2)-J4(2))^2 + (J2(3)-J4(3))^2 );
     
    % Altitude of Tool
    B = sim.transform.local{8};
    J8 = B.T;
    Altitude.tool = sqrt( (J2(1)-J8(1))^2 + (J2(2)-J8(2))^2 + (J2(3)-J8(3))^2 );
    
    % Altitude of Target
    T = Target.T;
    Altitude.target = sqrt( (J2(1)-T(1))^2 + (J2(2)-T(2))^2 + (J2(3)-T(3))^2 );

    % Difference in Altitude
    Altitude.diff = Altitude.target - Altitude.tool;
    
    % New altitude of wrist
    Altitude.new = Altitude.wrist + Altitude.diff; 

    disp(Altitude)
    
    %[new_Js, new_Je, dJw] = AltitudeControl(Js, Je, L1, L2, 'Altitude', Altitude.new);
    [new_Js, new_Je, dJw] = AltControl(Js, Je, L1, L2, Altitude.new);
    
    new_j2 = rad2deg(new_Js);
    new_j3 = rad2deg(new_Je);
    
    new_j4 = j4 + rad2deg(dJw);
    joint_history(i,:) = [j1, new_j2, new_j3, new_j4, j5, j6];
    sim.ForwardKinematics([j1, new_j2, new_j3, new_j4, j5, j6]);
end


plotJoints_Time(sim, Target, joint_history)