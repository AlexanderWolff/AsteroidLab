
sim = Simulation('UR3');
joints = [0,0,0,0,90,0];
sim.ForwardKinematics(joints);


%% Plot Trajectory in Workspace
sim.DisplayWorkspace
plotRobot(sim)


demo_length = 40;
x = linspace(-pi,pi,demo_length);
plot3(0.5*sin(x)-0.4, 0.5*cos(x), 0.5*sin(x)+0.2*cos(x) )

Targets = cell(1,length(x));
for i = 1:length(x)
    
    X = x(i);
    Target = Homogeneous.fromT([0.5*sin(X)-0.4, 0.5*cos(X), 0.5*sin(X)+0.2*cos(X)]);
    
    Target.setR(RMatrix.fromXYZ([0,0, x(i)]).R);
    
    Targets{i} = Target;
    
    % Display full trajectory
    sim.DisplayTarget(Targets{i});
end

Trajectory = cell(0);
J2 = sim.transform.local{2};

%% Gather array of Targets
for i = 1:length(Targets)
    
    Target = Targets{i};
    
    % Check if each Target is Reachable
    [valid, constraint] = sim.CheckWorkspace(Target);
    
    if valid
        Trajectory{length(Trajectory)+1} = Target;
    else
        % If Reachable but not operational, force orientation (point away from J2)
        if constraint.allowed && constraint.reachable
            
            newTarget = Target;
            newTarget.setR( Target.point_against(J2) );
            
            Trajectory{length(Trajectory)+1} = newTarget;
        end
    end
    
end

% Plot Approved Trajectory
figure;
plotRobot(sim)
sim.DisplayWorkspace
for i = 1:length(Trajectory)
    sim.DisplayTarget(Trajectory{i});
end

%% Compute Each Solution
Solutions = zeros(length(Trajectory),6);
for i = 1:length(Trajectory)
    
    %joints = [0,0,0,0,90,0];
    %sim.ForwardKinematics(joints);
    [joint_history, distances] = sim.InverseK(Trajectory{i}, false);
    
    if distances(end) > 1e-3
        disp('Attempting to find Position ONLY Inverse Kinematic Solution')
        joints = [0,0,0,0,90,0];
        sim.ForwardKinematics(joints);
        [joint_history, distances] = sim.InverseK_Position(Trajectory{i}, false);
    end
    
    %plotJoints_Time(sim, Target, joint_history)
    
    Solutions(i,:) = joint_history(end,:);
end


%% Plot Robot Joint History
plotSolutions(sim, Trajectory, Solutions)


%% Check All Solutions are Allowed
Valid = true;
for i = 1:length(Solutions)
    
    Solution = Solutions(i,:);
    sim.ForwardKinematics(Solution);
    Valid = Valid && sim.CheckPose;
    
end

if Valid
    disp('All Solutions are Valid')
else
    disp('Some Solutions are Invalid')
end







