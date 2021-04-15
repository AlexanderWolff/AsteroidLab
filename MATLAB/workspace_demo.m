

sim = Simulation('UR3');
joints = [0,0,0,0,90,0];
sim.ForwardKinematics(joints);


%% Target
Target = Homogeneous.fromT(((rand(1,3)-0.5)));
random = rad2deg((rand(1,3)-0.5)*2*pi);
Target.setR(RMatrix.fromXYZ(random).R);

if ~sim.CheckWorkspace(Target)
    return
end

figure;
hold on;
[~, constraint] = sim.CheckWorkspace(Target);
disp(constraint)

sim.DisplayWorkspace;
sim.DisplayTarget(Target);
plotRobot(sim)

%% determine whether each robot joints are within allowed workspace
if sim.CheckPose
    disp('Valid Pose')
else
    disp('Invalid Pose')
end


%% determine whether robot is intersecting with itself
% for each link find the vector describing it (joint a to joint b)
% find the vector of {parallel vector to above} to target joint
% thus find distance between target joint and link vector
% limit search to between joint a and joint b


%% determine whether robot is intersecting with other robot