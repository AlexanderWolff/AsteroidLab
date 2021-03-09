%% Global Pose Sandbox
% Plot both robots in a global frame (origin is base of UR3)


sim1 = Simulation('UR3');
sim2 = Simulation('UR5');

offset = Homogeneous.fromT([-1480.4e-3,0,0]);
offset.setR(RMatrix.fromXYZ(deg2rad([0,0,180])).R);
sim2.Set_Base_Offset(offset);

sim1.ForwardKinematics([0,0,0,0,0,0])
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
