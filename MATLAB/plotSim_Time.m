function plotSim_Time(sim, Target)





UR3_r = 0.064;
table_length = 1.99;
l = table_length/2;

table_width = 0.799;
w = table_width/2;

offset_lengthwise = l-UR3_r-0.182;
offset_widthwise = w-UR3_r-0.025;

table = [ l-offset_lengthwise, l-offset_lengthwise,-l-offset_lengthwise,-l-offset_lengthwise;...
          w-offset_widthwise,-w-offset_widthwise,-w-offset_widthwise, w-offset_widthwise]';



%% Plot
F1 = sim.transform.local{1};
F2 = sim.transform.local{2};
F3 = sim.transform.local{3};
F4 = sim.transform.local{4};
F5 = sim.transform.local{5};
F6 = sim.transform.local{6};
F7 = sim.transform.local{7};
FTool = sim.transform.local{8};
scale = 0.7;

%% Workspace
rx = abs(sim.parameters.workspace(1,1) - sim.parameters.workspace(1,2))/2;
offset_x = sim.parameters.workspace(1,1)+rx;
ry = abs(sim.parameters.workspace(2,1) - sim.parameters.workspace(2,2))/2;
offset_y = sim.parameters.workspace(2,1)+ry;
rz = abs(sim.parameters.workspace(3,1) - sim.parameters.workspace(3,2))/2;
offset_z = sim.parameters.workspace(3,1)+rz;
   

%% Plot 3D Local 
subplot(2,2,1)
hold on;

text(F1.T(1),F1.T(2)+scale*0.1,F1.T(3)+scale*0.1,strcat(sim.robot_class,' Base'))

plot3([F1.T(1),F2.T(1)],[F1.T(2),F2.T(2)],[F1.T(3),F2.T(3)],'-ok',...
    'MarkerEdgeColor',[1,0,0],'LineWidth',10);

plot3([F2.T(1),F3.T(1)],[F2.T(2),F3.T(2)],[F2.T(3),F3.T(3)],'-ok',...
    'MarkerEdgeColor',[1,0,0],'LineWidth',5);

plot3([F3.T(1),F4.T(1)],[F3.T(2),F4.T(2)],[F3.T(3),F4.T(3)],'-ok',...
    'MarkerEdgeColor',[1,0,0],'LineWidth',5);

plot3([F4.T(1),F5.T(1)],[F4.T(2),F5.T(2)],[F4.T(3),F5.T(3)],'-ok',...
    'MarkerEdgeColor',[1,0,0],'LineWidth',3);

plot3([F5.T(1),F6.T(1)],[F5.T(2),F6.T(2)],[F5.T(3),F6.T(3)],'-ok',...
    'MarkerEdgeColor',[1,0,0],'LineWidth',3);

plot3([F6.T(1),F7.T(1)],[F6.T(2),F7.T(2)],[F6.T(3),F7.T(3)],'-ok',...
    'MarkerEdgeColor',[1,0,0],'LineWidth',3);

plot3([F7.T(1),FTool.T(1)],[F7.T(2),FTool.T(2)],[F7.T(3),FTool.T(3)],'-ok',...
    'MarkerEdgeColor',[0,0,1],'LineWidth',1);

FTool.plot

scale = 0.2;
Z = Homogeneous.fromT([0,0,scale]);
Z = Z.transform(FTool);
plot3([FTool.T(1),Z.T(1)],[FTool.T(2),Z.T(2)],[FTool.T(3),Z.T(3)],'-ob',...
    'MarkerEdgeColor',[0,0,1],'LineWidth',1,'MarkerSize',5)
Z = Homogeneous.fromT([scale,0,0]);
Z = Z.transform(FTool);
plot3([FTool.T(1),Z.T(1)],[FTool.T(2),Z.T(2)],[FTool.T(3),Z.T(3)],'-xb',...
    'MarkerEdgeColor',[0,0,1],'LineWidth',1,'MarkerSize',5)

% Target
scale = 0.2;
Z = Homogeneous.fromT([0,0,scale]);
Z = Z.transform(Target);
text(Target.T(1),Target.T(2),Target.T(3)-scale*0.1, 'Target');
plot3(Target.T(1),Target.T(2),Target.T(3),'x',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);
plot3(Target.T(1),Target.T(2),Target.T(3),'o',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);
plot3([Target.T(1),Z.T(1)],[Target.T(2),Z.T(2)],[Target.T(3),Z.T(3)],'-om',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',5);
Z = Homogeneous.fromT([scale,0,0]);
Z = Z.transform(Target);
plot3([Target.T(1),Z.T(1)],[Target.T(2),Z.T(2)],[Target.T(3),Z.T(3)],'-xm',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',5)

% Draw Table
z = zeros(length(table));
c = ones(size(z));
surf(table(:,1),table(:,2),z,'FaceColor',[0.9 0.9 1])


xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([sim.parameters.workspace(1,1),sim.parameters.workspace(1,2)]);
ylim([sim.parameters.workspace(2,1),sim.parameters.workspace(2,2)]);
rz = abs(sim.parameters.workspace(3,1) - sim.parameters.workspace(3,2))/2;
offset_z = sim.parameters.workspace(3,1)+rz;
zlim([offset_z,rz-offset_z]);
view(40,20);


%% Plot X Plane (Y and Z)
subplot(2,2,2);
hold on
%sim.lDisplay_plane('X');

d = scale*0.1;
plot(F1.T(2),F1.T(3),'xr','MarkerSize',10);
text(F1.T(2),F1.T(3)+d,'Base')

plot([F1.T(2),F2.T(2)],[F1.T(3),F2.T(3)],'-or','LineWidth',1);
plot([F2.T(2),F3.T(2)],[F2.T(3),F3.T(3)],'-or','LineWidth',1);
plot([F3.T(2),F4.T(2)],[F3.T(3),F4.T(3)],'-or','LineWidth',1);
plot([F4.T(2),F5.T(2)],[F4.T(3),F5.T(3)],'-or','LineWidth',1);
plot([F5.T(2),F6.T(2)],[F5.T(3),F6.T(3)],'-or','LineWidth',1);
plot([F6.T(2),F7.T(2)],[F6.T(3),F7.T(3)],'-or','LineWidth',1);
plot([F7.T(2),FTool.T(2)],[F7.T(3),FTool.T(3)],'-or','LineWidth',1);

plot(FTool.T(2),FTool.T(3),'xb','MarkerSize',10);
text(FTool.T(2),FTool.T(3)+d,'Tool')

% Plot Table
plot([min(table(:,2)),max(table(:,2))],[0,0],':k');

% Plot reachable workspace
x = linspace(-pi,pi,1000);
plot(ry*sin(x)-offset_y,rz*cos(x)-offset_z,'--b')

% Plot Target
text(Target.T(2),Target.T(3)-scale*0.1, 'Target');
plot(Target.T(2),Target.T(3),'x',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);
plot(Target.T(2),Target.T(3),'o',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);

set(gca,'XTick',[], 'YTick', [])
title('X Plane')
xlabel('Y')
ylabel('Z')
xlim([-ry,ry]);
ylim([-rz,rz]);
axis equal;

%% Plot Y Plane (X and Z)
subplot(2,2,3);

%sim.lDisplay_plane('Y');
hold on
d = scale*0.1;
plot(F1.T(1),F1.T(3),'xr','MarkerSize',10);
text(F1.T(1),F1.T(3)+d,'Base')

plot([F1.T(1),F2.T(1)],[F1.T(3),F2.T(3)],'-or','LineWidth',1);
plot([F2.T(1),F3.T(1)],[F2.T(3),F3.T(3)],'-or','LineWidth',1);
plot([F3.T(1),F4.T(1)],[F3.T(3),F4.T(3)],'-or','LineWidth',1);
plot([F4.T(1),F5.T(1)],[F4.T(3),F5.T(3)],'-or','LineWidth',1);
plot([F5.T(1),F6.T(1)],[F5.T(3),F6.T(3)],'-or','LineWidth',1);
plot([F6.T(1),F7.T(1)],[F6.T(3),F7.T(3)],'-or','LineWidth',1);
plot([F7.T(1),FTool.T(1)],[F7.T(3),FTool.T(3)],'-or','LineWidth',1);

plot(FTool.T(1),FTool.T(3),'xb','MarkerSize',10);
text(FTool.T(1),FTool.T(3)+d,'Tool')

% Plot Table
plot([min(table(:,1)),max(table(:,1))],[0,0],':k');

% Plot reachable workspace
x = linspace(-pi,pi,1000);
plot(rx*sin(x)-offset_x,rz*cos(x)-offset_z,'--b')

% Plot Target
text(Target.T(1),Target.T(3)-scale*0.1, 'Target');
plot(Target.T(1),Target.T(3),'x',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);
plot(Target.T(1),Target.T(3),'o',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);

title('Y Plane')
set(gca,'XTick',[], 'YTick', [])
xlabel('X')
ylabel('Z')
xlim([-rx,rx]);
ylim([-rz,rz]);
axis equal;

%% Plot Z Plane (X and Y)

subplot(2,2,4);
%sim.lDisplay_plane('Z');

hold on
d = scale*0.1;
plot(F1.T(1),F1.T(2),'xr','MarkerSize',10);
text(F1.T(1),F1.T(2)+d,'Base')

plot([F1.T(1),F2.T(1)],[F1.T(2),F2.T(2)],'-or','LineWidth',1);
plot([F2.T(1),F3.T(1)],[F2.T(2),F3.T(2)],'-or','LineWidth',1);
plot([F3.T(1),F4.T(1)],[F3.T(2),F4.T(2)],'-or','LineWidth',1);
plot([F4.T(1),F5.T(1)],[F4.T(2),F5.T(2)],'-or','LineWidth',1);
plot([F5.T(1),F6.T(1)],[F5.T(2),F6.T(2)],'-or','LineWidth',1);
plot([F6.T(1),F7.T(1)],[F6.T(2),F7.T(2)],'-or','LineWidth',1);
plot([F7.T(1),FTool.T(1)],[F7.T(2),FTool.T(2)],'-or','LineWidth',1);

plot(FTool.T(1),FTool.T(2),'xb','MarkerSize',10);
text(FTool.T(1),FTool.T(2)+d,'Tool')

% Plot Table
plot([table(:,1);table(1,1)],[table(:,2);table(1,2)],':k');

% Plot reachable workspace
x = linspace(-pi,pi,1000);
plot(rx*sin(x)-offset_x,ry*cos(x)-offset_y,'--b')

% Plot Target
text(Target.T(1),Target.T(2)-scale*0.1, 'Target');
plot(Target.T(1),Target.T(2),'x',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);
plot(Target.T(1),Target.T(2),'o',...
    'MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);

title('Z Plane')
set(gca,'XTick',[], 'YTick', [])
xlabel('X')
ylabel('Y')
xlim([-rx,rx]);
ylim([-ry,ry]);

end