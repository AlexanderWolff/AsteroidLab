UR3_r = 0.064;
table_length = 1.99;
l = table_length/2;

table_width = 0.799;
w = table_width/2;

offset_lengthwise = l-UR3_r-0.182;
offset_widthwise = w-UR3_r-0.025;

table = [ l-offset_lengthwise, l-offset_lengthwise,-l-offset_lengthwise,-l-offset_lengthwise;...
          w-offset_widthwise,-w-offset_widthwise,-w-offset_widthwise, w-offset_widthwise]';

sim = Simulation('UR3');

sim.ForwardKinematics([0,-90,0,-90,0,0])


F1 = sim.transform.local{1};
F2 = sim.transform.local{2};
F3 = sim.transform.local{3};
F4 = sim.transform.local{4};
F5 = sim.transform.local{5};
F6 = sim.transform.local{6};
F7 = sim.transform.local{7};
Ftool_F7 = sim.transform.tool_offset;
Ftool = Ftool_F7.transform(F7);

positions = [F1.T,F2.T,F3.T,F4.T,F5.T,F6.T,F7.T,Ftool.T]';

min_pos = min(min(positions));
max_pos = max(max(positions));

scale = max(abs([min_pos,max_pos]));

margin = 20;
xpos = 400;
ypos = 600;
figure('Position', [xpos ypos scale*1000+margin*6 scale*1000+margin*6],...
       'name', 'UR3 in Physical Space');


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

plot3([F7.T(1),Ftool.T(1)],[F7.T(2),Ftool.T(2)],[F7.T(3),Ftool.T(3)],'-ok',...
    'MarkerEdgeColor',[0,0,1],'LineWidth',1);

Ftool.plot

% Draw Table
z = zeros(length(table));
c = ones(size(z));
surf(table(:,1),table(:,2),z,'FaceColor',[0.9 0.9 1])


xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-scale*1.1,scale*1.1]);
ylim([-scale*1.1,scale*1.1]);
zlim([-scale*0.1,scale*1.1]);
axis equal;

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
plot([F7.T(2),Ftool.T(2)],[F7.T(3),Ftool.T(3)],'-or','LineWidth',1);

plot(Ftool.T(2),Ftool.T(3),'xb','MarkerSize',10);
text(Ftool.T(2),Ftool.T(3)+d,'Tool')

% Plot Table
plot([min(table(:,2)),max(table(:,2))],[0,0],':k');

set(gca,'XTick',[], 'YTick', [])
title('X Plane')
xlabel('Y')
ylabel('Z')
xlim([-scale*1.1,scale*1.1]);
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
plot([F7.T(1),Ftool.T(1)],[F7.T(3),Ftool.T(3)],'-or','LineWidth',1);

plot(Ftool.T(1),Ftool.T(3),'xb','MarkerSize',10);
text(Ftool.T(1),Ftool.T(3)+d,'Tool')

% Plot Table
plot([min(table(:,1)),max(table(:,1))],[0,0],':k');

title('Y Plane')
set(gca,'XTick',[], 'YTick', [])
xlabel('X')
ylabel('Z')
xlim([-scale*1.1,scale*1.1]);
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
plot([F7.T(1),Ftool.T(1)],[F7.T(2),Ftool.T(2)],'-or','LineWidth',1);

plot(Ftool.T(1),Ftool.T(2),'xb','MarkerSize',10);
text(Ftool.T(1),Ftool.T(2)+d,'Tool')

% Plot Table
plot([table(:,1);table(1,1)],[table(:,2);table(1,2)],':k');

title('Z Plane')
set(gca,'XTick',[], 'YTick', [])
xlabel('X')
ylabel('Y')
xlim([-scale*1.1,scale*1.1]);
axis equal;