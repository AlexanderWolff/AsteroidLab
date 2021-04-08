function plotRobot(sim)

    %% Plot
    F1 = sim.transform.local{1};
    F2 = sim.transform.local{2};
    F3 = sim.transform.local{3};
    F4 = sim.transform.local{4};
    F5 = sim.transform.local{5};
    F6 = sim.transform.local{6};
    F7 = sim.transform.local{7};
    Ftool_F7 = sim.transform.tool_offset;
    Ftool = Ftool_F7.transform(F7);

    %% Plot
    scale = 0.7;
    hold on;

    %% Plot 3D Local 
    

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

end