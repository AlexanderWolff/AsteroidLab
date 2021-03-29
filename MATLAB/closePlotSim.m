function closePlotSim(sim, Target)

    Target_F0 = Target;

    % Get F4, F5, F6, F7, FTool and Target in terms of F4
    F6_F5 = sim.transform.chain{6};
    F7_F6 = sim.transform.chain{7}; 
    FTool_F7 = sim.transform.chain{8}; 
    F4_F0 = sim.transform.local{4};
    F0_F4 = F4_F0.inv;

    F4_F4 = Homogeneous.Empty;
    F5_F4 = sim.transform.chain{5};
    F6_F4 = F6_F5.transform(F5_F4);
    F7_F4 = F7_F6.transform(F6_F4);
    FTool_F4 = FTool_F7.transform(F7_F4);
    Target_F4 = Target_F0.transform(F0_F4);

    scale = 0.1;
    TargetX = Homogeneous.fromT([scale,0,0]);
    TargetX = TargetX.transform(Target_F4);
    TargetZ = Homogeneous.fromT([0,0,scale]);
    TargetZ = TargetZ.transform(Target_F4);

    ToolX = Homogeneous.fromT([scale,0,0]);
    ToolX = ToolX.transform(FTool_F4);
    ToolZ = Homogeneous.fromT([0,0,scale]);
    ToolZ = ToolZ.transform(FTool_F4);

    figure();
    hold on;
    view(40,20);
    linew = 30;
    markw = 30;
    plot3([F4_F4.T(1),F5_F4.T(1)],[F4_F4.T(2),F5_F4.T(2)],[F4_F4.T(3),F5_F4.T(3)],...
        '-ok','MarkerEdgeColor',[1,0,0],'LineWidth',linew,'MarkerSize',markw);
    plot3([F5_F4.T(1),F6_F4.T(1)],[F5_F4.T(2),F6_F4.T(2)],[F5_F4.T(3),F6_F4.T(3)],...
        '-ok','MarkerEdgeColor',[1,0,0],'LineWidth',linew,'MarkerSize',markw);
    plot3([F6_F4.T(1),F7_F4.T(1)],[F6_F4.T(2),F7_F4.T(2)],[F6_F4.T(3),F7_F4.T(3)],...
        '-ok','MarkerEdgeColor',[1,0,0],'LineWidth',linew,'MarkerSize',markw);
    plot3([F7_F4.T(1),FTool_F4.T(1)],[F7_F4.T(2),FTool_F4.T(2)],[F7_F4.T(3),FTool_F4.T(3)],...
        '-ok','MarkerEdgeColor',[1,0,0],'LineWidth',linew,'MarkerSize',markw);

    FTool_F4.plot;

    plot3([FTool_F4.T(1),ToolZ.T(1)],...
          [FTool_F4.T(2),ToolZ.T(2)],...
          [FTool_F4.T(3),ToolZ.T(3)],...
          '-ob','MarkerEdgeColor',[0,0,1],'LineWidth',1,'MarkerSize',5);

    plot3([FTool_F4.T(1),ToolX.T(1)],...
          [FTool_F4.T(2),ToolX.T(2)],...
          [FTool_F4.T(3),ToolX.T(3)],...
          '-xb','MarkerEdgeColor',[0,0,1],'LineWidth',1,'MarkerSize',5)

    text(Target_F4.T(1),Target_F4.T(2),Target_F4.T(3)-scale*0.1, 'Target');
    plot3(Target_F4.T(1),Target_F4.T(2),Target_F4.T(3),...
        'x','MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);
    plot3(Target_F4.T(1),Target_F4.T(2),Target_F4.T(3),...
        'o','MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',10);

    plot3([Target_F4.T(1),TargetZ.T(1)],...
          [Target_F4.T(2),TargetZ.T(2)],...
          [Target_F4.T(3),TargetZ.T(3)],...
          '-om','MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',5);

    plot3([Target_F4.T(1),TargetX.T(1)],...
          [Target_F4.T(2),TargetX.T(2)],...
          [Target_F4.T(3),TargetX.T(3)],...
          '-xm','MarkerEdgeColor',[1,0,1],'LineWidth',1,'MarkerSize',5)


    xlim([Target_F4.T(1)-0.2,Target_F4.T(1)+0.2])
    ylim([Target_F4.T(2)-0.2,Target_F4.T(2)+0.2])
    zlim([Target_F4.T(3)-0.2,Target_F4.T(3)+0.2])
    

end