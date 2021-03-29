function plotJoints_Time(sim, Target, joint_history)

    scale = 0.5;
    margin = 20;
    xpos = 400;
    ypos = 600;
    figure('Position', [xpos ypos scale*1000+margin*6 scale*1000+margin*6],...
           'name', 'Target is Within Workspace');


    sim.ForwardKinematics(joint_history(end,:))
    plotSim_Time(sim, Target);
    subplot(2,2,1)
    title('Finishing Pose')

    i = 1;
    
    uicontrol('style','slider','Min',1,'Max',length(joint_history),...
        'SliderStep',[1/(length(joint_history)-1),1/(length(joint_history)-1)],...
        'units','normalized',...
        'value',length(joint_history),...
        'position',[0 0 1 0.05],'callback',@hscroll_Callback);
    

    
    function hscroll_Callback(src,~)
        
        i = round(get(src,'value'));
        
        cla(subplot(2,2,1))
        cla(subplot(2,2,2))
        cla(subplot(2,2,3))
        cla(subplot(2,2,4))
        sim.ForwardKinematics(joint_history(i,:))
        plotSim_Time(sim, Target);
        subplot(2,2,1)
        xlim([sim.parameters.workspace(1,1),sim.parameters.workspace(1,2)]);
        ylim([sim.parameters.workspace(2,1),sim.parameters.workspace(2,2)]);
        rz = abs(sim.parameters.workspace(3,1) - sim.parameters.workspace(3,2))/2;
        offset_z = sim.parameters.workspace(3,1)+rz;
        zlim([offset_z,rz-offset_z]);
        title(sprintf('Iteration: %i',i));
    end
end

