function constraint = DisplayWorkspace(this, Target)
    
    bot_margin = this.parameters.bot_margin;


    %% Check that Target is within Workspace

    %% plot reachable workspace boundaries

    % workspace is a sphere offset at Joint 2
    workspace.radius = 0.5*abs(mean(this.parameters.workspace(1:3,1) -  this.parameters.workspace(1:3,2)));
    workspace.offset = -1*0.5*(this.parameters.workspace(1:3,1) +  this.parameters.workspace(1:3,2));

    [reach.X,reach.Y,reach.Z] = sphere(10);
    reach.X = (reach.X * workspace.radius) + workspace.offset(1);
    reach.Y = (reach.Y * workspace.radius) + workspace.offset(2);
    reach.Z = (reach.Z * workspace.radius) + workspace.offset(3);

    hold on;
    lightGrey = 0.8*[1 1 1];
    surface(reach.X, reach.Y, reach.Z, 'FaceColor', 'none','EdgeColor',lightGrey)
    axis equal

    Target.plotL('T');

    %% determine whether target is in reachable workspace

    % find distance between Target and workspace offset
    dx = Target.T(1) - workspace.offset(1);
    dy = Target.T(2) - workspace.offset(2);
    dz = Target.T(3) - workspace.offset(3);

    distance = sqrt( dx^2 + dy^2 + dz^2 );

    if distance <= workspace.radius
        constraint.reachable = true;
    else
        constraint.reachable = false;
    end

    %% plot operational workspace boundaries

    % target sphere of influence radius is sim.parameters.tool_radius
    operational_radius = workspace.radius-this.parameters.tool_radius;

    [operate.X,operate.Y,operate.Z] = sphere(20);
    operate.X = (operate.X * operational_radius ) + workspace.offset(1);
    operate.Y = (operate.Y * operational_radius ) + workspace.offset(2);
    operate.Z = (operate.Z * operational_radius ) + workspace.offset(3);

    lightBlue = 0.9*[.7 .7 1];
    surface(operate.X, operate.Y, operate.Z, 'FaceColor', 'none','EdgeColor',lightBlue)

    %% determine if point + orientation is in operational workspace

    if distance <= operational_radius
        constraint.operational = true;
    else
        constraint.operational = false;
    end

    if constraint.operational
        target.color = 0.9*[.7 1 .7];
    else
        if constraint.reachable
            target.color = 0.9*[.7 .7 1];
        else
            target.color = 0.9*[1 .7 .7];
        end
    end

    % sphere is centred along the vector of target-workspace offset
    % at one radius away from the target
    [target.X,target.Y,target.Z] = sphere(10);
    target.X = (target.X * this.parameters.tool_radius) + Target.T(1);
    target.Y = (target.Y * this.parameters.tool_radius) + Target.T(2);
    target.Z = (target.Z * this.parameters.tool_radius) + Target.T(3);

    surface(target.X, target.Y, target.Z, 'FaceColor', 'none','EdgeColor',target.color)

    %% reduce workspace with further constraints (eg. table)

    % create a cube (6 faces) which the target has to be within
    cube.bot = 0 + bot_margin;
    cube.top =  workspace.offset(3)+workspace.radius;
    cube.sid = workspace.radius*2;

    cube.squ = [ cube.sid/2, cube.sid/2;...
                 cube.sid/2,-cube.sid/2;...
                -cube.sid/2,-cube.sid/2;...
                -cube.sid/2, cube.sid/2;...
                 cube.sid/2, cube.sid/2;...
                 ];


    %% determine whether target is within allowed workspace
    if Target.T(3) >= cube.bot
        constraint.bot = true;
    else
        constraint.bot = false;
    end

    if Target.T(3) <= cube.top
        constraint.top = true;
    else
        constraint.top = false;
    end

    if constraint.bot
        bot.color = 0.9*[.7 1 .7];
    else
        bot.color = 0.9*[1 .7 .7];
    end

    if constraint.top
        top.color = 0.9*[.7 1 .7];
    else
        top.color = 0.9*[1 .7 .7];
    end

    if constraint.top &&  constraint.bot 
        constraint.allowed = true;
    else
        constraint.allowed = false;
    end

    surf(cube.squ(:,1),...
         cube.squ(:,2),...
         cube.bot*ones(5,5), 'FaceColor',bot.color )

    surf(cube.squ(:,1),...
         cube.squ(:,2),...
         cube.top*ones(5,5), 'FaceColor',top.color )
     alpha(.1)
    view(45,30)

    if constraint.reachable && constraint.operational && constraint.allowed
        constraint.all = true;
    else
        constraint.all = false;
    end
end