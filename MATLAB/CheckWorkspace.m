
function [valid, constraint] = CheckWorkspace(this, Target)

    bot_margin = this.parameters.bot_margin;


    %% Check that Target is within Workspace
    % workspace is a sphere offset at Joint 2
    workspace.radius = 0.5*abs(mean(this.parameters.workspace(1:3,1) -  this.parameters.workspace(1:3,2)));
    workspace.offset = -1*0.5*(this.parameters.workspace(1:3,1) +  this.parameters.workspace(1:3,2));

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

    %% determine if point + orientation is in operational workspace

    if distance <= operational_radius
        constraint.operational = true;
    else
        constraint.operational = false;
    end

    %% reduce workspace with further constraints (eg. table)

    % create a cube (6 faces) which the target has to be within
    cube.bot = 0 + bot_margin;
    cube.top =  workspace.offset(3)+workspace.radius;


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

    if constraint.top &&  constraint.bot 
        constraint.allowed = true;
    else
        constraint.allowed = false;
    end

    if constraint.reachable && constraint.operational && constraint.allowed
        constraint.all = true;
    else
        constraint.all = false;
    end

    valid = constraint.all;
end