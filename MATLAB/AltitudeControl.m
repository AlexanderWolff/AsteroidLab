function [new_Js, new_Je, dJw] = AltitudeControl(ShoulderJ, ElbowJ, L1, L2, Control, var)
    % Control = 'Joint'
    % Corrects Elbow Joint when given change in Shoulder Joint
    %
    % Control = 'Altitude'
    % Corrects Elbow and Shoulder Joints when given new altitude
    %
    % var : controlling variable (ie. new shoulder joint or new altitude)
    %
    % L1 : Length of Shoulder-Elbow
    % L2 : Length of Elbow-Wrist
    % ShoulderJ, ElbowJ in degrees

    %% Initialising
    Joint.Shoulder = deg2rad(ShoulderJ);
    Joint.Elbow    = deg2rad(ElbowJ);
    Joint.Wrist    = deg2rad(0);

    % L1 : shoulder to elbow, L2 : elbow to wrist, L3 : wrist to tip
    Length.L1 = L1;
    Length.L2 = L2;
    Length.L3 = 0.1;

    % Points
    Point.Shoulder.x = 0;
    Point.Shoulder.y = 0;

    Point.Elbow.x = Point.Shoulder.x + Length.L1 * cos( Joint.Shoulder );
    Point.Elbow.y = Point.Shoulder.y + Length.L1 * sin( Joint.Shoulder );

    Point.Wrist.x = Point.Elbow.x + Length.L2 * cos( Joint.Elbow + Joint.Shoulder );
    Point.Wrist.y = Point.Elbow.y + Length.L2 * sin( Joint.Elbow + Joint.Shoulder );

    Point.Tip.x = Point.Wrist.x + Length.L3 * cos( Joint.Wrist + Joint.Elbow + Joint.Shoulder );
    Point.Tip.y = Point.Wrist.y + Length.L3 * sin( Joint.Wrist + Joint.Elbow + Joint.Shoulder );


    %% Guide

    % y = m*x + c (but centred at origin so c = 0)
    Guide.m = -1;

    % Let initial Guide be where Wrist and Shoulder meet
    dx = Point.Shoulder.x - Point.Wrist.x;
    dy = Point.Shoulder.y - Point.Wrist.y;
    Guide.m = dy/dx;

    %Guide Tangent
    Guide.m_perp = -1/Guide.m;

    %% START

    % Start Altitude
    dx = Point.Shoulder.x - Point.Wrist.x;
    dy = Point.Shoulder.y - Point.Wrist.y;
    Altitude.start = sqrt(dx^2 + dy^2 );

    Start.Length.A = Length.L2;
    Start.Length.B = Altitude.start;
    Start.Length.C = Length.L1;

    A = Start.Length.A;
    B = Start.Length.B;
    C = Start.Length.C;

    a = acos( (B^2 + C^2 - A^2)/(2*B*C) ); 
    b = acos( (A^2 + C^2 - B^2)/(2*A*C) );
    c = acos( (A^2 + B^2 - C^2)/(2*A*B) );

    Start.Angle.a = a;
    Start.Angle.b = b;
    Start.Angle.c = c;

    % Triangle A C D
    A = Length.L3;

    dx = Point.Shoulder.x - Point.Tip.x;
    dy = Point.Shoulder.y - Point.Tip.y;
    C = sqrt(dx^2 + dy^2 );

    D = Altitude.start;

    % Angle between guide and tip
    Start.Angle.d = pi - acos( (D^2 + A^2 - C^2)/(2*D*A) );

    Altitude.min = abs(Length.L1 - Length.L2);
    Altitude.max = Length.L1 + Length.L2;

    % Max Altitude is always when:
    % 1. Joint.Elbow = 0 AND
    % 2. Joint.Shoulder is aligned with Guide

    %% Plotting
    % Pi Sweep to plot circles
    o = linspace(-pi,pi,1000);

    figure;
    hold on;

    % Base : Shoulder
    plot( Point.Shoulder.x, Point.Shoulder.y, 'xk' );

    % Shoulder to Elbow
    plot( Point.Elbow.x, Point.Elbow.y, 'xr' );
    plot( [Point.Shoulder.x,Point.Elbow.x], [Point.Shoulder.y,Point.Elbow.y], '--r' );
    plot( Point.Shoulder.x+Length.L1 * cos( o ), Point.Shoulder.y+Length.L1 * sin( o ), ':r' );

    % Elbow to Wrist
    plot( Point.Wrist.x, Point.Wrist.y, 'xb' );
    plot( [Point.Elbow.x,Point.Wrist.x], [Point.Elbow.y,Point.Wrist.y], '--b' );
    plot( Point.Elbow.x+Length.L2 * cos( o ), Point.Elbow.y+Length.L2 * sin( o ), ':b' );

    % Joining Line
    plot( [Point.Shoulder.x,Point.Wrist.x], [Point.Shoulder.y,Point.Wrist.y], ':k' );

    % Wrist to Tip
    plot( Point.Tip.x, Point.Tip.y, 'xm' );
    plot( [Point.Wrist.x,Point.Tip.x], [Point.Wrist.y,Point.Tip.y], '--m' );

    % Guide
    x = linspace(-3,3,2);
    plot( x, Guide.m*x, 'g' );
    plot( x, Guide.m_perp*x, ':g' );

    plot( Point.Shoulder.x + Altitude.min * cos(o),Point.Shoulder.y + Altitude.min * sin(o), ':k');
    plot( Point.Shoulder.x + Altitude.max * cos(o),Point.Shoulder.y + Altitude.max * sin(o), ':k');

    %% Altitude Control
    if strcmp(Control,'Altitude')

        Altitude.new = var;

        if Altitude.new > Altitude.max
            Altitude.new = Altitude.max;
        elseif Altitude.new < Altitude.min
            Altitude.new = Altitude.min;
        end

        disp('Current Alt:'+string(Altitude.start))
        disp('Max Alt:'+string(Altitude.max))
        disp('Min Alt:'+string(Altitude.min))
        disp('Target Alt:'+string(Altitude.new))
        disp('_')

        Stop.Length.A = Length.L2;
        Stop.Length.B = Altitude.new;
        Stop.Length.C = Length.L1;

        A = Stop.Length.A;
        B = Stop.Length.B;
        C = Stop.Length.C;

        a = acos( (B^2 + C^2 - A^2)/(2*B*C) ); 
        b = acos( (A^2 + C^2 - B^2)/(2*A*C) );
        c = acos( (A^2 + B^2 - C^2)/(2*A*B) );

        Stop.Angle.a = a;
        Stop.Angle.b = b;
        Stop.Angle.c = c;

        new_Js = Joint.Shoulder + ( Start.Angle.a - Stop.Angle.a );

        new_Je = Joint.Elbow + ( Start.Angle.b - Stop.Angle.b );
    end
    %% Joint Control (Using Shoulder Joint)
    if strcmp(Control,'Joint')

        new_Js = deg2rad(23.475);
        Js = new_Js;

        %% Check if it is possible to reach Guide


        %% Find R and alpha
        m = Guide.m;
        L1 = Length.L1;
        L2 = Length.L2;

        x = - ( sin(Js)-m*cos(Js) )/( cos(Js)+m*sin(Js) );
        alpha = atan(1/x);
        R = sqrt( x^2 + 1 );


        %% Detect Quadrant relative to Guide


        Point.Elbow.x = Point.Shoulder.x + Length.L1 * cos( Js );
        Point.Elbow.y = Point.Shoulder.y + Length.L1 * sin( Js );

        if Point.Elbow.y < Point.Elbow.x * Guide.m_perp
            if Point.Elbow.x < Point.Elbow.y / Guide.m
                Quadrant = 3;
            else
                Quadrant = 4;
            end
        else
            if Point.Elbow.x < Point.Elbow.y / Guide.m
                Quadrant = 2;
            else
                Quadrant = 1;
            end
        end

        %% Jx = Je - alpha
        if Quadrant == 1 || Quadrant == 3
            Jx = acos(x*L1/L2*1/R);
            Je = Jx + alpha;
            difference = Start.Angle.b-Je;

            new_Je = Joint.Elbow+difference;

        elseif Quadrant == 2 || Quadrant == 4
                Jx = acos(x*L1/L2*1/R);
                Je = Jx + alpha;

                % Correction
                Je = pi-Je;

                difference = Start.Angle.b-Je;

                new_Je = Joint.Elbow+difference;

                % Correction
                new_Je = -1*new_Je;
        end

        disp('Current Joint:'+string(rad2deg(Joint.Elbow)))
        disp('Current B:'+string(rad2deg(b)))
        disp('Target B :'+string(rad2deg(Je)))
        disp('Target Joint difference :'+string(rad2deg(difference)))
        disp('Target Joint :'+string(rad2deg(new_Je)))
    end

    Joint.Shoulder = new_Js;
    Joint.Elbow    = new_Je;
    Joint.Wrist    = Joint.Wrist;

    %% STOP
    % Points
    Point.Elbow.x = Point.Shoulder.x + Length.L1 * cos( Joint.Shoulder );
    Point.Elbow.y = Point.Shoulder.y + Length.L1 * sin( Joint.Shoulder );

    Point.Wrist.x = Point.Elbow.x + Length.L2 * cos( Joint.Elbow + Joint.Shoulder );
    Point.Wrist.y = Point.Elbow.y + Length.L2 * sin( Joint.Elbow + Joint.Shoulder );

    Point.Tip.x = Point.Wrist.x + Length.L3 * cos( Joint.Wrist + Joint.Elbow + Joint.Shoulder  );
    Point.Tip.y = Point.Wrist.y + Length.L3 * sin( Joint.Wrist + Joint.Elbow + Joint.Shoulder  );

    % End Altitude
    dx = Point.Shoulder.x - Point.Wrist.x;
    dy = Point.Shoulder.y - Point.Wrist.y;
    Altitude.end = sqrt(dx^2 + dy^2 );

    Stop.Length.A = Length.L2;
    Stop.Length.B = Altitude.end;
    Stop.Length.C = Length.L1;

    A = Stop.Length.A;
    B = Stop.Length.B;
    C = Stop.Length.C;

    a = acos( (B^2 + C^2 - A^2)/(2*B*C) ); 
    b = acos( (A^2 + C^2 - B^2)/(2*A*C) );
    c = acos( (A^2 + B^2 - C^2)/(2*A*B) );

    Stop.Angle.a = a;
    Stop.Angle.b = b;
    Stop.Angle.c = c;

    % Triangle A C D
    A = Length.L3;

    dx = Point.Shoulder.x - Point.Tip.x;
    dy = Point.Shoulder.y - Point.Tip.y;
    C = sqrt(dx^2 + dy^2 );

    D = Altitude.end;

    % Angle between guide and tip
    Stop.Angle.d = pi - acos( (D^2 + A^2 - C^2)/(2*D*A) );

    % Difference
    dJw = real(Start.Angle.d - Stop.Angle.d);

    new_Jw = Joint.Wrist + dJw;

    Joint.Wrist    = new_Jw;

    Point.Tip.x = Point.Wrist.x + Length.L3 * cos( Joint.Wrist + Joint.Elbow + Joint.Shoulder  );
    Point.Tip.y = Point.Wrist.y + Length.L3 * sin( Joint.Wrist + Joint.Elbow + Joint.Shoulder  );

    %% Plotting
    % Base : Shoulder
    plot( Point.Shoulder.x, Point.Shoulder.y, 'xk' );

    % Shoulder to Elbow
    plot( Point.Elbow.x, Point.Elbow.y, 'xk' );
    plot( [Point.Shoulder.x,Point.Elbow.x], [Point.Shoulder.y,Point.Elbow.y], 'r' );

    % Elbow to Wrist
    plot( Point.Wrist.x, Point.Wrist.y, 'xk' );
    plot( [Point.Elbow.x,Point.Wrist.x], [Point.Elbow.y,Point.Wrist.y], 'b' );

    % Wrist to Tip
    plot( Point.Tip.x, Point.Tip.y, 'xk' );
    plot( [Point.Wrist.x,Point.Tip.x], [Point.Wrist.y,Point.Tip.y], 'm' );


    % Text
    text(-1, -1.2, strcat('start alt: ',string(Altitude.start)));
    text(-1, -1.3, strcat('end  alt: ',string(Altitude.end)));

    text(Point.Shoulder.x+0.05, Point.Shoulder.y-0.05, 'a');
    text(Point.Elbow.x+0.05, Point.Elbow.y-0.05, 'b');
    text(Point.Wrist.x+0.05, Point.Wrist.y-0.05, 'c');

    text(0, -1.2, strcat('start  a: ',string(rad2deg(Start.Angle.a))));
    text(0, -1.3, strcat('start  b: ',string(rad2deg(Start.Angle.b))));
    text(0, -1.4, strcat('start  c: ',string(rad2deg(Start.Angle.c))));

    text(1, -1.2, strcat('stop  a: ',string(rad2deg(Stop.Angle.a))));
    text(1, -1.3, strcat('stop  b: ',string(rad2deg(Stop.Angle.b))));
    text(1, -1.4, strcat('stop  c: ',string(rad2deg(Stop.Angle.c))));

    L = Length.L1+Length.L2;
    s = 1.0;
    xlim([-s*L,s*L]);
    ylim([-s*L,s*L]);
    axis equal;

end