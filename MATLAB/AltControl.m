function [new_Js, new_Je, dJw] = AltControl(ShoulderJ, ElbowJ, L1, L2, var)
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

    
    %% Altitude Control

    Altitude.new = var;

    if Altitude.new > Altitude.max
        disp('Error: cannot be more than Max Altitude')
        Altitude.new = Altitude.max;
    elseif Altitude.new < Altitude.min
        disp('Error: cannot be less than Min Altitude')
        Altitude.new = Altitude.min;
    end

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
end