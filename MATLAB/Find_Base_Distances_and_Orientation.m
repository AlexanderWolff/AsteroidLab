function [AA, AB] = Find_Base_Distances_and_Orientation(D_, A_,p)

    if strcmp(p, 'plot')
        x =  linspace(-pi, pi, 1000);
        figure('name', 'Simulated Relative Pose');
        hold on;
    end

    %% UR3
    % Base Diameter
    d1 = 128;
    UR3_r = d1/2;

    % Base Angle
    A = deg2rad(0);

    % Base Displacement (only modelled along x axis)
    D = 0;

    if strcmp(p, 'plot')
        plot( UR3_r*sin(x)+D, UR3_r*cos(x), 'k' );
        text( D-12, 0, 'UR3');
    end
    
    % Bolt Diameter
    bd = 6.6;
    UR3_br = bd/2;

    % Distance of bolt centre from base centre
    UR3_bl = 55;

    % Angle bolt A
    bA = deg2rad(180)+A;
    UR3_A = [cos(bA)*UR3_bl+D, sin(bA)*UR3_bl];

    if strcmp(p, 'plot')
        plot( UR3_br*sin(x)+UR3_A(1), UR3_br*cos(x)+UR3_A(2), 'b' );
    end
    
   
    %% UR5
    % Base Diameter
    d2 = 149;
    UR5_r = d2/2;

    % Base Angle
    A = A_;

    % Base Displacement (only modelled along x axis)
    D = UR3_r+UR5_r+D_;

    if strcmp(p, 'plot')
        plot( UR5_r*sin(x)+D, UR5_r*cos(x), 'k' );
        text( D-12, 0, 'UR5');
    end
    % Bolt Diameter
    bd = 8.5;
    UR5_br = bd/2;

    % Distance of bolt centre from base centre
    UR5_bl = 60;

    % Angle bolt A
    bA = deg2rad(180)+A;

    % Angle bolt B
    bB = deg2rad(0)+A;

    UR5_A = [cos(bA)*UR5_bl+D, sin(bA)*UR5_bl];
    UR5_B = [cos(bB)*UR5_bl+D, sin(bB)*UR5_bl];

    if strcmp(p, 'plot')
        plot( UR5_br*sin(x)+UR5_A(1), UR5_br*cos(x)+UR5_A(2), 'b' );
        plot( UR5_br*sin(x)+UR5_B(1), UR5_br*cos(x)+UR5_B(2), 'r' );
    end
    
    % Distance UR5_A to UR5_B
    dx = abs(UR5_A(1) - UR5_B(1));
    dy = abs(UR5_A(2) - UR5_B(2));
    UR5_D = sqrt(dx^2 + dy^2);

    %% Distances Aa and Ab

    % Aa
    start = [UR3_A(1), UR3_A(2)];
    stop =  [UR5_A(1), UR5_A(2)];

    if strcmp(p, 'plot')
        plot( [start(1), stop(1)], [start(2), stop(2)], '.-b');
    end
    dx = abs(start(1)-stop(1));
    dy = abs(start(2)-stop(2));

    % Distance is taken from far edge of each bolt 
    aa = sqrt(dx^2 + dy^2)+UR3_br+UR5_br;

    % Ab
    start = [UR3_A(1), UR3_A(2)];
    stop =  [UR5_B(1), UR5_B(2)];

    if strcmp(p, 'plot')
        plot( [start(1), stop(1)], [start(2), stop(2)], '.-r');
    end
    dx = abs(start(1)-stop(1));
    dy = abs(start(2)-stop(2));

    % Distance is taken from far edge of each bolt 
    ab = sqrt(dx^2 + dy^2)+UR3_br+UR5_br;

    AA = aa;
    AB = ab;

    %% Distance Ba and Bb
    % 
    % % Ba
    % start = [UR3_B(1), UR3_B(2)];
    % stop =  [UR5_A(1), UR5_A(2)];
    % 
    % plot( [start(1), stop(1)], [start(2), stop(2)], '.-r');
    % dx = abs(start(1)-stop(1));
    % dy = abs(start(2)-stop(2));
    % 
    % % Distance is taken from far edge of each bolt 
    % ba = sqrt(dx^2 + dy^2)+UR3_br+UR5_br;
    % disp('Ba '+ string(ba));
    % 
    % % Bb
    % start = [UR3_B(1), UR3_B(2)];
    % stop =  [UR5_B(1), UR5_B(2)];
    % 
    % plot( [start(1), stop(1)], [start(2), stop(2)], '.-r');
    % dx = abs(start(1)-stop(1));
    % dy = abs(start(2)-stop(2));
    % 
    % % Distance is taken from far edge of each bolt 
    % bb = sqrt(dx^2 + dy^2)+UR3_br+UR5_br;
    % disp('Bb '+ string(bb));

    %% Mathematics Derive Distance and Angle

    % Remove bolt radius offsets
    aa = aa - (UR3_br+UR5_br);
    ab = ab - (UR3_br+UR5_br);

    % Collect Triangle Distances
    A_sides = [aa; ab; UR5_bl*2];
    A_angles = zeros(3,1);

    % Find Angles by Cosine Rule
    A_angles(1) = CosineRule(A_sides(1),A_sides(2),A_sides(3));
    A_angles(2) = CosineRule(A_sides(2),A_sides(3),A_sides(1));
    A_angles(3) = CosineRule(A_sides(3),A_sides(1),A_sides(2));

    % Find out Distance
    halfpoint = ((UR5_A + UR5_B)/2);
    
    if strcmp(p, 'plot')
        plot( [UR3_A(1), halfpoint(1)], [UR3_A(2), halfpoint(2)], ':k');
    end
    dx = abs(UR3_A(1) - halfpoint(1));
    dy = abs(UR3_A(2) - halfpoint(2));
    d_ = sqrt(dx^2 + dy^2);

    % by Cosine Rule
    b = UR5_bl;
    c = A_sides(1);
    d = sqrt( b^2 + c^2 - 2*b*c* cos( A_angles(2) ) );
    distance = d - UR5_r - UR3_r - UR3_bl;
    
    % Figure out Orientation by  Sine Rule
    a = d;
    C = asin( c*sin( A_angles(2))/a );

    % if A_angles(1)>A_angles(2) then angle is between +-90 and +-180
    if A_angles(1)>A_angles(2)
        C = pi-C;
    end
    orientation = C;
    
    if strcmp(p, 'plot')
        plot([UR3_r, UR3_r+distance], [0, 0], 'o-g');
        
        scale = (d1+d2+D_)*1.5;
        
        text( scale/3, -scale/6, 'angle: ('+string(rad2deg(orientation))+' deg) '+string(orientation)+' rad');
        text( scale/3, -scale/4, 'distance: '+string(distance)+'mm');
    
        
        xlim( [-UR3_r*4/3, scale*2/3] );
        axis equal
    end
    
end