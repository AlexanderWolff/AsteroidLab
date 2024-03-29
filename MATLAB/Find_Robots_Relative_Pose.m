%% Find out Relative distance and orientation between UR3 and UR5 Robots
% lengths in mm
% Measure the distance from bolt A of UR3 to bolt A of UR5 - distance a
% Measure the distance from bolt A of UR3 to bolt B of UR5 - distance b
% bolt A is the closest bolt to the left  of the wire cable 
% bolt B is the opposite to bolt A

% Bolt Diameter
bd = 6.6;
UR3_br = bd/2;

% Distance of bolt centre from base centre
UR3_bl = 55;

% Bolt Diameter
bd = 8.5;
UR5_br = bd/2;

% Distance of bolt centre from base centre
UR5_bl = 60;

% Base Diameter
d1 = 128;
UR3_r = d1/2;

d2 = 149;
UR5_r = d2/2;


displacement = 100;
angle = 32.432;
[aa, ab] = Find_Base_Distances_and_Orientation(displacement, deg2rad(angle), '');

%% Mathematics Derive Distance and Angle

% Add Error
error = 0.5;
aa = aa+error;
ab = ab-error;

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

% Find out Distance by Cosine Rule
b = UR5_bl;
c = A_sides(1);
d = sqrt( b^2 + c^2 - 2*b*c* cos( A_angles(2) ) );
distance = d - UR5_r - UR3_r - UR3_bl;

% Figure out Orientation
disp('Displacement at ' + string(displacement) + ' mm');
disp('Est. Displacement at ' + string(distance) + ' mm');

% Sine Rule
a = d;
C = asin( c*sin( A_angles(2))/a );

% if A_angles(1)>A_angles(2) then angle is between +-90 and +-180
if A_angles(1)>A_angles(2)
    C = pi-C;
end
orientation = C;

disp('Orientation at '  + string(angle) + ' degrees');
disp('Est. Orientation at '  + string(rad2deg(orientation)) + ' degrees');

Find_Base_Distances_and_Orientation(distance, orientation, 'plot');





