%% Align orientation of PA so that PA Z axis faces PB

% Original Point
PointA = Homogeneous.fromT( [0,0,0] );

% Pointing Towards
PointB = Homogeneous.fromT( rand([1,3]) );

% Gradients
A = PointA.T;
B = PointB.T;

dx = B(1) - A(1);
dy = B(2) - A(2);
dz = B(3) - A(3);

ab = sqrt( dx^2 + dy^2 + dz^2 );

ab_projection = sqrt( dx^2 + dy^2 );

%% Right Ascension
% Find out angle between XY projection of AB and X axis

% Triangle: projection of AB, dx, dy
right_ascension = atan2( dx, dy );


%% Declination
% Find out angle between XY projection of AB and AB

% Triangle: projection of AB, AB, height difference
a = ab_projection;
b = ab;
c = dz;

declination = atan2( dz, ab_projection );

disp(rad2deg(right_ascension))
disp(rad2deg(declination))
%%

R = RMatrix.fromXYZ( [0, pi/2-declination, pi/2-right_ascension] ).R;
PointA.setR( R );

%% Guide
scale = 1;

Z = Homogeneous.fromT([0,0,ab]);

%Transform
Z = Z.transform(PointA);


%% Plot
figure;
hold on
PointA.plotL('A')
PointB.plotL('B')
Z.plotL('X')

plot3([-1,1], [ 0,0], [ 0,0], ':g')
plot3([ 0,0], [-1,1], [ 0,0], ':r')
plot3([ 0,0], [ 0,0], [-1,1], ':b')

plot3([A(1),B(1)], [A(2),B(2)], [ 0, 0 ], '-r')

plot3([B(1),B(1)], [B(2),B(2)], [ A(3), B(3) ], '-b')

plot3([A(1),B(1)], [A(2),B(2)], [ A(3), B(3) ], '-m')

plot3( [0,B(1)],[B(2),B(2)], [ 0, 0 ], '-c')

axis equal