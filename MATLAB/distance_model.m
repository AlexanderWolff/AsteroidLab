
%% Find out distance between bases from the circumference
% lengths in mm

% Radius of first robot base
r1 = 125/2;

% Radius of second robot base
r2 = 146/2;

% Measured Circumference
c = 3657.5;

% Calculate distance between bases
l = sqrt( ((c-pi*(r1+r2))/2)^2 -(r1-r2)^2 ) - r1 - r2;
distance = l

% Calculate Circumference (used to derive above equation)
arc1 = pi*r1;
arc2 = pi*r2;
segment = sqrt( ( l+r1+r2 )^2 + (r1-r2)^2 );
circumference = arc1+arc2+2*segment

%% Display  
x = linspace(-pi, pi, 1000);
x1 = linspace(-pi, 0, 1000);
x2 = linspace( 0, pi, 1000);
scale = r1+l+r2;


figure;
xlim( [-scale*2/3, scale*2/3] );
ylim( [-scale*2/3, scale*2/3] );
hold on;
plot( r1*sin(x) - l/2, r1*cos(x) );
plot( r2*sin(x) + l/2, r2*cos(x) );


% Display Circumference

plot( r1*sin(x1) - l/2, r1*cos(x1), 'k--' );
plot( r2*sin(x2) + l/2, r2*cos(x2), 'k--' );

plot( [-l/2, l/2],[r1,r2], 'k--' )
plot( [-l/2, l/2],[-r1,-r2],'k--')

plot( [-l/2+r1, l/2-r2], [0,0], '--')