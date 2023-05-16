function [Current_Pos, Current_head] = carrot_c(C, Pos, head, Pos_f, flag)

% Parameters
%.. Angle Converting Parameters
r2d                     =               180 / pi ;          % Radian to Degree [-]
d2r                     =               1 / r2d ;           % Degree to Radian [-]

%.. Time Step Size
dt                      =               0.1 ;               % Time Step Size [s]

%.. Time
t(1)                    =               0 ;                 % Simulation Time [s]

%.. Circular Orbit
O                       =               C ;         % Centre of Orbit [m]
r                       =               5 ;               % Radius of Orbit [m]

%.. Position and Velocity of Robot
x(1)                    =               Pos(1) ;               % Initial Robot X Position [m]
y(1)                    =               Pos(2) ;                 % Initial Robot Y Position [m]
psi(1)                  =               head ;           % Initial Robot Heading Angle [rad]

p(:,1)                  =               [ x(1), y(1) ]' ;   % Robot Position Initialization [m]
va                      =               5 ;                % Robot Velocity [m/s]

%.. Maximum Lateral Acceleration of Robot
Rmin                    =               5 ;                % Robot Minimum Turn Radius [m]
umax                    =               va^2 / Rmin ;       % Robot Maximum Lateral Acceleration [m]

%.. Design Parameters
kappa                   =               10;              % Gain
lambda                  =              - 45 * d2r * flag ;          % Carrot Distance

% Path Following Algorithm
i                       =               0 ;                 % Time Index

%current state prediction
Current_Pos         =               Pos;
Current_head        =               head;

while (sqrt((Current_Pos(1) - Pos_f(1)) ^ 2 + (Current_Pos(2) - Pos_f(2)) ^ 2) > 1)
    i                   =               i + 1 ;
    %==============================================================================%
    %.. Path Following Algorithm
    
    % Step 1
    % Distance between orbit and current Robot position, d
    d                   =                norm(O - p(:, 1), 2) - r;
    
    % Step 2
    % Orientation of vector from initial waypoint to final waypoint, theta
    theta               =               atan2(p(2, i) - O(2), p(1, i) - O(1));                                                            
    
    % Step 3
    % Carrot position, s = ( xt, yt )
    xt                  =                O(1) + r * (cos(theta + lambda));
    yt                  =                O(2) + r * (sin(theta + lambda));
    
    % Step 4
    % Desired heading angle, psid
    psid                =                atan2(yt - p(2, i), xt - p(1, i));

    % Heading angle error, DEL_psi
    DEL_psi             =                psid - psi(i);
    % Wrapping up DEL_psi
    DEL_psi             =               rem(DEL_psi,2*pi);
    if DEL_psi < -pi
        DEL_psi = DEL_psi + 2*pi;
    elseif DEL_psi > pi
        DEL_psi = DEL_psi-2*pi;
    end
    
    % Step 5
    % Guidance command, u
    u(i)                =                 (kappa*(DEL_psi) * va);
    % Limit u
    if u(i) > umax
        u(i)            =               umax;
    elseif u(i) < -umax
        u(i)            =             - umax;
    end
    %==============================================================================%
    
    %.. Robot Dynamics
    % Dynamic Model of Robot
    dx                  =               va * cos( psi(i) ) ;
    dy                  =               va * sin( psi(i) ) ;
    dpsi                =               u(i) / va ;
    
    % Robot State Update
    x(i+1)              =               x(i) + dx * dt ;
    y(i+1)              =               y(i) + dy * dt ;
    psi(i+1)            =               psi(i) + dpsi * dt ;
    
    % Robot Position Vector Update
    p(:,i+1)            =               [ x(i+1), y(i+1) ]' ;
    Current_Pos         =               [ x(i+1), y(i+1) ]' ; 
    Current_head        =               psi(i+1);
    
    %.. Time Update
    t(i+1)              =               t(i) + dt ;
    if (i == 200)
        break;
    end
end

% Result Plot
% Parameterized Circular Orbit
TH                      =               0:0.01:2*pi ;
Xc                      =               O(1) + r * cos( TH ) ;
Yc                      =               O(2) + r * sin( TH ) ;

%.. Trajectory Plot
figure(2) ;
hold on;
plot( Xc, Yc, 'r', 'LineWidth', 2 ) ;
hold on ;
plot( x, y, 'LineWidth', 2, 'Color', 'blue'  ) ;
hold on ;
xlabel('X (m)') ;
ylabel('Y (m)') ;
legend('Desired Path', 'Robot Trajectory', 'Location', 'southeast' ) ;
axis([ -200 200 -200 200 ]) ;

