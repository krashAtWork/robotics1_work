%% find A1_0
%     0.6397    0.0000   -0.7687    0.0928
%     0.7687   -0.0000    0.6397    0.1115
%          0   -1.0000   -0.0000    0.5400
%          0         0         0    1.0000
a_vec = [0.145, 1.150, 0.250, 0, 0, 0];
d_vec = [0.54, 0, 0, -1.812, 0, -0.1];
alpha_vec = [3*pi/2, -pi, -pi/2, pi/2, 3*pi/2, pi];
theta_vec = [0.87674,  -0.78611,   0.21930,   0.16801,   1.68849,   4.65091];


syms theta  [1 6]

A1_0 = [cos(theta1), 0  , -sin(theta1),  a_vec(1)* cos(theta1) ;
        sin(theta1), 0  , cos(theta1),   a_vec(1)* sin(theta1) ;
        0          , -1 , 0          ,  d_vec(1)              ;
        0          , 0  ,   0         ,     1                  ;
        ];

A2_1 = [cos(theta2),  sin(theta2)  , 0 ,  a_vec(2)* cos(theta2) ;
        sin(theta2),  -cos(theta2) , 0  ,   a_vec(2)* sin(theta2) ;
        0          ,  0      ,     -1    ,  d_vec(2)              ;
        0          , 0       ,   0         ,     1                 ;
        ];
    
A3_2 = [cos(theta3),  0  , -sin(theta3) ,  a_vec(3)* cos(theta3) ;
        sin(theta3),    0  , cos(theta3),  a_vec(3)* sin(theta3) ;
        0          ,  -1      ,     0    ,  d_vec(3)              ;
        0          , 0       ,   0         ,     1                 ;
        ];

A4_3 = [cos(theta4), 0  , sin(theta4),  a_vec(4)* cos(theta4) ;
        sin(theta4), 0  , -cos(theta4),   a_vec(4)* sin(theta4) ;
        0          , 1  ,     0          ,  d_vec(4)              ;
        0          , 0  ,     0         ,     1                  ;
        ];
    

A4_0 = A1_0 * A2_1 * A3_2 * A4_3

w = A4_0(1:3, 4)

%% do jacobian on the wrist

wx = w(1,1)
wy = w(2,1)
wz = w(3,1)

Jac(theta1, theta2, theta3) =  [diff(wx, theta1), diff(wx, theta2 ), diff(wx, theta3 );
        diff(wy, theta1), diff(wy, theta2 ), diff(wy, theta3 );
        diff(wz, theta1), diff(wz, theta2 ), diff(wz, theta3 );]
    
double(Jac(0.87674,  -0.78611,   0.21930))




