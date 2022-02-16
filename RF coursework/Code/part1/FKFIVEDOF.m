%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2022 Jan %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  WENTAO GAO &&& JINGZHI WU %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% initialization %%%

clear all
close all
clc
disp('The following code simulates the forward kinematics of a simple 2DOF')
disp('serial manipulator. Figure 2 shows its movement from start to end position,')
disp('Figure 1 shows the location of its end effector at points of its trajectory')
disp('and Figure 3 shows the maximum potential workspace of its end effector')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% A series of joint angles %%%

q1 = [55 65 75 85]*sym(pi)/180;
q2 = [-30 -35 -40 -45]*sym(pi)/180;
q3 = [45 50 55 60]*sym(pi)/180;
q4 = [-60 -55 -50 -45]*sym(pi)/180;

q1_range = [0,180]*sym(pi)/180;
q2_range = [0,180]*sym(pi)/180;
q3_range = [0,120]*sym(pi)/180;
q4_range = [-180,0]*sym(pi)/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Links Lengths %%%

l1 = 0.1 ;
l2 = 0   ;
l3 = 0.1 ;
l4 = 0.1 ;
l5 = 0.1 ;


alpha = [0 , sym(pi)/2 , 0 , 0 , -sym(pi)/2 ];
a     = [0, 0 , l3 , l4 , 0];
theta = [0 ,0 , 0 ,0 , 0];
d     = [l1 , 0 , 0 , 0 , l5];

for i = 1:4

theta(1,1) = q1(1,i);
theta(1,2) = q2(1,i);
theta(1,3) = q3(1,i);
theta(1,4) = q4(1,i);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Trigonometric abbreviations %%%
c1 = cos(theta(1,1));
c2 = cos(theta(1,2));
c3 = cos(theta(1,3));
c4 = cos(theta(1,4));
c5 = cos(theta(1,5));
c12 = cos(q1+q2);

ca1 = cos(alpha(1));
ca2 = cos(alpha(2));
ca3 = cos(alpha(3));
ca4 = cos(alpha(4));
ca5 = cos(alpha(5));

s1 = sin(theta(1,1));
s2 = sin(theta(1,2));
s3 = sin(theta(1,3));
s4 = sin(theta(1,4));
s5 = sin(theta(1,5));
s12 = sin(q1+q2);

sa1 = sin(alpha(1));
sa2 = sin(alpha(2));
sa3 = sin(alpha(3));
sa4 = sin(alpha(4));
sa5 = sin(alpha(5));


T1 = [  [             c1,            -s1,            0,              a(1)]
        [         ca1*s1,         ca1*c1,         -sa1,         -d(1)*sa1]
        [         sa1*s1,         sa1*c1,          ca1,          d(1)*ca1]
        [              0,              0,            0,                 1]  ];
    
T12 = [  [             c2,            -s2,            0,              a(2)]
        [         ca2*s2,         ca2*c2,         -sa2,         -d(2)*sa2]
        [         sa2*s2,         sa2*c2,          ca2,          d(2)*ca2]
        [              0,              0,            0,                 1]  ];
T23 = [  [             c3,            -s3,            0,              a(3)]
        [         ca3*s3,         ca3*c3,         -sa3,         -d(3)*sa3]
        [         sa3*s3,         sa3*c3,          ca3,          d(3)*ca3]
        [              0,              0,            0,                 1]  ];
T34 = [  [             c4,            -s4,            0,              a(4)]
        [         ca4*s4,         ca4*c4,         -sa4,         -d(4)*sa4]
        [         sa4*s4,         sa4*c4,          ca4,          d(4)*ca4]
        [              0,              0,            0,                 1]  ];
T45 = [  [             c5,            -s5,            0,              a(5)]
        [         ca5*s5,         ca5*c5,         -sa5,         -d(5)*sa5]
        [         sa5*s5,         sa5*c5,          ca5,          d(5)*ca5]
        [              0,              0,            0,                 1]  ];

T01 = T1;    
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tip position %%%




frames_x = [0, T01(1,4), T02(1,4), T03(1,4), T04(1,4), T05(1,4)]';   % x coordinate of frames
frames_y = [0, T01(2,4), T02(2,4), T03(2,4), T04(2,4), T05(2,4)]';   % y coordinate of frames
frames_z = [0, T01(3,4), T02(3,4), T03(3,4), T04(3,4), T05(3,4)]';   % z coordinate of frames



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot the robotic arm, in 4 different positions %%%
figure (1)

Color = ['r','g','b','y'];

plot3(frames_x ,frames_y ,frames_z,Color(1,i));

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;
hold on;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Workspace %%%

figure (3) 

for j = 1:10000
c1 = cos(theta(1,1));
c2 = cos(theta(1,2));
c3 = cos(theta(1,3));
c4 = cos(theta(1,4));
c5 = cos(theta(1,5));
c12 = cos(q1+q2);

ca1 = cos(alpha(1));
ca2 = cos(alpha(2));
ca3 = cos(alpha(3));
ca4 = cos(alpha(4));
ca5 = cos(alpha(5));

s1 = sin(theta(1,1));
s2 = sin(theta(1,2));
s3 = sin(theta(1,3));
s4 = sin(theta(1,4));
s5 = sin(theta(1,5));
s12 = sin(q1+q2);

sa1 = sin(alpha(1));
sa2 = sin(alpha(2));
sa3 = sin(alpha(3));
sa4 = sin(alpha(4));
sa5 = sin(alpha(5));

theta(1,1) = 0 + 2*pi*rand;
theta(1,2) = 0 + 2*pi*rand;
theta(1,3) = 0+ 2/3*pi*rand;
theta(1,4) = -pi+ pi*rand;
theta(1,5) = 0;

T1 = [  [             c1,            -s1,            0,              a(1)]
        [         ca1*s1,         ca1*c1,         -sa1,         -d(1)*sa1]
        [         sa1*s1,         sa1*c1,          ca1,          d(1)*ca1]
        [              0,              0,            0,                 1]  ];
    
T12 = [  [             c2,            -s2,            0,              a(2)]
        [         ca2*s2,         ca2*c2,         -sa2,         -d(2)*sa2]
        [         sa2*s2,         sa2*c2,          ca2,          d(2)*ca2]
        [              0,              0,            0,                 1]  ];
T23 = [  [             c3,            -s3,            0,              a(3)]
        [         ca3*s3,         ca3*c3,         -sa3,         -d(3)*sa3]
        [         sa3*s3,         sa3*c3,          ca3,          d(3)*ca3]
        [              0,              0,            0,                 1]  ];
T34 = [  [             c4,            -s4,            0,              a(4)]
        [         ca4*s4,         ca4*c4,         -sa4,         -d(4)*sa4]
        [         sa4*s4,         sa4*c4,          ca4,          d(4)*ca4]
        [              0,              0,            0,                 1]  ];
T45 = [  [             c5,            -s5,            0,              a(5)]
        [         ca5*s5,         ca5*c5,         -sa5,         -d(5)*sa5]
        [         sa5*s5,         sa5*c5,          ca5,          d(5)*ca5]
        [              0,              0,            0,                 1]  ];

T050 = T1*T12*T23*T34*T45;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot the workspace of the robot %%%


X = T050(1,4);
Y = T050(2,4);
Z = T050(3,4);
plot3(X,Y,Z,'b.','MarkerSize',1);
hold on;

end