%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2022 Jan %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  WENTAO GAO &&& JINGZHI WU %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% initialization %%%
clear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% parameters %%%

l1 = 0.1 ;
l2 = 0   ;
l3 = 0.1 ;
l4 = 0.1 ;
l5 = 0.1 ;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% DH table %%%

theta = [0 ,0 , 0 ,0 , 0];
d     = [l1 , 0 , 0 , 0 , l5];
alpha = [0 , sym(pi)/2 , 0 , 0 , -sym(pi)/2 ];
a     = [0, 0 , l3 , l4 , 0];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Stright path to 5 points planning  %%%

for i = 1:100

 if i<=50

 X = 0.18+0.0006*i;
 Y = 0.12+0.0005*i;
 Z = 0.08+0.0006*i;

 end

 if i>50
 X = 0.21-0.0006*(i-50);
 Y = 0.145+0.0005*(i-50);
 Z = 0.11-0.0006*(i-50);
 end

 
Q = IK11(X,Y,Z,0);


theta(1) = Q(1,1);
theta(2) = Q(1,2);
theta(3) = Q(1,3);
theta(4) = Q(1,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% get essentical sin and cos value %%%

c1 = cos(theta(1));
c2 = cos(theta(2));
c3 = cos(theta(3));
c4 = cos(theta(4));
c5 = cos(theta(5));

ca1 = cos(alpha(1));
ca2 = cos(alpha(2));
ca3 = cos(alpha(3));
ca4 = cos(alpha(4));
ca5 = cos(alpha(5));

s1 = sin(theta(1));
s2 = sin(theta(2));
s3 = sin(theta(3));
s4 = sin(theta(4));
s5 = sin(theta(5));


sa1 = sin(alpha(1));
sa2 = sin(alpha(2));
sa3 = sin(alpha(3));
sa4 = sin(alpha(4));
sa5 = sin(alpha(5));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% get the transfor matrix of each part of Ly- Arm %%%

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

T010 = T1;    
T020 = T010*T12;
T030 = T020*T23;
T040 = T030*T34;
T050 = T040*T45;


frames_x = [0, T010(1,4), T020(1,4), T030(1,4), T040(1,4), T050(1,4)];   
frames_y = [0, T010(2,4), T020(2,4), T030(2,4), T040(2,4), T050(2,4)];   
frames_z = [0, T010(3,4), T020(3,4), T030(3,4), T040(3,4), T050(3,4)];   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% plot in matlab %%%

scatter3(frames_x(6),frames_y(6),frames_z(6),'b');
hold on;

arm=plot3(frames_x,frames_y,frames_z,'g');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;
pause(0.0001);
delete(arm);
end