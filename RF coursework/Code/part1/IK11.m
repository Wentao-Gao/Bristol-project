%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2022 Jan %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  WENTAO GAO &&& JINGZHI WU %%%%%%%%%%%%%%%%%%%%%%%%%%%
function  Q = IK11(X,Y,Z,phy)  

Z = Z-0.1 ; %0.1 is the height of the first section

%%% calculate Q1 %%%
Q(1,1)= atan2(Y,X);
Q(2,1)= atan2(Y,X);

x = X-0.1*cos(phy)*cos(Q(1,1));
y = Y-0.1*cos(phy)*sin(Q(1,1));
z = Z - 0.1*sin(phy);

%%% calculate Q3 and gama %%%
Q(1,3) = -acos((x^2+y^2+z^2)/(2*0.1^2)-1);
Q(2,3) =  acos((x^2+y^2+z^2)/(2*0.1^2)-1);

gama = acos((sqrt(x^2+y^2+z^2))/(2*0.1));

%%% calculate Q2 by using gama and Q3 %%%
Q(1,2) = atan2(z,sqrt(x^2+y^2))+gama;
Q(2,2) = atan2(z,sqrt(x^2+y^2))-gama;

%%% calculate Q4 by using phy , Q2 and Q3 %%%
Q(1,4) = phy - Q(1,2) - Q(1,3)-pi/2;
Q(2,4) = phy - Q(2,2) - Q(2,3)-pi/2;

%%% prtint two solution %%%
disp('1st solution')
disp([ Q(1,1) Q(1,2) Q(1,3) Q(1,4) ]*180/pi)

disp('2nd solution')
disp([ Q(2,1) Q(2,2) Q(2,3) Q(2,4) ]*180/pi)

