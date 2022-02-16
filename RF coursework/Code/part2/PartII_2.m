%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2022 Jan %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  WENTAO GAO &&& JINGZHI WU %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% initialization %%%
clear all;
clc;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% parameters %%%

for i=1:3 
ra(i) =170;  
end

L=130; 
Rplat=130; 
Rbase=290; 

lim=150;        %lim=limit
res=10;         %res=resolution
Xc=-lim:res:lim;
Yc=-lim:res:lim;

alpha = input('  angle :   ');

P=[0,0,0;
   0,0,0];
B=[0,0,0;
   0,0,0];
PB2PP=[0,0,0;
       0,0,0];
Theta=[0,0,0];
t=[0,0,0];

e1=[0,0,0];
e2=[0,0,0];
e3=[0,0,0];

points=zeros(2,2*lim);

m=1;


for i=1:3
    B(1,i)=-Rbase*cos((210+120*(i-1))*(pi()/180));
    B(2,i)=-Rbase*sin((210+120*(i-1))*(pi()/180));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% the loop is to calculate the coordinates of the base and platform
%%% points using the formula and theory mentioned in the coursework sheet
%%% page 7 and 8 %%%


for p=1:length(Yc)
  for q=1:length(Xc)
    for i=1:3 
         P(1,i)=-Rplat*cos((30+alpha+120*(i-1))*(pi()/180))+Xc(p);
         P(2,i)=-Rplat*sin((30+alpha+120*(i-1))*(pi()/180))+Yc(q);
         PB2PP(1,i)=B(1,i)+P(1,i);
         PB2PP(2,i)=B(2,i)+P(2,i);
         e1(i)=-2*PB2PP(2,i)*ra(i);
         e2(i)=-2*PB2PP(1,i)*ra(i);
         e3(i)=(PB2PP(1,i))^2+(PB2PP(2,i))^2+ra(i)^2-L^2;
         t(i)=(-e1(i)-sqrt((e1(i))^2+(e2(i))^2-(e3(i))^2))/(e3(i)-e2(i));
         Theta(i)=2*atan(t(i));
    end
    if isreal(Theta)==1 
        points(1,m)=Xc(p);
        points(2,m)=Yc(q);
        m=m+1;
    end
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% plot %%% 

base=[-B(1,:) -B(1,1);-B(2,:) -B(2,1)];
if points==0
   %do nothing
else
    scatter(points(1,:),points(2,:),'g.');
    hold on;
end

line(base(1,:),base(2,:), 'Color', 'r');
plot(0,0,'b*');
axis equal;








