clear all
clc

%% 建立机器人DH参数,初始姿态为竖直。
L(1)=Link('revolute','d',0.216,'a',0,'alpha',pi/2); 
L(2)=Link('revolute','d',0,'a',0.5,'alpha',0,'offset',pi/2);
L(3)=Link('revolute','d',0,'a',sqrt(0.145^2+0.42746^2),'alpha',0,'offset',-atan(427.46/145));
L(4)=Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',atan(427.46/145));
L(5)=Link('revolute','d',0.258,'a',0,'alpha',0);  
robot=SerialLink(L,'name','5-dof', 'manufacturer','innfos')
robot.base=transl(0,0,0.28)

q0=[0 0 0 0 0];
v=[35 20];
w=[-1 1 -1 1 0 2]
robot.plot3d(q0,'tilesize',0.1,'workspace',w,'path','D:\matlab2021\robot\arm','nowrist','view',v)
light('Position',[1 1 1],'color','w');
%robot.teach('workspace',w)

%第一个小球
Positionball_1 = [0.5 0.4 0.5]
r1 = 0.04;
plot_sphere(Positionball_1,r1,'r');
%求解第一个球
T1 =transl(Positionball_1)*rpy2tr(180,0,0);
q1 = robot.ikunc(T1);
q = jtraj(q0,q1,60);
%由初始位置移动到第一个小球
robot.plot3d(q,'view',v,'fps',60,'nowrist')
%第一个小球目标位置
Positiontarget1 = [0.5 -0.5 1];
T2 =transl(Positiontarget1)*rpy2tr(90,90,0);
q2 = robot.ikunc(T2);
t = 30;
q = jtraj(q1,q2,t);
%robot.plot3d(q,'view',v,'fps',60,'nowrist')
%将小球一移动至目标位置
for i = 1:30
    qi = q(i,:);
    Ti = robot.fkine(qi);
    Pi = transl(Ti);
    plot_sphere(Pi,r1,'r');
    robot.plot3d(qi,'view',v,'nowrist')
    cla
end
%%归位
plot_sphere(Pi,r1,'r');
q = jtraj(q2,q0,60);
robot.plot3d(q,'view',v,'fps',60,'nowrist')

%第二个小球
Positionball_2 = [-0.25 -0.5 0.6]
r2 = 0.04;
plot_sphere(Positionball_2,r2,'b');

%求解第二个小球
%第二个小球
T1_2 =transl(Positionball_2)*rpy2tr(180,0,0);
q1_2 = robot.ikunc(T1_2);
q_2 = jtraj(q0,q1_2,60);
robot.plot3d(q_2,'view',v,'fps',60,'nowrist')%由初始位置移动到第二个小球
Positiontarget2 = [0.5 0.8 1.5];%第二个小球目标位置
T2_2 =transl(Positiontarget2)*rpy2tr(90,90,0);
q2_2 = robot.ikunc(T2_2);
t = 30;
q_2 = jtraj(q1_2,q2_2,t);
%robot.plot3d(q,'view',v,'fps',60,'nowrist')
%将小球二移动至目标位置
for i = 1:30
    qi_2 = q_2(i,:);
    Ti_2 = robot.fkine(qi_2);
    Pi_2 = transl(Ti_2);
    plot_sphere(Pi_2,r2,'b');
    robot.plot3d(qi_2,'view',v,'nowrist')
    cla
end

%%归位
plot_sphere(Pi_2,r2,'b');
q = jtraj(q2_2,q0,60);
robot.plot3d(q,'view',v,'fps',60,'nowrist')

%q = [jtraj(q0,q1,60);
 %   jtraj(q1,q2,60);
  %  jtraj(q2,q1,60)];
%robot.plot3d(q,'view',v,'fps',60,'nowrist','trail',{'r','LineWidth',1});




