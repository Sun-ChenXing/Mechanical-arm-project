clc
clear
% 建立机器模型
L1=Link('d',162,'a',0,  'alpha',-pi/2); 
L2=Link('d',0,  'a',448,'alpha',0,     'offset',-pi/2);
L3=Link('d',0,  'a',42, 'alpha',-pi/2);
L4=Link('d',451,'a',0,   'alpha',pi/2);
L5=Link('d',0,  'a',0,   'alpha',-pi/2);
L6=Link('d',164,'a',-77,'alpha',0);
robot = SerialLink([L1,L2,L3,L4,L5,L6],'name','IRB1200');
initial_p = [0 0 0 0 0 0];%初始关节位置
robot.display()%显示DH参数

%正运动学仿真
fkine_value = robot.fkine(initial_p);%对应于末端执行器位姿的齐次变换矩阵
figure(1)
%版本问题，画图要先加view(3)表示三维 如果版本报错可以把view删去
view(3);robot.plot(initial_p);%机器人处于初始位姿状态图
%理想关节位置
p1 = [0 pi/2 0 pi/4 -pi/12 0];
fkine_value1 = robot.fkine(p1);%对应于末端执行器位姿的齐次变换矩阵
figure(2)
view(3);robot.plot(p1);%理想状态图
%关节空间中随意调整期位置和姿态
figure(3)
view(3);robot.teach

%轨迹规划
init = [0 0 0.5 0 0 0];%初始姿态
over = [0.2 0.3 0.2 pi/4 0 0];%末端姿态
%求出末端执行器在两个笛卡儿位姿之间移动变换矩阵
T_init = transl(0,0,0.5) * trotz(0) * troty(0) * trotx(0);%初始
T_over = transl(0.2,0.3,0.5) * trotz(0) * troty(0) * trotx(pi/4);%末端
%求出对应的关节变量
init_p= robot.ikine(T_init);
end_p = robot.ikine(T_over);
%利用jtraj()函数来进行关节空间的轨迹规划
%整个运动显示时间两秒 采样时间间隔为50微秒
t = [0:0.05:2];
%s sd sdd 分别为关节的位置，速度，加速度
[s,sd,sdd] = jtraj(init_p,end_p,t);
%分别画出关节位置 关节速度 关节加速度曲线变化图
%可以看出轨迹启动与结束比较平滑,在运行中间速度达到最大,加速度变化平滑稳定,无突变、机械硬冲现象。
figure(4)
subplot(2,2,1);plot(s);title('位置');
subplot(2,2,2);plot(sd);title('速度');
subplot(2,2,3);plot(sdd);title('加速度');
%画出机器人轨迹动态图
%可以看出运动平稳，未出现突变点
figure(5)
view(3);robot.plot(s);
%将正运动学作用于关节坐标轨迹，画出末端执行器xy平面内的轨迹：类似于一个圆弧
T = robot.fkine(s);
p = transl(T);
figure(6);
plot(p(:,1), p(:,2));title('xy 平面内的轨迹')

%封闭解 ikine6s方法来计算其逆运动学的封闭解
%T = robot.fkine(s);%轨迹中各个关节点的末端执行器位姿
%q = robot.ikine6s(T);%逆求解各个关节点的末端执行器位姿所需要的关节变量
%qi = p560.ikine6s(T, 'ru')%右手位型解

%以下方法可以求出数值解 但是无法判断是否封闭
%T = robot.fkine(s);%轨迹中各个关节点的末端执行器位姿
%q = robot.ikine(T);%逆求解各个关节点的末端执行器位姿所需要的关节变量
%T1 = robot.fkine(q);%用逆求解之后的关节变量倒推各个关节点所对应的末端执行器位姿
%qi = robot.ikine(T, 'ru');%以右手位形解为例




