clear;clc
A = [0 0 0 0 1 0 0 0
    0 0 0 0 0 1 0 0
    0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 1
    0 -12.4928 -2.0824 2.2956 -5.1127 0.0075 0.0024 -0.0053
    0 67.1071 65.2564 -71.9704 14.0176 0.0039 -0.1948 0.1659
    0 144.5482 -394.2536 272.1049 5.2021 -0.4334 1.1287 -0.7492
    0 -300.4564 512.8310 -258.9198 -10.8077 0.6476 -1.3621 0.826];
B = [0 0 0 0 3.651 -10.012 -3.716 7.720]';
C = [1 0 0 0 0 0 0 0
    0 1 0 0 0 0 0 0
    0 0 1 0 0 0 0 0
    0 0 0 1 0 0 0 0];
D = 0;
Q = zeros(8,8);
Q(1,1) = 1000;
Q(2,2) = 1000;
Q(3,3) = 1000;
Q(4,4) = 1000;
R = 1000;
%由上面这个系统，可以计算出K
K = lqr(A,B,Q,R);
Ac = A - B*K;
%对系统进行模拟
x0 = [-3;pi;pi;pi;0;0;0;0]; %初始状态
% x0 = [0.1;pi/2;-pi/3;pi/4;0;0;0;0]; %初始状态
% t = 0:0.05:4;
t=linspace(1,5,1000);
u = zeros(size(t));
[y,x]=lsim(Ac,B,C,D,u,t,x0); 

% % 绘制响应曲线
% figure
% plot(t',y(:,1),'r');hold on;
% plot(t',y(:,2),'g');hold on;
% plot(t',y(:,3),'b');hold on;
% plot(t',y(:,4),'k');

q0= x(:,1)';
q1= x(:,2)';
q2= x(:,3)';
q3= x(:,4)';


dq0= x(:,5)';
dq1= x(:,6)';
dq2= x(:,7)';
dq3= x(:,8)';

u = (x*K')';

% draw trajecotry
[p0, p1, p2, p3, v0, v1, v2, v3] = tripleInvPenKinematics(x');

% % 调节轨迹叠加图
% figure(1); clf;
% nFrame = 15;  %Number of frames to draw
% drawTripleInvPenTraj(t,p0,p1,p2,p3,nFrame);
% 
% % 调节过程展开图
% nFrame=40;
% figure(2); clf 
% drawTimeShift(t,p0,p1,p2,p3,nFrame);
% 
% 动态avi
figure(3);clf;
set(gcf,'Position',[0 0 1500 750]);
nFrameAnim=240;
drawTripleInvPenAnim(t,p0,p1,p2,p3,q0,q1,q2,q3,dq0,dq1,dq2,dq3,u,nFrameAnim)

% % Plot the solution:
% % 画各状态量变化折线图
% figure(3); clf;
% 
% subplot(2,1,1)
% plot(t,q1*180/pi)
% ylabel('q [degree]','color','b')
% title('Triple Pendulum Swing-Up');
% hold on
% plot(t,q2*180/pi,'color','r')
% hold on
% plot(t,q3*180/pi,'color','g')
% legend('q1', 'q2', 'q3')
% grid on;
% grid minor;
% 
% 
% subplot(2,1,2)
% plot(t,dq1,'-o','DisplayName','q1d')
% ylabel('qdot[rad/s]')
% hold on
% plot(t,dq2, '-o','DisplayName','q2d')
% hold on
% plot(t,dq3, '-o','DisplayName','q3d')
% 
% 
% figure(4); clf;
% subplot(3,1,1)
% plot(t,q0, '-o', 'DisplayName','q0')
% ylabel('q0 [m]')
% title('Triple Pendulum Swing-Up');
% 
% subplot(3,1,2)
% plot(t,dq0, '-o', 'DisplayName','q0d')
% ylabel('q0d [m/s]')
% 
% subplot(3,1,3)
% plot(t,u, '-o', 'DisplayName', 'u')
% ylabel('u [m/s^2')