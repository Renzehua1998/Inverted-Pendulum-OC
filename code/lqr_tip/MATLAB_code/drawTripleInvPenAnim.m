function drawTripleInvPenAnim(t,p0,p1,p2,p3,q0,q1,q2,q3,dq0,dq1,dq2,dq3,u,nFrame)
%
%
% INPUTS:
%  
%   nFrame = scalar integer = number of "freeze" frames to display
%
pause_time=t(end)/nFrame;
clf; hold on;
subplot(3,2,1);
Cart_Width = 0.2;
Cart_Height = 0.1;

Pen_Width = 3;  %pixels

%%%% Figure out the window size:

[xLow, xUpp, yLow, yUpp] = getBounds(p0,p1,p2,p3);

xLow = xLow - 0.7*Cart_Width;
xUpp = xUpp + 0.7*Cart_Width;

yLow = yLow -Cart_Height;
yUpp = yUpp +8*Cart_Height;


Limits = [xLow,xUpp,yLow,2];

grid on;
%%%% Get color map for the figure
map = colormap;
tMap = linspace(t(1),t(end),size(map,1))';

%%%% Plot Rails
title('倒立摆仿真动画');
plot([Limits(1) Limits(2)],-0.5*Cart_Height*[1,1],'k-','LineWidth',4)

%%%% Compute the frames for plotting:
tFrame = linspace(t(1), t(end), nFrame);
cart = interp1(t',p0',tFrame')';
pen1 = interp1(t',p1',tFrame')';
pen2 = interp1(t',p2',tFrame')';
pen3 = interp1(t',p3',tFrame')';

r=interp1(t',q0',tFrame')';
vv=interp1(t',dq0',tFrame')';
u=interp1(t',u',tFrame')';
phi1=interp1(t',(q1')*180/pi,tFrame')';
phi2=interp1(t',(q2')*180/pi,tFrame')';
phi3=interp1(t',(q3')*180/pi,tFrame')';
dphi1=interp1(t',(dq1')*180/pi,tFrame')';
dphi2=interp1(t',(dq2')*180/pi,tFrame')';
dphi3=interp1(t',(dq3')*180/pi,tFrame')';

x = cart(1,1) - 0.5*Cart_Width;
y = -0.5*Cart_Height;
w = Cart_Width;
h = Cart_Height;
hCart = rectangle('Position',[x,y,w,h],'LineWidth',2);


color = interp1(tMap,map,tFrame(1));
%Plot Pendulum1
Rod_X = [cart(1,1), pen1(1,1)];
Rod_Y = [cart(2,1), pen1(2,1)];
pen1_line=line(Rod_X,Rod_Y,'Marker','.','MarkerSize',1,'LineWidth',Pen_Width,'Color',color);

%Plot Pendulum2
Rod_X = [pen1(1,1), pen2(1,1)];
Rod_Y = [pen1(2,1), pen2(2,1)];
pen2_line=line(Rod_X,Rod_Y,'Marker','.','MarkerSize',1,'LineWidth',Pen_Width,'Color',0.3*color);

Rod_X = [pen2(1,1), pen3(1,1)];
Rod_Y = [pen2(2,1), pen3(2,1)];
pen3_line=line(Rod_X,Rod_Y,'Marker','.','MarkerSize',1,'LineWidth',Pen_Width,'Color',0.7*color);

% plot the curve which is made by tipe of the pen3 
idx = 1:2;
x = pen3(1,idx);
y = pen3(2,idx);
% curve=plot(x,y,'Color',c, 'LineWidth',2);
curve=animatedline('Marker','.', 'MarkerSize',5, 'color','r');
xticks([-2 -1.5 -1 -.5  0 .5  1  1.5 2])
%These commands keep the window from automatically rescaling in funny ways.
axis(Limits);
axis('equal');
% pbaspect([16 8 1]);
axis manual;
grid on;
grid minor;


% plot angels 
subplot(3,2,3)
phi1_curve=animatedline('Marker','.','MarkerSize',5,'color','b');
title('各杆角度变化曲线');
ylabel('q[degree]');
xlabel('time [s]');
hold on,
phi2_curve=animatedline('Marker','.','MarkerSize',5,'color','r');
hold on,
subplot(3,2,3)
phi3_curve=animatedline('Marker','.','MarkerSize',5,'color','g');
legend('q1','q2','q3');
% axis([0,t(end)+.2,-200,300]);
% axis manual;
grid on;

% plot angels diff 
subplot(3,2,5)
dphi1_curve=animatedline('Marker','.','MarkerSize',5,'color','b');
title('各杆角速度变化曲线');
ylabel('q[rad/s]');
xlabel('time [s]');
hold on,
dphi2_curve=animatedline('Marker','.','MarkerSize',5,'color','r');
hold on,
subplot(3,2,5)
dphi3_curve=animatedline('Marker','.','MarkerSize',5,'color','g');
legend('q1d','q2d','q3d');
grid on;

% plot x 
subplot(3,2,2)
x_curve=animatedline('Marker','.','MarkerSize',5,'color','b');
title('小车位置变化曲线');
ylabel('x[m]');
xlabel('time [s]');

% plot v 
subplot(3,2,4)
v_curve=animatedline('Marker','.','MarkerSize',5,'color','b');
title('小车速度变化曲线');
ylabel('x[m/s]');
xlabel('time [s]');

% plot x 
subplot(3,2,6)
u_curve=animatedline('Marker','.','MarkerSize',5,'color','b');
title('输入控制变化曲线');
ylabel('u[m/s^2]');
xlabel('time [s]');

% write animation to a file
v=VideoWriter('triple_pen.avi');
open(v);

tic; % start timing 
for i = 1:nFrame
    
    % Compute color:
%     color = interp1(tMap,map,tFrame(i));
      
        
    %Plot Cart
    x = cart(1,i) - 0.5*Cart_Width;
    y = -0.5*Cart_Height;
    w = Cart_Width;
    h = Cart_Height;
    set(hCart,'position',[x,y,w,h] ,'FaceColor',[.25, .25, .25]);
    set(hCart,'EdgeColor',0.6*[.25, .25, .25]);
    
    % set Pendulum1
    Rod_X = [cart(1,i), pen1(1,i)];
    Rod_Y = [cart(2,i), pen1(2,i)];
    set(pen1_line, 'XData',Rod_X,'YData',Rod_Y,'Marker','.',...
        'MarkerSize',20,'MarkerEdgeColor',[0.38, 0.70, 0.80],...
        'LineWidth',Pen_Width,'Color',[0.3, 0.80, 0.9410])
    
    % set pendulum 2
    Rod_X = [pen1(1,i), pen2(1,i)];
    Rod_Y = [pen1(2,i), pen2(2,i)];
    set(pen2_line, 'XData',Rod_X,'YData',Rod_Y,'Marker','.',...
        'MarkerEdgeColor',[0.4, 0.4, .4],'MarkerSize',20,...
        'LineWidth',Pen_Width,'Color',[0.6350, 0.0780, 0.1840])
    
    % set pendulum 3
    Rod_X = [pen2(1,i), pen3(1,i)];
    Rod_Y = [pen2(2,i), pen3(2,i)];
    set(pen3_line, 'XData',Rod_X,'YData',Rod_Y,'Marker','.',...
        'MarkerEdgeColor',[0.4, 0.4, .4],'MarkerSize',20,...
        'LineWidth',Pen_Width,'Color',[0, 0.5, 0])
    
    
    % plot trajectory curve of pen3
     
    x = pen3(1,i);
    y = pen3(2,i);
    addpoints(curve,x,y);
    
    % plot angels as it moves
    xphi1= tFrame(i);
    yphi1 = phi1(i);
    addpoints(phi1_curve,xphi1,yphi1)
    
    xphi2= tFrame(i);
    yphi2 = phi2(i);
    addpoints(phi2_curve,xphi2,yphi2)
    
    xphi3= tFrame(i);
    yphi3 = phi3(i);
    addpoints(phi3_curve,xphi3,yphi3)
    
    % plot angels diff as it moves
    xphi1= tFrame(i);
    yphi1 = dphi1(i);
    addpoints(dphi1_curve,xphi1,yphi1)
    
    xphi2= tFrame(i);
    yphi2 = dphi2(i);
    addpoints(dphi2_curve,xphi2,yphi2)
    
    xphi3= tFrame(i);
    yphi3 = dphi3(i);
    addpoints(dphi3_curve,xphi3,yphi3)
    
    % plot x v u as it moves
    xphi1= tFrame(i);
    yphi1 = r(i);
    addpoints(x_curve,xphi1,yphi1)
    
    xphi2= tFrame(i);
    yphi2 = vv(i);
    addpoints(v_curve,xphi2,yphi2)
    
    xphi3= tFrame(i);
    yphi3 = u(i);
    addpoints(u_curve,xphi3,yphi3)
    
    %get frame as an image
    frame=getframe(gcf);
    writeVideo(v,frame);
    
%     pause(pause_time)
    
    drawnow;
      
end

% create AVI file
close(v);

end



function [xLow, xUpp, yLow, yUpp] = getBounds(p0,p1,p2,p3)
%
% Returns the upper and lower bound on the data in val
%

val = [p0,p1,p2,p3];
xLow = min(val(1,:));
xUpp = max(val(1,:));
yLow = min(val(2,:));
yUpp = max(val(2,:));

end