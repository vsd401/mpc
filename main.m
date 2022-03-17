clc
clear 
close all
 
% Kp = 1.0 ; 
dt = 0.1  ;% [s]
Length = 2.9 ;% [m] 车辆轴距
Nx=3;%状态量的个数
Nu =2;%控制量的个数
Np =60;%预测步长
Nc=30;%控制步长
% Row=10;%松弛因子
Q=100*eye(Nx*Np,Nx*Np);    
R=1*eye(Nc*Nu);
    
max_steer =60 * pi/180; % in rad
target_v =30.0 / 3.6;
 
cx = 0:0.1:200; % sampling interception from 0 to 200, with step 0.1
for i = 1:500% here we create a original reference line, which the vehicle should always follow when there is no obstacles;
    cy(i) = -sin(cx(i)/10)*cx(i)/8;
end
for i = 501: length(cx)
    cy(i) = -sin(cx(i)/10)*cx(i)/8; %cy(500);
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% here we provide another reference line for testing, now you dont need to
% use it
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
p = [cx', cy'];
 %计算一阶导数
 for i = 1:length(cx)-1
 pd(i) = (p(i+1,2)-p(i,2))/(p(i+1,1)-p(i,1));
 end
 pd(end+1) = pd(end);
  %计算二阶导数
 for i =2: length(cx)-1
     pdd(i) = (p(i+1,2)-2*p(i,2) + p(i-1,2))/(0.5*(-p(i-1,1)+p(i+1,1)))^2;
 end
      pdd(1) = pdd(2);
     pdd(length(cx)) = pdd(length(cx)-1);
    %计算曲率 
  for i  = 1:length(cx)-1
     cur(i) = (pdd(i))/(1+pd(i)^2)^(1.5);
  end
  
  cx= cx';
  cy =cy';
  cyaw = atan(pd');
%   ck = cur';
%%%%%%%%%%%%%%%%%%%%   above things are preprocessing   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
i = 1;
T = 80;
lastIndex = length(cx);
x = 0.1; y = -0.1; yaw = 0.1; v = 0.1;
U = [0.01;0.01];
vd1_p = 0;
vd2_p = 0;
vd_p = [vd1_p; vd2_p];
ind =0;
figure(1)
plot(cx,cy,'r-');
ylabel('Y(m)');
xlabel('X(m)');
hold on
     
while ind < length(cur)
  [ind, e] = calc_target_index(x,y,cx,cy,cyaw);
    if ind > length(cur) % we do not allow the vehicle deviation larger than 4
        fprintf('complete!\n')
        break;
    end
  [delta,v,U, vd_p ] = mpc_control(x,y,yaw,cx,cy,cyaw,cur,dt,Length,Q,R,U,target_v,ind) ;
   
    if abs(e)> 3 % we do not allow the vehicle deviation larger than 4
        fprintf('diviation too big!\n')
        break;
    end
     [x,y,yaw,v] = update(x,y,yaw,v, delta, dt,Length, max_steer); % vehicle model
%      figure(2)
%      hold on
%      plot(i,delta,'*')
     posx(i) = x;
     posy(i)  =y;
     i = i+1;
     delta111(i-1)=delta;
     VV111(i-1)=v;
     yaw111(i-1)=yaw;
%      figure(1)
%      hold on
     plot(posx(i-1),posy(i-1),'.')
%      pause(0.01);
     
      hold on
end
h=figure(2);
plot([1:length(VV111)]*dt,VV111);
axis([0,35,0,10]);
ylabel('车速（m/s）');
xlabel('时间(s)');
figure(3)
plot([1:length(VV111)]*dt,yaw111);
ylabel('横摆角速度（rad/s）');
xlabel('时间(s)');
figure(4)
plot([1:length(VV111)]*dt,delta111);
ylabel('前轮转角（rad）');
xlabel('时间(s)');