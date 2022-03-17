function    [delta,v,U, vd_p ] = ...
    mpc_control(x,y,yaw,cx,cy,cyaw,cur,dt,Length,Q,R,U,target_v,ind) 
 
u = [x,y,yaw];
vd1=target_v ;
k =cur(ind);
vd2=atan(Length * k);
 
r = [cx(ind), cy(ind), cyaw(ind)];
    Nx=3;%状态量的个数
    Nu =2;%控制量的个数
    Np =60;%预测步长
    Nc=30;%控制步长
    Row=10;%松弛因子
    t_d =u(3);%角度为弧度
    
    kesi=zeros(Nx+Nu,1);
    kesi(1)=u(1)-r(1);%u(1)==X(1)
    kesi(2)=u(2)-r(2);%u(2)==X(2)
    kesi(3)=t_d-r(3); %u(3)==X(3)
    kesi(4)=U(1);
    kesi(5)=U(2);
%     fprintf('Update start, u(1)=%4.2f\n',U(1))
%     fprintf('Update start, u(2)=%4.2f\n',U(2))
    T=dt;
    % Mobile Robot Parameters
    L = Length;
    % Mobile Robot variable
    
%矩阵初始化   
    delta_u=zeros(Nx,Nu);
    
    a=[1    0   -vd1*sin(t_d)*T;
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;
       sin(t_d)*T   0;
       tan(vd2)*T/L      vd1*T/(L * (cos(vd2)^2))];
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    C=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;];
    PHI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    for j=1:1:Np
        PHI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Nx,Nu);
            end
        end
    end
    PHI=cell2mat(PHI_cell);%size(PHI)=[Nx*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);
    H = 1/2*(H+ H');
     error=PHI*kesi;
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=cell2mat(f_cell);
    
 %% 以下为约束生成区域
 %不等式约束
    A_t=zeros(Nc,Nc);%见falcone论文 P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A,求克罗内克积
    Ut=kron(ones(Nc,1),U);%此处感觉论文里的克罗内科积有问题,暂时交换顺序
    umin=[-1;-0.6;];%维数与控制变量的个数相同  控制量的范围，第一个为车速的增量；第二个是车轮转角的增量
    umax=[1;0.6];
%     delta_umin=[0.05;-0.0082;];
%     delta_umax=[0.05;0.0082];
    delta_umin = [ -0.1 ; -0.0085]; %控制量的增量范围，第一个为车速的增量；第二个是车轮转角的增量
    delta_umax = [  0.1 ;  0.0085];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
   % 状态量约束
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
 
    %% 开始求解过程
%     options = optimset('Algorithm','interior-point-convex');
    options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
    %options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    %% 计算输出
%     for i = 1:length(X)
%     if X == [] 
%         delta_u(1)=randn(1)*0.01;
%         delta_u(2)=randn(1)*0.01;
%     elseif  X(i) == 0
%         delta_u(1)=randn(1)*0.01;
%         delta_u(2)=randn(1)*0.01;
%     else
%         delta_u(1)=X(1);
%         delta_u(2)=X(2);
%     end
%     end
        delta_u(1)=X(1);
        delta_u(2)=X(2);
        
    U(1)=kesi(4)+delta_u(1) ;%用于存储上一个时刻的控制量
    U(2)=kesi(5)+delta_u(2); %kesi here is previous step U
    u_real(1)=U(1)+vd1; %  vd1 and vd2 here is U_ref
    u_real(2)=U(2)+vd2;
   delta=  u_real(2);
    v = u_real(1);
%     U(1)=kesi(4)+delta_u(1) + vd1 - vd_p(1);%用于存储上一个时刻的控制量
%     U(2)=kesi(5)+delta_u(2) + vd2 - vd_p(2); %kesi here is previous step U
 
    kesi(4) = U(1);
    kesi(5) = U(2);
    vd_p  = [vd1;vd2 ]; 
%     delta =  U(2)
%     v = 3
 
end


