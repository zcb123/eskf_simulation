syms x1 x2 ;%定义变量
a = [x1 x2];
f = [x1*x1+2*x1;x2*cos(x2)]; %定义函数，以矩阵的形式 
x = jacobian(f,a); % 求取雅可比矩阵，会发现x是sym类型的
b = [1 2]; 
y = subs(x,a,b); %赋值
%此时结果
y = eval(y); %转化为double型普通矩阵，

syms dThetaX dThetaY dThetaZ
dTheta = [dThetaX, dThetaY, dThetaZ];
dVel = [dVelX, dVelY, dVelZ];
dPos = [dPosX, dPosY, dPosZ];
d_ang_db = [dAng_dbx, dAng_dby, dAng_dbz];
d_vel_db = [dVel_dbx, dVel_dby, dVel_dbz];
di = [dix,diy,diz];
dib = [dibx,diby,dibz];
dw = [dwx,dwy];
error_state = [dTheta,dVel,dPos,d_ang_db,d_vel_db,di,dib,dw];
