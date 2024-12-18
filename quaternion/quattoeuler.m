%返回行向量
function [euler] = quattoeuler(q)
rad2deg=180/pi;
T=[ 1 - 2 * (q(4) *q(4) + q(3) * q(3))  2 * (q(2) * q(3) +q(1) * q(4))         2 * (q(2) * q(4)-q(1) * q(3));
    2 * (q(2) * q(3)-q(1) * q(4))       1 - 2 * (q(4) *q(4) + q(2) * q(2))     2 * (q(3) * q(4)+q(1) * q(2));
    2 * (q(2) * q(4) +q(1) * q(3))      2 * (q(3) * q(4)-q(1) * q(2))          1 - 2 * (q(2) *q(2) + q(3) * q(3))];%cnb

roll  = atan2(T(2,3),T(3,3))*rad2deg;
pitch = asin(-T(1,3))*rad2deg;
yaw   = atan2(T(1,2),T(1,1))*rad2deg;  
yaw = wrap_pi(yaw/57.3)*57.3;
euler = [roll pitch yaw];

end