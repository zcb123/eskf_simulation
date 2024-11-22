% 定义矢量场
[X, Y, Z] = meshgrid(-2:0.5:2, -2:0.5:2, -2:0.5:2);
U = X;
V = Y;
W = Z;

% 绘制矢量场
quiver3(X, Y, Z, U, V, W);

% 添加标签
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
title('三维矢量场');
grid on;
