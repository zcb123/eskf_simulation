% 定义二维数组
Z = peaks(20);

% 创建 X 和 Y 坐标网格
[X, Y] = meshgrid(1:size(Z, 2), 1:size(Z, 1));

% 绘制表面图
surf(X, Y, Z);

% 添加标题和坐标轴标签
title('二维数组表面图');
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');

% 设置视角和颜色映射
view(3); % 3D 视角
colormap(hot);
colorbar;
