% 定义二维矩阵
A = magic(10);

% 绘制矩阵图
imagesc(A);

% 设置颜色条
colorbar;

% 添加标题和标签
title('二维矩阵平面图');
xlabel('列');
ylabel('行');
