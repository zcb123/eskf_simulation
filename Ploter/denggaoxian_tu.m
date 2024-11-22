% 定义二维矩阵
A = peaks(20);

% 绘制等高线图
contour(A, 20); % 20 是等高线的层数

% 添加标题和标签
title('等高线图');
xlabel('列');
ylabel('行');
