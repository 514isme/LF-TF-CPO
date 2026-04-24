% =========================================================================
% LF-TF-CPO 算法环境构建：合成山地 DEM 生成与物理特征重构
% 核心功能：生成 3km x 3km 合成山地，提取坡度、粗糙度与 TW-CVT 任务密度地图
% 注意：本脚本用于替代受限的真实地理数据，确保 GitHub 仓库的完全可复现性
% =========================================================================

clear; clc; close all;

%% 1. 定义空间参数 (严格对齐论文设定)
L = 3000;               % 区域长度 (3000m) [cite: 318]
grid_res = 10;          % 分辨率 (10m) [cite: 319]
N = L / grid_res;       % 矩阵规模 (300x300) [cite: 319]

x = linspace(0, L, N);
y = linspace(0, L, N);
[X_final, Y_final] = meshgrid(x, y);

%% 2. 生成合成山地地形 (分形噪声法)
% 使用不同频率的正弦波与随机噪声叠加，模拟自然山体的分级特征
rng(42); % 固定随机种子确保结果可重复 [cite: 122]

% 基础大地形 (大尺度山脉)
Z_base = 300 * sin(X_final/800) .* cos(Y_final/1000) + 200 * cos(sqrt(X_final.^2 + Y_final.^2)/1200);

% 中尺度起伏 (山脊与山谷)
Z_mid = 100 * sin(X_final/300) .* sin(Y_final/400) + 80 * cos(X_final/250);

% 小尺度细节 (地形粗糙度)
Z_detail = 20 * randn(size(X_final));
Z_detail = imfilter(Z_detail, fspecial('gaussian', [15 15], 3)); % 平滑处理防止过于尖锐

% 叠加并归一化高程
Z_final = Z_base + Z_mid + Z_detail;
Z_final = Z_final - min(Z_final(:)); % 确保最小高度为0
Z_final = imfilter(Z_final, fspecial('gaussian', [5 5], 1)); % 最终平滑

disp('合成山地 DEM 生成完毕，开始提取物理特征...');

%% 3. 提取核心地形特征 (严格遵循论文公式)

% 3.1 计算坡度 (Slope, 单位: 度) [cite: 353, 358]
[dzdx, dzdy] = gradient(Z_final, grid_res, grid_res);
Slope_final = atan(sqrt(dzdx.^2 + dzdy.^2)) * 180 / pi;

% 3.2 计算地形粗糙度 (Roughness) [cite: 363]
% 论文定义：局部高程标准差与均值的比值 (此处采用 3x3 邻域)
Z_mean = imfilter(Z_final, ones(3)/9, 'replicate');
Z_std = sqrt(imfilter(Z_final.^2, ones(3)/9, 'replicate') - Z_mean.^2);
Roughness_final = Z_std ./ (Z_mean + 1); % 加1防止除零，且区域高程均大于500m时此项稳定 [cite: 369]

%% 4. 构建 TW-CVT 任务密度矩阵 (Density Map) [cite: 380]

% 4.1 特征归一化 (Min-Max Normalization) [cite: 372]
norm_slope = (Slope_final - min(Slope_final(:))) / (max(Slope_final(:)) - min(Slope_final(:)) + 1e-6);
norm_rough = (Roughness_final - min(Roughness_final(:))) / (max(Roughness_final(:)) - min(Roughness_final(:)) + 1e-6);

% 4.2 加权生成密度分布 [cite: 380, 383]
% 权重设定：坡度 0.7，粗糙度 0.3，基础常数 0.1
Density_Map = 0.7 * norm_slope + 0.3 * norm_rough + 0.1;

disp('TW-CVT 任务密度矩阵构建完毕！');

%% 5. 保存数据与可视化
save('Mountain_Env_3km.mat', 'X_final', 'Y_final', 'Z_final', 'Slope_final', 'Roughness_final', 'Density_Map');
disp('环境矩阵已保存至 Mountain_Env_3km.mat，该文件将作为所有后续实验的输入。');

% 绘图评估 (风格对齐论文 Figure 1) [cite: 447]
figure('Name', 'Synthetic Terrain Analysis', 'Position', [100, 100, 1100, 450], 'Color', 'white');

% 子图1：3D 高程模型
subplot(1,2,1);
surf(X_final, Y_final, Z_final, 'EdgeColor', 'none');
shading interp; colormap(gca, terrainColorMap()); colorbar;
title('Synthetic 3D Terrain (10 m Resolution)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Elevation (m)');
daspect([1 1 0.5]); view(-35, 45); camlight; lighting gouraud; grid on;

% 子图2：任务密度热力图
subplot(1,2,2);
imagesc(x, y, Density_Map);
axis xy; colormap(gca, 'hot'); colorbar;
title('Task Density Map (\rho) for LF-TF-CPO');
xlabel('X (m)'); ylabel('Y (m)');
grid on;

function cmap = terrainColorMap()
    % 简易地形着色器：从深绿(谷)到棕褐(峰)
    c1 = [0.1, 0.4, 0.1]; % 深绿
    c2 = [0.9, 0.8, 0.6]; % 浅黄
    c3 = [0.5, 0.3, 0.1]; % 棕色
    n = 256;
    cmap = [interp1([1, n/2, n], [c1; c2; c3], 1:n)];
end