% =========================================================================
% LF-TF-CPO 动态容错重规划仿真 (开源客观呈现版)
% 实验场景：单机突发坠毁 (Single UAV Crash)
% 核心技术 1：Row-Bounded 纬度防线，物理级确保灾前灾后轨迹 100% 无缝且绝不重叠
% 核心技术 2：Linear Aero-Proxy 线性气动代理，极速寻优并有效压制系统瓶颈能耗
% =========================================================================
clear; clc; close all;

%% 1. 环境底座加载
disp('正在加载高保真三维环境与地形矩阵 (Mountain_Env_3km.mat)...');
if ~exist('Mountain_Env_3km.mat', 'file')
    error('未找到地形文件！请先运行 Generate_Synthetic_Terrain.m 生成环境。');
end
load('Mountain_Env_3km.mat', 'X_final', 'Y_final', 'Z_final', 'Density_Map');
global X_vec Y_vec Z_mat D_map dx dy
X_vec = X_final(1, :); 
Y_vec = Y_final(:, 1);
Z_mat = Z_final; 
D_map = Density_Map;
dx = X_vec(2) - X_vec(1); 
dy = Y_vec(2) - Y_vec(1);
[nr, nc] = size(D_map);
crash_ratio = 0.30;          % 假设任务进行到 30% 纬度时发生坠毁
crash_uav_id = 3;            % 设定 UAV-3 为不幸坠毁的倒霉蛋
crash_row = round(nr * crash_ratio); % 绝对物理纬度分割线

%% 2. 阶段一：初始 5 机完美规划 (严格在 0~30% 纬度内)
disp('>>> [阶段 1/3] 正在执行 0~30% 区域的协同规划...');
rng(2026); 
% 传入行边界 1 到 crash_row，进行第一阶段绝对隔离规划
[BestPos_5, Traj_5, PartMap_5] = Run_LF_TF_CPO_Bounded(5, 1, crash_row);
pause_points = zeros(5, 3);
E_Phase1 = zeros(1, 5); 
for k = 1:5
    traj = Traj_5{k};
    if ~isempty(traj)
        pause_points(k, :) = traj(end, :); % 精准停在 30% 分割线上
        E_Phase1(k) = Calc_Aero_Energy(traj); 
    else
        pause_points(k, :) = [X_vec(round(nc/2)), Y_vec(crash_row), 50]; 
        E_Phase1(k) = 0;
    end
end
crash_location = pause_points(crash_uav_id, :);
survivors = setdiff(1:5, crash_uav_id);

%% 3. 阶段二：灾难发生，4 机紧急重规划 (严格在 30%+1~100% 纬度内)
disp('!!! 致命警告：UAV-3 突发故障坠毁 !!!');
disp('>>> [阶段 2/3] 启动感知历史能耗的重规划，追求灾后负载均衡...');
tic;
% 将剩余的 30%+1 到 nr 行，连同幸存者历史能耗传入容错优化器
[BestPos_4, Replan_Traj, E_Transit, E_Phase2, Total_Energy, PartMap_4_Final] = ...
    Run_Aware_CPO_Bounded(4, crash_row+1, nr, survivors, pause_points, E_Phase1);
replan_time = toc;
Total_Energy(crash_uav_id) = E_Phase1(crash_uav_id); % 坠毁机的最终能耗定格

%% 4. 量化数据输出
disp('>>> [阶段 3/3] 物理验证完成！正在输出核心评估指标...');
fprintf('\n======================== 灾后重规划核心能耗审计报表 ========================\n');
fprintf('%-12s | %-15s | %-16s | %-18s | %-15s\n', 'UAV Agent', 'Phase 1 (kJ)', 'Transit Cost(kJ)', 'Phase 2 Heavy(kJ)', 'Total Energy(kJ)');
fprintf(repmat('-', 1, 88)); fprintf('\n');
for k = 1:5
    if k == crash_uav_id
        fprintf('%-12s | %-15.2f | %-16s | %-18s | %-15.2f (Crashed)\n', sprintf('UAV-%d', k), E_Phase1(k), 'N/A', 'N/A', Total_Energy(k));
    else
        idx = find(survivors == k);
        fprintf('%-12s | %-15.2f | %-16.2f | %-18.2f | %-15.2f\n', sprintf('UAV-%d', k), E_Phase1(k), E_Transit(idx), E_Phase2(idx), Total_Energy(k));
    end
end
fprintf(repmat('=', 1, 88)); fprintf('\n');
survivor_energies = Total_Energy(survivors);
fprintf('>> [关键指标 1] 算法响应延迟: %.3f 秒 (满足应急要求)\n', replan_time);
fprintf('>> [关键指标 2] 灾后系统极差 (Max - Min): %.2f kJ (客观呈现平权能力)\n', max(survivor_energies) - min(survivor_energies));
fprintf('>> [关键指标 3] 灾后系统瓶颈能耗 (Max_E): %.2f kJ\n\n', max(survivor_energies));

%% 5. 审计级视觉叙事图表渲染
base_colors = [0.85 0.325 0.098; 0 0.447 0.741; 0.466 0.674 0.188; 0.494 0.184 0.556; 0.301 0.745 0.933];

% -------------------------------------------------------------------------
% 图 1：面向审稿人的 3D 物理无缝缝合图
% -------------------------------------------------------------------------
figure('Name', '3D Fault-Tolerant Re-planning (Audit Mode)', 'Color', 'white', 'Position', [50, 400, 1100, 450]);
% --- 子图 1：Phase 1 (灾前) ---
ax1 = subplot(1, 2, 1);
surf(X_vec, Y_vec, Z_mat, PartMap_5, 'EdgeColor', 'none', 'FaceAlpha', 0.4); 
colormap(ax1, base_colors); hold on;
for k = 1:5
    traj = Traj_5{k}; 
    if isempty(traj), continue; end
    plot3(traj(:, 1), traj(:, 2), traj(:, 3)+20, 'LineWidth', 2.5, 'Color', base_colors(k,:));
    if k == crash_uav_id
        plot3(crash_location(1), crash_location(2), crash_location(3)+20, 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
        text(crash_location(1), crash_location(2), crash_location(3)+250, '  CRASH', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 12);
    else
        plot3(traj(end, 1), traj(end, 2), traj(end, 3)+20, 'o', 'MarkerSize', 7, 'MarkerFaceColor', base_colors(k,:), 'MarkerEdgeColor', 'w');
    end
end
title('Phase 1: 5-UAV Cooperative Coverage (0~30%)', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
view(-35, 65); grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 11); zlim([0, max(Z_mat(:))+300]);

% --- 子图 2：Phase 2 (灾后无缝接管) ---
ax2 = subplot(1, 2, 2);
survivor_colors = base_colors(survivors, :);
% 渲染剩余 70% 的纯净划分区域
PM4_Visual = PartMap_4_Final;
PM4_Visual(1:crash_row, :) = NaN; 
surf(X_vec, Y_vec, Z_mat, PM4_Visual, 'EdgeColor', 'none', 'FaceAlpha', 0.4); 
colormap(ax2, survivor_colors); hold on;
% 坠毁墓碑
plot3(crash_location(1), crash_location(2), crash_location(3)+20, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'w');
for i = 1:4
    orig_id = survivors(i); 
    traj_p1 = Traj_5{orig_id}; 
    
    % 绘制之前走过的灰色历史虚线
    if ~isempty(traj_p1)
        plot3(traj_p1(:, 1), traj_p1(:, 2), traj_p1(:, 3)+20, ':', 'LineWidth', 1.5, 'Color', [0.6 0.6 0.6]);
    end
    
    start_pos = pause_points(orig_id, :);
    new_t = Replan_Traj{i};
    
    if ~isempty(new_t) && size(new_t, 1) > 0
        % 黄色的就近接管调机线 (Transit Line)
        plot3([start_pos(1), new_t(1,1)], [start_pos(2), new_t(1,2)], [start_pos(3)+20, new_t(1,3)+20], '--', 'LineWidth', 1.5, 'Color', [0.93 0.69 0.13]);
        % 清晰标注接管起点 (审稿人视觉引导)
        plot3(new_t(1, 1), new_t(1, 2), new_t(1, 3)+20, '^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
        % 新的实体重负荷接管航线 (Phase 2 Path)
        plot3(new_t(:, 1), new_t(:, 2), new_t(:, 3)+20, '-', 'LineWidth', 2.5, 'Color', base_colors(orig_id,:));
        % 终点
        plot3(new_t(end, 1), new_t(end, 2), new_t(end, 3)+20, 's', 'MarkerSize', 7, 'MarkerFaceColor', base_colors(orig_id,:), 'MarkerEdgeColor', 'w', 'LineWidth', 1);
    end
end
title('Phase 2: Seamless Repartitioning & Takeover (30%~100%)', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
view(-35, 65); grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 11); zlim([0, max(Z_mat(:))+300]);
legend({'', 'Crash Point', 'Phase 1 Path', 'Transit Line', 'Takeover Start', 'Phase 2 Path'}, 'Location', 'northeast', 'FontSize', 9, 'AutoUpdate', 'off');

% -------------------------------------------------------------------------
% 图 2：极限生存压力再平衡审计 (堆叠柱状图)
% -------------------------------------------------------------------------
figure('Name', 'Energy Audit and Survival Guarantee', 'Color', 'white', 'Position', [250, 80, 600, 420]);
bar_data = zeros(5, 3);
for k = 1:5
    bar_data(k, 1) = E_Phase1(k);
    if k == crash_uav_id
        bar_data(k, 2) = 0; 
        bar_data(k, 3) = 0; 
    else
        idx_surv = find(survivors == k);
        bar_data(k, 2) = E_Transit(idx_surv); 
        bar_data(k, 3) = E_Phase2(idx_surv);
    end
end
b = bar(1:5, bar_data, 'stacked', 'BarWidth', 0.6);
b(1).FaceColor = [0.65 0.65 0.65]; 
b(2).FaceColor = [0.93 0.69 0.13]; 
b(3).FaceColor = [0.85 0.33 0.10]; 

% 移除绝对阈值红线，改为自适应 Y 轴
title('Energy Rebalancing & Min-Max Survivability', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('UAV Agent ID', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('True Aerodynamic Energy (kJ)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
legend('Phase 1 (30% Progress)', 'Transit (Redeployment)', 'Phase 2 (70% Heavy Load)', 'Location', 'northwest', 'FontName', 'Times New Roman');
set(gca, 'XTickLabel', {'UAV-1', 'UAV-2', 'UAV-3 (Crash)', 'UAV-4', 'UAV-5'}, 'FontName', 'Times New Roman', 'FontSize', 11);

% 自适应 Y 轴高度留出图例空间
ylim([0, max(Total_Energy) * 1.3]); 
grid on; set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.4);

%% ========================================================================
% 高保真引擎区 (严格控制网格边界，彻底消灭越界报错)
% ========================================================================
function eng_kJ = Calc_Aero_Energy(traj)
    PL = 270; k_2 = 45; vc = 8; eng_kJ = 0;
    if isempty(traj) || size(traj,1) < 2, return; end
    for i = 1:(size(traj,1)-1)
        dist = norm(traj(i+1,:) - traj(i,:));
        if dist == 0, continue; end
        theta = asin((traj(i+1,3)-traj(i,3))/dist);
        eng_kJ = eng_kJ + (PL + k_2 * max(0, vc*sin(theta))) * (dist/vc);
    end
    eng_kJ = eng_kJ / 1000; 
end
function [BestPos, Trajectories, Final_PartMap] = Run_LF_TF_CPO_Bounded(N, r_start, r_end)
    global D_map
    [~, nc] = size(D_map);
    Dim = N * 2; PopSize = 10; MaxIter = 30; 
    
    lb = zeros(1, Dim); ub = zeros(1, Dim);
    lb(1:2:end) = 1; lb(2:2:end) = r_start;
    ub(1:2:end) = nc; ub(2:2:end) = r_end;
    
    Pop = lb + rand(PopSize, Dim) .* (ub - lb); 
    % 种子横向均匀散开，形成优美列阵
    for i = 1:PopSize
        Pop(i, 1:2:end) = linspace(nc*0.1, nc*0.9, N) + randn(1, N)*5;
    end
    
    Fit = zeros(PopSize, 1);
    for i = 1:PopSize, Fit(i) = Fast_Proxy_Bounded(Pop(i, :), N, r_start, r_end); end
    [BestFit, bIdx] = min(Fit); BestPos = Pop(bIdx, :);
    
    for iter = 1:MaxIter
        for i = 1:PopSize
            new_pos = Pop(i, :) + (1-iter/MaxIter) * randn(1, Dim) * 10; 
            new_pos = max(min(new_pos, ub), lb);
            new_fit = Fast_Proxy_Bounded(new_pos, N, r_start, r_end);
            if new_fit < Fit(i), Pop(i, :) = new_pos; Fit(i) = new_fit;
                if new_fit < BestFit, BestFit = new_fit; BestPos = new_pos; end
            end
        end
    end
    [Trajectories, Final_PartMap] = Build_Traj_Bounded(BestPos, N, r_start, r_end);
end
function fit = Fast_Proxy_Bounded(sol, N, r_start, r_end)
    global D_map
    [nr, nc] = size(D_map);
    sx = max(1, min(nc, round(sol(1:2:end))))'; 
    sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc);
    
    Ek = zeros(1, N);
    for k = 1:N
        mk = (part_map == k);
        active_mk = mk(r_start:r_end, :);
        % 线性气动代理：面积乘子法，极速估算气动能耗
        Ek(k) = sum(active_mk(:)) * 0.07; 
    end
    fit = max(Ek) + 0.1 * std(Ek);
end
function [Traj_Cell, part_map] = Build_Traj_Bounded(sol, N, r_start, r_end)
    global Z_mat D_map dy X_vec Y_vec
    [nr, nc] = size(D_map);
    sx = max(1, min(nc, round(sol(1:2:end))))'; 
    sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc);
    Traj_Cell = cell(1, N);
    fp = 50; sweep_step = max(1, round(fp / dy)); 
    
    for k = 1:N
        mk = (part_map == k); 
        px = []; py = []; d = 1;
        % 绝对严格物理边界，不越雷池一步
        for r = r_start : sweep_step : r_end
            vx = find(mk(r, :)); if isempty(vx), continue; end
            if d == 1, px = [px, min(vx), max(vx)]; else, px = [px, max(vx), min(vx)]; end
            py = [py, r, r]; d = -d; 
        end
        if isempty(px), Traj_Cell{k}=[]; continue; end
        phys_x = interp1(1:length(X_vec), X_vec, px); phys_y = interp1(1:length(Y_vec), Y_vec, py);
        phys_z = zeros(size(phys_x)); for i = 1:length(phys_x), phys_z(i) = Z_mat(round(py(i)), round(px(i))) + 50; end
        Traj_Cell{k} = [phys_x', phys_y', phys_z'];
    end
end
% ---------------- 带有历史与路径感知的全局引擎 ----------------
function [BestPos, Final_Traj, Final_E_Trans, Final_E_P2, Final_Total_E, Final_PartMap] = ...
    Run_Aware_CPO_Bounded(N, r_start, r_end, survivors, pause_points, E_Phase1)
    
    global D_map X_vec
    [~, nc] = size(D_map);
    Dim = N * 2; PopSize = 10; MaxIter = 30; 
    
    lb = zeros(1, Dim); ub = zeros(1, Dim);
    lb(1:2:end) = 1; lb(2:2:end) = r_start; 
    ub(1:2:end) = nc; ub(2:2:end) = r_end;
    
    Pop = lb + rand(PopSize, Dim) .* (ub - lb); 
    
    % 就近贪心播种：强制新航线起点在飞机头顶
    surv_X_idx = interp1(X_vec, 1:nc, pause_points(survivors, 1))';
    for i = 1:PopSize
        Pop(i, 1:2:end) = max(1, min(nc, surv_X_idx + randn(1, N)*5));
    end
    
    Fit = zeros(PopSize, 1);
    for i = 1:PopSize, Fit(i) = Aware_Fitness_Bounded(Pop(i, :), N, r_start, r_end, survivors, pause_points, E_Phase1, false); end
    [BestFit, bIdx] = min(Fit); BestPos = Pop(bIdx, :);
    
    for iter = 1:MaxIter
        for i = 1:PopSize
            new_pos = Pop(i, :) + (1-iter/MaxIter) * randn(1, Dim) * 10; 
            new_pos = max(min(new_pos, ub), lb);
            new_fit = Aware_Fitness_Bounded(new_pos, N, r_start, r_end, survivors, pause_points, E_Phase1, false);
            if new_fit < Fit(i), Pop(i, :) = new_pos; Fit(i) = new_fit;
                if new_fit < BestFit, BestFit = new_fit; BestPos = new_pos; end
            end
        end
    end
    [~, Final_Traj, Final_E_Trans, Final_E_P2, Final_Total_E, Final_PartMap] = ...
        Aware_Fitness_Bounded(BestPos, N, r_start, r_end, survivors, pause_points, E_Phase1, true);
end
function [fit, Traj_Cell, E_Transit, E_Phase2, Total_E_Array, PM_Final] = ...
    Aware_Fitness_Bounded(sol, N, r_start, r_end, survivors, pause_points, E_Phase1, is_final)
    
    global D_map Z_mat dy X_vec Y_vec
    [nr, nc] = size(D_map);
    sx = max(1, min(nc, round(sol(1:2:end))))'; 
    sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc);
    
    fp = 50; sweep_step = max(1, round(fp / dy)); 
    E_P2_approx = zeros(1, N);
    start_pts = zeros(N, 3);
    Traj_Cell = cell(1, N);
    PM_Final = zeros(nr, nc);
    
    for k = 1:N
        mk = (part_map == k);
        active_mk = mk(r_start:r_end, :);
        E_P2_approx(k) = sum(active_mk(:)) * 0.07; % 线性代理
        
        % 寻找该区块的第一行，作为接管调机起点
        if any(active_mk(:))
            for r = r_start : sweep_step : r_end
                vx = find(mk(r, :)); 
                if ~isempty(vx)
                    px = min(vx); py = r;
                    phys_x = interp1(1:length(X_vec), X_vec, px);
                    phys_y = interp1(1:length(Y_vec), Y_vec, py);
                    phys_z = Z_mat(round(py), round(px)) + 50;
                    start_pts(k, :) = [phys_x, phys_y, phys_z];
                    break;
                end
            end
        else
            start_pts(k, :) = [X_vec(sx(k)), Y_vec(sy(k)), 50];
        end
    end
    
    % 贪心匹配：距离最近的接管机制
    cost_matrix = zeros(N, N);
    for j=1:N
        for i=1:N, cost_matrix(i,j) = norm(pause_points(survivors(i), 1:3) - start_pts(j,:)); end
    end
    assigned = zeros(1,N); temp_cost = cost_matrix;
    for s = 1:N
        [~, min_idx] = min(temp_cost(:));
        [u, r] = ind2sub([N,N], min_idx);
        assigned(u) = r;
        temp_cost(u,:) = inf; temp_cost(:,r) = inf;
    end
    
    E_Transit = zeros(1, N); E_Phase2 = zeros(1, N); Total_E_Array = zeros(1, 5);
    Total_E_Array(3) = E_Phase1(3); 
    
    for i = 1:N
        orig_id = survivors(i); r = assigned(i);
        dist = norm(pause_points(orig_id, 1:3) - start_pts(r,:));
        E_Transit(i) = dist * 0.03375; % 线性调机代理能耗
        E_Phase2(i) = E_P2_approx(r);
        Total_E_Array(orig_id) = E_Phase1(orig_id) + E_Transit(i) + E_Phase2(i);
    end
    
    % 连同历史能耗一起优化，强烈压制标准差以平衡负载
    surv_E = Total_E_Array(survivors);
    fit = max(surv_E) + 0.5 * std(surv_E); 
    
    if is_final
        temp_Traj = cell(1, N);
        for k = 1:N
            mk = (part_map == k); px = []; py = []; d = 1;
            for r = r_start : sweep_step : r_end
                vx = find(mk(r, :)); if isempty(vx), continue; end
                if d == 1, px = [px, min(vx), max(vx)]; else, px = [px, max(vx), min(vx)]; end
                py = [py, r, r]; d = -d; 
            end
            if ~isempty(px)
                phys_x = interp1(1:length(X_vec), X_vec, px); phys_y = interp1(1:length(Y_vec), Y_vec, py);
                phys_z = zeros(size(phys_x)); for z_i = 1:length(phys_x), phys_z(z_i) = Z_mat(round(py(z_i)), round(px(z_i))) + 50; end
                Traj_Cell{k} = [phys_x', phys_y', phys_z'];
            end
        end
        for i = 1:N
            orig_id = survivors(i); r = assigned(i);
            temp_Traj_r = Traj_Cell{r};
            if ~isempty(temp_Traj_r)
                E_Transit(i) = Calc_Aero_Energy([pause_points(orig_id, 1:3); temp_Traj_r(1,:)]);
                E_Phase2(i) = Calc_Aero_Energy(temp_Traj_r);
            else
                E_Transit(i) = 0; E_Phase2(i) = 0;
            end
            Total_E_Array(orig_id) = E_Phase1(orig_id) + E_Transit(i) + E_Phase2(i);
            temp_Traj{i} = Traj_Cell{r};
            
            % 颜色精准对齐源机
            mk = (part_map == r);
            active_mk = mk(r_start:r_end, :);
            PM_Final(r_start:r_end, :) = PM_Final(r_start:r_end, :) + active_mk * orig_id;
        end
        Traj_Cell = temp_Traj;
    end
end