% =========================================================================
% LF-TF-CPO 动态容错重规划仿真 (开源客观呈现版)
% 实验场景：双机同时坠毁极限压力测试 (Simultaneous Double Crash at 80%)
% 核心挑战：在任务后期损失 40% 运力，剩余 3 机需在极度不平衡的历史能耗下完成接管
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
crash_ratio = 0.80;          % 极端场景：在任务完成 80% 时发生灾难
crash_row = round(nr * crash_ratio); 
crashed_uavs = [2, 4];       % 设定 UAV-2 和 UAV-4 同时坠毁
survivors = setdiff(1:5, crashed_uavs); 

%% 2. 阶段一：0~80% 区域的协同规划
disp('>>> [阶段 1/2] 正在执行 0~80% 区域的初始协同规划...');
rng(2026); 
[BestPos_5, Traj_5, PartMap_5] = Run_LF_TF_CPO_Bounded(5, 1, crash_row);
pause_points = zeros(5, 3);
E_Phase1 = zeros(1, 5); 
for k = 1:5
    traj = Traj_5{k};
    if ~isempty(traj)
        pause_points(k, :) = traj(end, :); 
        E_Phase1(k) = Calc_Aero_Energy(traj); 
    else
        pause_points(k, :) = [X_vec(round(nc/2)), Y_vec(crash_row), 50]; 
        E_Phase1(k) = 0;
    end
end

%% 3. 阶段二：灾难发生，启动重规划
fprintf('!!! 致命警告：UAV-%d 和 UAV-%d 在 80%% 进度处同时坠毁 !!!\n', crashed_uavs(1), crashed_uavs(2));
disp('>>> [阶段 2/2] 仅剩 3 架无人机，启动紧急容错重规划...');
tic; 
[~, Replan_Traj, E_Transit, E_Phase2, Total_Energy, PartMap_3_Final] = ...
    Run_Aware_CPO_Bounded(length(survivors), crash_row+1, nr, survivors, pause_points, E_Phase1);
replan_time = toc; 
for c_id = crashed_uavs
    Total_Energy(c_id) = E_Phase1(c_id); % 坠毁无人机能量定格
end

%% 4. 量化数据输出
fprintf('\n================== 场景: 双机同时坠毁容错审计 ==================\n');
fprintf('>> [核心指标 1] 3机极限接管重规划耗时: %.4f 秒\n', replan_time);
survivor_energies = Total_Energy(survivors);
fprintf('>> [核心指标 2] 灾后系统极差 (Max - Min): %.2f kJ\n', max(survivor_energies) - min(survivor_energies));
fprintf('>> [核心指标 3] 最终集群最高能耗 (Max_E): %.2f kJ\n', max(survivor_energies));
fprintf('==================================================================\n');

%% 5. 审计级视觉叙事图表渲染
base_colors = [0.85 0.325 0.098; 0 0.447 0.741; 0.466 0.674 0.188; 0.494 0.184 0.556; 0.301 0.745 0.933];

% ------- 图 1：3D 物理无缝缝合图 -------
figure('Name', 'Scenario: 80% Double Crash', 'Color', 'white', 'Position', [50, 400, 1100, 450]);

% 子图 1：Phase 1
ax1 = subplot(1, 2, 1);
surf(X_vec, Y_vec, Z_mat, PartMap_5, 'EdgeColor', 'none', 'FaceAlpha', 0.4); 
colormap(ax1, base_colors); hold on;
for k = 1:5
    traj = Traj_5{k}; 
    if isempty(traj), continue; end
    plot3(traj(:, 1), traj(:, 2), traj(:, 3)+20, 'LineWidth', 2.5, 'Color', base_colors(k,:));
    
    if ismember(k, crashed_uavs)
        plot3(pause_points(k,1), pause_points(k,2), pause_points(k,3)+20, 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
        text(pause_points(k,1), pause_points(k,2), pause_points(k,3)+250, [' CRASH UAV-',num2str(k)], 'Color', 'r', 'FontWeight', 'bold');
    else
        plot3(traj(end, 1), traj(end, 2), traj(end, 3)+20, 'o', 'MarkerSize', 7, 'MarkerFaceColor', base_colors(k,:), 'MarkerEdgeColor', 'w');
    end
end
title('Phase 1: 5-UAV Coverage (0~80%)', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
view(-35, 65); grid on; zlim([0, max(Z_mat(:))+300]);

% 子图 2：Phase 2
ax2 = subplot(1, 2, 2);
PM_Visual = PartMap_3_Final; 
PM_Visual(1:crash_row, :) = NaN; 
surf(X_vec, Y_vec, Z_mat, PM_Visual, 'EdgeColor', 'none', 'FaceAlpha', 0.4); 
colormap(ax2, base_colors(survivors, :)); hold on;
for k = 1:5
    if ismember(k, crashed_uavs)
        plot3(pause_points(k,1), pause_points(k,2), pause_points(k,3)+20, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'w');
    else
        traj_p1 = Traj_5{k};
        if ~isempty(traj_p1)
            plot3(traj_p1(:, 1), traj_p1(:, 2), traj_p1(:, 3)+20, ':', 'LineWidth', 1.5, 'Color', [0.6 0.6 0.6]); 
        end
    end
end
for i = 1:length(survivors)
    orig_id = survivors(i); 
    new_t = Replan_Traj{i};
    if ~isempty(new_t)
        plot3([pause_points(orig_id,1), new_t(1,1)], [pause_points(orig_id,2), new_t(1,2)], [pause_points(orig_id,3)+20, new_t(1,3)+20], '--', 'LineWidth', 1.5, 'Color', [0.93 0.69 0.13]);
        plot3(new_t(1,1), new_t(1,2), new_t(1,3)+20, '^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
        plot3(new_t(:, 1), new_t(:, 2), new_t(:, 3)+20, '-', 'LineWidth', 2.5, 'Color', base_colors(orig_id,:));
        plot3(new_t(end,1), new_t(end,2), new_t(end,3)+20, 's', 'MarkerSize', 7, 'MarkerFaceColor', base_colors(orig_id,:), 'MarkerEdgeColor', 'w');
    end
end
title('Phase 2: Survivors Take Over (80%~100%)', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
view(-35, 65); grid on; zlim([0, max(Z_mat(:))+300]);
legend({'', 'Crash Point', 'Phase 1 Path', 'Transit Line', 'Takeover Start', 'Phase 2 Path'}, 'Location', 'northeast', 'FontSize', 9, 'AutoUpdate', 'off');

% ------- 图 2：能量堆叠柱状图 -------
figure('Name', 'Energy Audit - Scenario 1', 'Color', 'white', 'Position', [250, 80, 600, 420]);
bar_data = zeros(5, 3);
for k = 1:5
    bar_data(k, 1) = E_Phase1(k);
    if ~ismember(k, crashed_uavs)
        idx_surv = find(survivors == k);
        bar_data(k, 2) = E_Transit(idx_surv); 
        bar_data(k, 3) = E_Phase2(idx_surv);
    end
end
b = bar(1:5, bar_data, 'stacked', 'BarWidth', 0.6);
b(1).FaceColor = [0.65 0.65 0.65]; 
b(2).FaceColor = [0.93 0.69 0.13]; 
b(3).FaceColor = [0.85 0.33 0.10]; 

% 移除绝对阈值红线，改为自适应Y轴
title('Energy Rebalancing (Double Crash)', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('UAV Agent ID'); ylabel('Aerodynamic Energy (kJ)');
legend('Phase 1', 'Transit', 'Phase 2', 'Location', 'northwest');
set(gca, 'XTickLabel', {'UAV-1', 'UAV-2 (C)', 'UAV-3', 'UAV-4 (C)', 'UAV-5'}); 

% 自适应 Y 轴高度留出图例空间
ylim([0, max(Total_Energy) * 1.3]); 
grid on; set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.4);

%% ========================================================================
% 底层引擎函数区
% ========================================================================
function eng_kJ = Calc_Aero_Energy(traj)
    PL = 270; k_2 = 45; vc = 8; eng_kJ = 0;
    if isempty(traj) || size(traj,1) < 2, return; end
    for i = 1:(size(traj,1)-1)
        dist = norm(traj(i+1,:) - traj(i,:)); if dist == 0, continue; end
        theta = asin((traj(i+1,3)-traj(i,3))/dist);
        eng_kJ = eng_kJ + (PL + k_2 * max(0, vc*sin(theta))) * (dist/vc);
    end
    eng_kJ = eng_kJ / 1000; 
end

function [BestPos, Trajectories, Final_PartMap] = Run_LF_TF_CPO_Bounded(N, r_start, r_end)
    global D_map Z_mat dy X_vec Y_vec
    [~, nc] = size(D_map); Dim = N * 2; PopSize = 10; MaxIter = 30; 
    lb = zeros(1, Dim); ub = zeros(1, Dim);
    lb(1:2:end) = 1; lb(2:2:end) = r_start; ub(1:2:end) = nc; ub(2:2:end) = r_end;
    Pop = lb + rand(PopSize, Dim) .* (ub - lb); 
    for i = 1:PopSize, Pop(i, 1:2:end) = linspace(nc*0.1, nc*0.9, N) + randn(1, N)*5; end
    Fit = zeros(PopSize, 1);
    for i = 1:PopSize, Fit(i) = Fast_Proxy_Bounded(Pop(i, :), N, r_start, r_end); end
    [BestFit, bIdx] = min(Fit); BestPos = Pop(bIdx, :);
    for iter = 1:MaxIter
        for i = 1:PopSize
            new_pos = Pop(i, :) + (1-iter/MaxIter) * randn(1, Dim) * 10; 
            new_pos = max(min(new_pos, ub), lb); new_fit = Fast_Proxy_Bounded(new_pos, N, r_start, r_end);
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
    sx = max(1, min(nc, round(sol(1:2:end))))'; sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc);
    Ek = zeros(1, N);
    for k = 1:N
        active_mk = part_map(r_start:r_end, :) == k; Ek(k) = sum(active_mk(:)) * 0.07; 
    end
    fit = max(Ek) + 0.1 * std(Ek);
end

function [Traj_Cell, part_map] = Build_Traj_Bounded(sol, N, r_start, r_end)
    global Z_mat D_map dy X_vec Y_vec
    [nr, nc] = size(D_map);
    sx = max(1, min(nc, round(sol(1:2:end))))'; sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc); Traj_Cell = cell(1, N);
    sweep_step = max(1, round(50 / dy)); 
    for k = 1:N
        mk = (part_map == k); px = []; py = []; d = 1;
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

function [BestPos, Final_Traj, Final_E_Trans, Final_E_P2, Final_Total_E, Final_PartMap] = ...
    Run_Aware_CPO_Bounded(N, r_start, r_end, survivors, pause_points, E_Phase1)
    global D_map X_vec
    [~, nc] = size(D_map); Dim = N * 2; PopSize = 10; MaxIter = 30; 
    lb = zeros(1, Dim); ub = zeros(1, Dim);
    lb(1:2:end) = 1; lb(2:2:end) = r_start; ub(1:2:end) = nc; ub(2:2:end) = r_end;
    Pop = lb + rand(PopSize, Dim) .* (ub - lb); 
    surv_X_idx = interp1(X_vec, 1:nc, pause_points(survivors, 1))';
    for i = 1:PopSize, Pop(i, 1:2:end) = max(1, min(nc, surv_X_idx + randn(1, N)*5)); end
    Fit = zeros(PopSize, 1);
    for i = 1:PopSize, Fit(i) = Aware_Fitness_Bounded(Pop(i, :), N, r_start, r_end, survivors, pause_points, E_Phase1, false); end
    [BestFit, bIdx] = min(Fit); BestPos = Pop(bIdx, :);
    for iter = 1:MaxIter
        for i = 1:PopSize
            new_pos = max(min(Pop(i, :) + (1-iter/MaxIter) * randn(1, Dim) * 10, ub), lb);
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
    sx = max(1, min(nc, round(sol(1:2:end))))'; sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc);
    sweep_step = max(1, round(50 / dy)); 
    E_P2_approx = zeros(1, N); start_pts = zeros(N, 3); Traj_Cell = cell(1, N); PM_Final = zeros(nr, nc);
    for k = 1:N
        active_mk = part_map(r_start:r_end, :) == k; E_P2_approx(k) = sum(active_mk(:)) * 0.07; 
        if any(active_mk(:))
            for r = r_start : sweep_step : r_end
                vx = find(part_map(r,:) == k); 
                if ~isempty(vx)
                    start_pts(k, :) = [interp1(1:length(X_vec), X_vec, min(vx)), interp1(1:length(Y_vec), Y_vec, r), Z_mat(round(r), round(min(vx))) + 50]; break;
                end
            end
        else
            start_pts(k, :) = [X_vec(sx(k)), Y_vec(sy(k)), 50];
        end
    end
    
    cost_matrix = zeros(N, N);
    for j=1:N, for i=1:N, cost_matrix(i,j) = norm(pause_points(survivors(i), 1:3) - start_pts(j,:)); end; end
    assigned = zeros(1,N); temp_cost = cost_matrix;
    for s = 1:N
        [~, min_idx] = min(temp_cost(:)); [u, r] = ind2sub([N,N], min_idx);
        assigned(u) = r; temp_cost(u,:) = inf; temp_cost(:,r) = inf;
    end
    E_Transit = zeros(1, N); E_Phase2 = zeros(1, N); Total_E_Array = zeros(1, 5);
    
    crashed_history = setdiff(1:5, survivors);
    for c_id = crashed_history, Total_E_Array(c_id) = E_Phase1(c_id); end
    
    for i = 1:N
        orig_id = survivors(i); r = assigned(i);
        E_Transit(i) = norm(pause_points(orig_id, 1:3) - start_pts(r,:)) * 0.03375; 
        E_Phase2(i) = E_P2_approx(r); Total_E_Array(orig_id) = E_Phase1(orig_id) + E_Transit(i) + E_Phase2(i);
    end
    surv_E = Total_E_Array(survivors); fit = max(surv_E) + 0.5 * std(surv_E); 
    
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
            if ~isempty(Traj_Cell{r})
                E_Transit(i) = Calc_Aero_Energy([pause_points(orig_id, 1:3); Traj_Cell{r}(1,:)]);
                E_Phase2(i) = Calc_Aero_Energy(Traj_Cell{r});
            else
                E_Transit(i) = 0; E_Phase2(i) = 0;
            end
            Total_E_Array(orig_id) = E_Phase1(orig_id) + E_Transit(i) + E_Phase2(i); temp_Traj{i} = Traj_Cell{r};
            active_mk = part_map(r_start:r_end, :) == r; PM_Final(r_start:r_end, :) = PM_Final(r_start:r_end, :) + active_mk * orig_id;
        end
        Traj_Cell = temp_Traj;
    end
end