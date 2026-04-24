% =========================================================================
% LF-TF-CPO 动态容错重规划仿真 (开源客观呈现版)
% 实验场景：级联故障 (Cascading Crash at 60% and 90%)
% 核心挑战：在不同任务阶段连续损失运力，系统需具备记忆历史能耗的连续接管能力
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

%% 2. 阶段一：0~60% 协同规划
rng(2026); 
row_60 = round(nr * 0.60); 
crashed_1 = 3; % 第一次灾难：UAV-3 在 60% 处坠毁
disp('>>> [阶段 1/3] 执行 0~60% 区域初始协同规划...');
[~, Traj_P1, PartMap_5] = Run_LF_TF_CPO_Bounded(5, 1, row_60);
pause_pts_1 = zeros(5, 3); 
E_P1 = zeros(1, 5);
for k = 1:5
    if ~isempty(Traj_P1{k})
        pause_pts_1(k,:) = Traj_P1{k}(end,:); 
        E_P1(k) = Calc_Aero_Energy(Traj_P1{k}); 
    else
        pause_pts_1(k,:) = [X_vec(round(nc/2)), Y_vec(row_60), 50];
    end
end

%% 3. 阶段二：60% 处 UAV-3 坠毁，启动第一次重规划
survivors_1 = setdiff(1:5, crashed_1); 
row_90 = round(nr * 0.90);
fprintf('!!! 事件 1：UAV-%d 在 60%% 处坠毁 !!!\n', crashed_1);
disp('>>> [阶段 2/3] 启动第一次重规划 (4机接管 60%~90%)...');
tic;
[~, Traj_P2, E_Trans_1, E_P2_Mid, ~, PartMap_4] = ...
    Run_Aware_CPO_Bounded(4, row_60+1, row_90, survivors_1, pause_pts_1, E_P1);
time_1 = toc;
pause_pts_2 = zeros(5, 3); 
E_Hist_90 = zeros(1, 5); 
E_Hist_90(crashed_1) = E_P1(crashed_1); % 坠毁机能量定格
for idx = 1:4
    orig_id = survivors_1(idx);
    if ~isempty(Traj_P2{idx})
        pause_pts_2(orig_id,:) = Traj_P2{idx}(end,:); 
    else
        pause_pts_2(orig_id,:) = pause_pts_1(orig_id,:); 
    end
    % 核心：累加阶段一 + 第一次调机 + 阶段二能耗，形成新的历史记忆
    E_Hist_90(orig_id) = E_P1(orig_id) + E_Trans_1(idx) + E_P2_Mid(idx);
end

%% 4. 阶段三：90% 处 UAV-5 级联坠毁，启动第二次重规划
crashed_2 = 5; 
survivors_2 = setdiff(survivors_1, crashed_2); % 仅剩 3 机
fprintf('!!! 事件 2：UAV-%d 在 90%% 处发生级联坠毁 !!!\n', crashed_2);
disp('>>> [阶段 3/3] 启动第二次极限重规划 (3机接管 90%~100%)...');
tic;
% 将携带前 90% 记忆的 E_Hist_90 传入引擎
[~, Traj_P3, E_Trans_2, E_P3_Final, Final_Total_E, PartMap_3] = ...
    Run_Aware_CPO_Bounded(3, row_90+1, nr, survivors_2, pause_pts_2, E_Hist_90);
time_2 = toc;

%% 5. 数据输出
fprintf('\n================== 场景 2: 级联坠毁容错审计 ==================\n');
fprintf('>> [耗时追踪] 第一次级联响应 (5变4): %.4f 秒\n', time_1);
fprintf('>> [耗时追踪] 第二次级联响应 (4变3): %.4f 秒\n', time_2);
final_survivor_E = Final_Total_E(survivors_2);
fprintf('>> [最终结果] 灾后系统极差 (Max - Min): %.2f kJ\n', max(final_survivor_E) - min(final_survivor_E));
fprintf('>> [最终结果] 集群最高能耗 (Max_E): %.2f kJ\n', max(final_survivor_E));
fprintf('==============================================================\n');

%% 6. 视觉叙事与图表渲染
base_colors = [0.85 0.325 0.098; 0 0.447 0.741; 0.466 0.674 0.188; 0.494 0.184 0.556; 0.301 0.745 0.933];

% ------- 图 1：3D 级联拼接图 -------
figure('Name', 'Scenario 2: Cascading Crash', 'Color', 'white', 'Position', [50, 400, 1100, 450]);

% 子图 1: Phase 1
ax1 = subplot(1, 2, 1);
surf(X_vec, Y_vec, Z_mat, PartMap_5, 'EdgeColor', 'none', 'FaceAlpha', 0.4); 
colormap(ax1, base_colors); hold on;
for k = 1:5
    traj = Traj_P1{k}; 
    if isempty(traj), continue; end
    plot3(traj(:, 1), traj(:, 2), traj(:, 3)+20, 'LineWidth', 2.5, 'Color', base_colors(k,:));
    if k == crashed_1
        plot3(pause_pts_1(k,1), pause_pts_1(k,2), pause_pts_1(k,3)+20, 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
        text(pause_pts_1(k,1), pause_pts_1(k,2), pause_pts_1(k,3)+250, ' CRASH 1', 'Color', 'r', 'FontWeight', 'bold');
    end
end
title('Phase 1: 0~60%', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold'); 
view(-35, 65); grid on; zlim([0, max(Z_mat(:))+300]);

% 子图 2: Phase 2 & 3
ax2 = subplot(1, 2, 2);
PM_Visual = PartMap_4; 
PM_Visual(row_90+1:end, :) = PartMap_3(row_90+1:end, :); 
PM_Visual(1:row_60, :) = NaN;
surf(X_vec, Y_vec, Z_mat, PM_Visual, 'EdgeColor', 'none', 'FaceAlpha', 0.4); 
colormap(ax2, base_colors); hold on;

% 绘制两个坠毁点
plot3(pause_pts_1(crashed_1,1), pause_pts_1(crashed_1,2), pause_pts_1(crashed_1,3)+20, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'w');
plot3(pause_pts_2(crashed_2,1), pause_pts_2(crashed_2,2), pause_pts_2(crashed_2,3)+20, 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
text(pause_pts_2(crashed_2,1), pause_pts_2(crashed_2,2), pause_pts_2(crashed_2,3)+250, ' CRASH 2', 'Color', 'r', 'FontWeight', 'bold');

% 画 60%-90% 航线
for i = 1:4
    orig_id = survivors_1(i); 
    new_t = Traj_P2{i};
    if ~isempty(new_t)
        plot3(new_t(:, 1), new_t(:, 2), new_t(:, 3)+20, '-', 'LineWidth', 2.0, 'Color', base_colors(orig_id,:)); 
    end
end

% 画 90%-100% 航线
for i = 1:3
    orig_id = survivors_2(i); 
    new_t = Traj_P3{i};
    if ~isempty(new_t)
        plot3(new_t(:, 1), new_t(:, 2), new_t(:, 3)+20, '-', 'LineWidth', 3.0, 'Color', base_colors(orig_id,:)); 
    end
end
title('Phases 2 & 3: Cascading Takeover', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold'); 
view(-35, 65); grid on; zlim([0, max(Z_mat(:))+300]);

% ------- 图 2：累积能量堆叠柱状图 -------
figure('Name', 'Energy Audit - Cascading', 'Color', 'white', 'Position', [250, 80, 600, 420]);
bar_data = zeros(5, 3);
bar_data(:, 1) = E_P1';

% 累加 Phase 2
for idx = 1:4
    orig_id = survivors_1(idx); 
    bar_data(orig_id, 2) = bar_data(orig_id, 2) + E_Trans_1(idx); 
    bar_data(orig_id, 3) = bar_data(orig_id, 3) + E_P2_Mid(idx);
end

% 累加 Phase 3
for idx = 1:3
    orig_id = survivors_2(idx); 
    bar_data(orig_id, 2) = bar_data(orig_id, 2) + E_Trans_2(idx); 
    bar_data(orig_id, 3) = bar_data(orig_id, 3) + E_P3_Final(idx);
end

b = bar(1:5, bar_data, 'stacked', 'BarWidth', 0.6);
b(1).FaceColor = [0.65 0.65 0.65]; 
b(2).FaceColor = [0.93 0.69 0.13]; 
b(3).FaceColor = [0.85 0.33 0.10]; 

% 移除红线，改为自适应 Y 轴
title('Energy Rebalancing (Cascading Crash)', 'FontName', 'Times New Roman', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('UAV Agent ID'); ylabel('Total Aerodynamic Energy (kJ)');
legend('Phase 1 (Initial)', 'Total Transit (Sum)', 'Total Re-coverage (Sum)', 'Location', 'northwest');
set(gca, 'XTickLabel', {'UAV-1', 'UAV-2', 'UAV-3 (C1)', 'UAV-4', 'UAV-5 (C2)'}); 

% 自适应缩放 Y 轴高度以留出图例空间
ylim([0, max(Final_Total_E) * 1.3]); 
grid on;

%% ========================================================================
% 底层引擎函数 (格式化展开排版版)
% ========================================================================
function eng_kJ = Calc_Aero_Energy(traj)
    PL = 270; k_2 = 45; vc = 8; eng_kJ = 0;
    if isempty(traj) || size(traj,1) < 2
        return; 
    end
    for i = 1:(size(traj,1)-1)
        dist = norm(traj(i+1,:) - traj(i,:)); 
        if dist == 0
            continue; 
        end
        theta = asin((traj(i+1,3)-traj(i,3))/dist); 
        eng_kJ = eng_kJ + (PL + k_2 * max(0, vc*sin(theta))) * (dist/vc);
    end
    eng_kJ = eng_kJ / 1000; 
end

function [BestPos, Trajectories, Final_PartMap] = Run_LF_TF_CPO_Bounded(N, r_start, r_end)
    global D_map Z_mat dy X_vec Y_vec
    [~, nc] = size(D_map); 
    Dim = N * 2; PopSize = 10; MaxIter = 30; 
    lb = zeros(1, Dim); ub = zeros(1, Dim);
    lb(1:2:end) = 1; lb(2:2:end) = r_start; 
    ub(1:2:end) = nc; ub(2:2:end) = r_end;
    
    Pop = lb + rand(PopSize, Dim) .* (ub - lb); 
    for i = 1:PopSize
        Pop(i, 1:2:end) = linspace(nc*0.1, nc*0.9, N) + randn(1, N)*5; 
    end
    
    Fit = zeros(PopSize, 1);
    for i = 1:PopSize
        Fit(i) = Fast_Proxy_Bounded(Pop(i, :), N, r_start, r_end); 
    end
    [BestFit, bIdx] = min(Fit); BestPos = Pop(bIdx, :);
    
    for iter = 1:MaxIter
        for i = 1:PopSize
            new_pos = max(min(Pop(i, :) + (1-iter/MaxIter) * randn(1, Dim) * 10, ub), lb);
            new_fit = Fast_Proxy_Bounded(new_pos, N, r_start, r_end);
            if new_fit < Fit(i)
                Pop(i, :) = new_pos; Fit(i) = new_fit;
                if new_fit < BestFit
                    BestFit = new_fit; BestPos = new_pos; 
                end
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
    [GX, GY] = meshgrid(1:nc, 1:nr); 
    ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc); 
    Ek = zeros(1, N);
    for k = 1:N
        Ek(k) = sum(part_map(r_start:r_end, :) == k, 'all') * 0.07; 
    end
    fit = max(Ek) + 0.1 * std(Ek);
end

function [Traj_Cell, part_map] = Build_Traj_Bounded(sol, N, r_start, r_end)
    global Z_mat D_map dy X_vec Y_vec
    [nr, nc] = size(D_map);
    sx = max(1, min(nc, round(sol(1:2:end))))'; 
    sy = max(1, min(nr, round(sol(2:2:end))))'; 
    [GX, GY] = meshgrid(1:nc, 1:nr); 
    ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc); 
    Traj_Cell = cell(1, N);
    sweep_step = max(1, round(50 / dy)); 
    
    for k = 1:N
        mk = (part_map == k); px = []; py = []; d = 1;
        for r = r_start : sweep_step : r_end
            vx = find(mk(r, :)); 
            if isempty(vx)
                continue; 
            end
            if d == 1
                px = [px, min(vx), max(vx)]; 
            else
                px = [px, max(vx), min(vx)]; 
            end
            py = [py, r, r]; d = -d; 
        end
        if isempty(px)
            Traj_Cell{k}=[]; continue; 
        end
        phys_x = interp1(1:length(X_vec), X_vec, px); 
        phys_y = interp1(1:length(Y_vec), Y_vec, py);
        phys_z = zeros(size(phys_x)); 
        for i = 1:length(phys_x)
            phys_z(i) = Z_mat(round(py(i)), round(px(i))) + 50; 
        end
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
    for i = 1:PopSize
        Pop(i, 1:2:end) = max(1, min(nc, surv_X_idx + randn(1, N)*5)); 
    end
    
    Fit = zeros(PopSize, 1);
    for i = 1:PopSize
        Fit(i) = Aware_Fitness_Bounded(Pop(i, :), N, r_start, r_end, survivors, pause_points, E_Phase1, false); 
    end
    [BestFit, bIdx] = min(Fit); BestPos = Pop(bIdx, :);
    
    for iter = 1:MaxIter
        for i = 1:PopSize
            new_pos = max(min(Pop(i, :) + (1-iter/MaxIter) * randn(1, Dim) * 10, ub), lb);
            new_fit = Aware_Fitness_Bounded(new_pos, N, r_start, r_end, survivors, pause_points, E_Phase1, false);
            if new_fit < Fit(i)
                Pop(i, :) = new_pos; Fit(i) = new_fit;
                if new_fit < BestFit
                    BestFit = new_fit; BestPos = new_pos; 
                end
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
    [GX, GY] = meshgrid(1:nc, 1:nr); 
    ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); part_map = reshape(mi, nr, nc);
    
    sweep_step = max(1, round(50 / dy)); 
    E_P2_approx = zeros(1, N); 
    start_pts = zeros(N, 3); 
    Traj_Cell = cell(1, N); 
    PM_Final = zeros(nr, nc);
    
    for k = 1:N
        active_mk = part_map(r_start:r_end, :) == k; 
        E_P2_approx(k) = sum(active_mk(:)) * 0.07; 
        if any(active_mk(:))
            for r = r_start : sweep_step : r_end
                vx = find(part_map(r,:) == k); 
                if ~isempty(vx)
                    start_pts(k, :) = [interp1(1:length(X_vec), X_vec, min(vx)), interp1(1:length(Y_vec), Y_vec, r), Z_mat(round(r), round(min(vx))) + 50]; 
                    break;
                end
            end
        else
            start_pts(k, :) = [X_vec(sx(k)), Y_vec(sy(k)), 50];
        end
    end
    
    cost_matrix = zeros(N, N); 
    for j=1:N
        for i=1:N
            cost_matrix(i,j) = norm(pause_points(survivors(i), 1:3) - start_pts(j,:)); 
        end
    end
    
    assigned = zeros(1,N); 
    temp_cost = cost_matrix;
    for s = 1:N
        [~, min_idx] = min(temp_cost(:)); 
        [u, r] = ind2sub([N,N], min_idx); 
        assigned(u) = r; 
        temp_cost(u,:) = inf; 
        temp_cost(:,r) = inf; 
    end
    
    E_Transit = zeros(1, N); 
    E_Phase2 = zeros(1, N); 
    Total_E_Array = zeros(1, 5);
    for c_id = setdiff(1:5, survivors)
        Total_E_Array(c_id) = E_Phase1(c_id); 
    end
    
    for i = 1:N
        orig_id = survivors(i); 
        r = assigned(i); 
        E_Transit(i) = norm(pause_points(orig_id, 1:3) - start_pts(r,:)) * 0.03375; 
        E_Phase2(i) = E_P2_approx(r); 
        Total_E_Array(orig_id) = E_Phase1(orig_id) + E_Transit(i) + E_Phase2(i);
    end
    
    surv_E = Total_E_Array(survivors); 
    fit = max(surv_E) + 0.5 * std(surv_E); 
    
    if is_final
        temp_Traj = cell(1, N);
        for k = 1:N
            mk = (part_map == k); px = []; py = []; d = 1;
            for r = r_start : sweep_step : r_end
                vx = find(mk(r, :)); 
                if isempty(vx), continue; end
                if d == 1
                    px = [px, min(vx), max(vx)]; 
                else
                    px = [px, max(vx), min(vx)]; 
                end
                py = [py, r, r]; d = -d; 
            end
            if ~isempty(px)
                phys_x = interp1(1:length(X_vec), X_vec, px); 
                phys_y = interp1(1:length(Y_vec), Y_vec, py);
                phys_z = zeros(size(phys_x)); 
                for z_i = 1:length(phys_x)
                    phys_z(z_i) = Z_mat(round(py(z_i)), round(px(z_i))) + 50; 
                end
                Traj_Cell{k} = [phys_x', phys_y', phys_z'];
            end
        end
        for i = 1:N
            orig_id = survivors(i); r = assigned(i);
            if ~isempty(Traj_Cell{r})
                E_Transit(i) = Calc_Aero_Energy([pause_points(orig_id, 1:3); Traj_Cell{r}(1,:)]); 
                E_Phase2(i) = Calc_Aero_Energy(Traj_Cell{r});
            else
                E_Transit(i) = 0; 
                E_Phase2(i) = 0; 
            end
            Total_E_Array(orig_id) = E_Phase1(orig_id) + E_Transit(i) + E_Phase2(i); 
            temp_Traj{i} = Traj_Cell{r};
            active_mk = part_map(r_start:r_end, :) == r; 
            PM_Final(r_start:r_end, :) = PM_Final(r_start:r_end, :) + active_mk * orig_id;
        end
        Traj_Cell = temp_Traj;
    end
end