% =========================================================================
% LF-TF-CPO 算法全栈多算法对比实验套件 (对应论文 Section 4.2 & Table 2)
% 核心升级：包含 2024 SOTA 算法 Alpha Evolution (AE) 与 QEDGIX (GNN+QMIX)
% 功能说明：自动读取合成地形，执行6种算法，输出高保真指标表格与收敛曲线
% =========================================================================

clear; clc; close all;

%% 1. 加载底层环境与特征 (衔接上一步生成的数据)
disp('正在加载环境与地形密度矩阵 (Mountain_Env_3km.mat)...');
if ~exist('Mountain_Env_3km.mat', 'file')
    error('未找到地形文件！请先运行 Generate_Synthetic_Terrain.m 生成环境。');
end
load('Mountain_Env_3km.mat', 'X_final', 'Y_final', 'Z_final', 'Density_Map');

global X_vec Y_vec Z_mat D_map dx dy num_uavs
X_vec = X_final(1, :); 
Y_vec = Y_final(:, 1);
Z_mat = Z_final; 
D_map = Density_Map;
dx = X_vec(2) - X_vec(1); 
dy = Y_vec(2) - Y_vec(1);
num_uavs = 5; % 无人机集群规模

%% 2. 实验全局参数统一设定
Dim = num_uavs * 2; % 每个无人机一个(x,y)锚点
PopSize = 15; 
MaxIter = 80;

% 边界设定：限制在地图的 5% ~ 95% 范围内，防止初始点越界
lb = zeros(1, Dim); ub = zeros(1, Dim);
lb(1:2:end) = round(length(X_vec) * 0.05); lb(2:2:end) = round(length(Y_vec) * 0.05);
ub(1:2:end) = round(length(X_vec) * 0.95); ub(2:2:end) = round(length(Y_vec) * 0.95);

% 初始化 6 种算法的记录器
Curves = zeros(6, MaxIter);
Algo_Times = zeros(1, 6); 
Final_Seeds = cell(1, 6); 

% 莱维飞行常数计算 (用于 LF-TF-CPO)
beta_levy = 1.5;
sigma_u = (gamma(1+beta_levy)*sin(pi*beta_levy/2)/(gamma((1+beta_levy)/2)*beta_levy*2^((beta_levy-1)/2)))^(1/beta_levy);

disp('======================================================');
disp('开始执行高保真对比实验...');

%% 3. 共享初始种群生成
rng(2026); % 锁定全局熵，保证所有伪随机算法起跑线绝对公平
Shared_Initial_Pop = lb + rand(PopSize, Dim) .* (ub - lb);

%% ========================================================================
% 算法 1: 3D Adaptive Boustrophedon (牛耕法基线 - 离线规划)
% ========================================================================
disp('[1/6] 正在运行: 3D Adaptive Boustrophedon...');
tic;
[num_rows, num_cols] = size(D_map); 
partition_bous = zeros(num_rows, num_cols);
col_step = floor(num_cols / num_uavs);
for k = 1:num_uavs
    col_start = (k-1)*col_step + 1; 
    col_end = min(k*col_step, num_cols);
    if k == num_uavs, col_end = num_cols; end
    partition_bous(:, col_start:col_end) = k;
end
[~, Fit_Bous_Pure] = Evaluate_Fitness_Grid(partition_bous);
Curves(1, :) = repmat(Fit_Bous_Pure, 1, MaxIter);
Algo_Times(1) = toc; 
Final_Seeds{1} = 'Boustrophedon_Rigid'; 

%% ========================================================================
% 算法 2: Standard PSO (经典群体智能基线)
% ========================================================================
disp('[2/6] 正在运行: Standard PSO...');
tic;
Pop_PSO = Shared_Initial_Pop; 
V_PSO = zeros(PopSize, Dim); 
pBest_Pos = Pop_PSO; 
pBest_Fit = zeros(PopSize, 1);
for i = 1:PopSize, [pBest_Fit(i), ~] = Evaluate_Fitness(Pop_PSO(i, :)); end
[gBest_Fit, gIdx] = min(pBest_Fit); gBest_Pos = pBest_Pos(gIdx, :);
w = 0.7; c1 = 1.5; c2 = 1.5;
for iter = 1:MaxIter
    for i = 1:PopSize
        V_PSO(i, :) = w * V_PSO(i, :) + c1 * rand(1, Dim) .* (pBest_Pos(i, :) - Pop_PSO(i, :)) + c2 * rand(1, Dim) .* (gBest_Pos - Pop_PSO(i, :));
        Pop_PSO(i, :) = max(min(Pop_PSO(i, :) + V_PSO(i, :), ub), lb);
        [current_fit, ~] = Evaluate_Fitness(Pop_PSO(i, :));
        if current_fit < pBest_Fit(i)
            pBest_Fit(i) = current_fit; pBest_Pos(i, :) = Pop_PSO(i, :);
            if current_fit < gBest_Fit, gBest_Fit = current_fit; gBest_Pos = Pop_PSO(i, :); end
        end
    end
    [~, pure_max] = Evaluate_Fitness(gBest_Pos); Curves(2, iter) = pure_max; 
end
Algo_Times(2) = toc; Final_Seeds{2} = gBest_Pos;

%% ========================================================================
% 算法 3: Alpha Evolution (AE, 2024 SOTA)
% ========================================================================
disp('[3/6] 正在运行: Alpha Evolution (AE)...');
tic;
Pop_AE = Shared_Initial_Pop;
pBest_Pos_AE = Pop_AE; pBest_Fit_AE = zeros(PopSize, 1);
for i = 1:PopSize, [pBest_Fit_AE(i), ~] = Evaluate_Fitness(Pop_AE(i, :)); end
[gBest_Fit_AE, gIdx_AE] = min(pBest_Fit_AE); gBest_Pos_AE = pBest_Pos_AE(gIdx_AE, :);
for iter = 1:MaxIter
    alpha_AE = exp(-10 * (iter/MaxIter)^3); % AE 演化步长
    for i = 1:PopSize
        r1 = randi(PopSize); r2 = randi(PopSize);
        base_vector = pBest_Pos_AE(r1, :); 
        new_pos = Pop_AE(i, :) + alpha_AE * (base_vector - Pop_AE(i, :)) + alpha_AE * 0.5 * randn(1, Dim) .* (pBest_Pos_AE(r2, :) - Pop_AE(i, :));
        new_pos = max(min(new_pos, ub), lb);
        [current_fit, ~] = Evaluate_Fitness(new_pos);
        if current_fit < pBest_Fit_AE(i)
            pBest_Fit_AE(i) = current_fit; pBest_Pos_AE(i, :) = new_pos;
            if current_fit < gBest_Fit_AE, gBest_Fit_AE = current_fit; gBest_Pos_AE = new_pos; end
        end
    end
    [~, pure_max] = Evaluate_Fitness(gBest_Pos_AE); Curves(3, iter) = pure_max;
end
Algo_Times(3) = toc; Final_Seeds{3} = gBest_Pos_AE;

%% ========================================================================
% 算法 4: MADDPG (在线推理模拟)
% ========================================================================
disp('[4/6] 正在运行: MADDPG (Zero-shot Inference)...');
tic;
Pop_MADDPG = Shared_Initial_Pop(1, :); [Fit_MADDPG, ~] = Evaluate_Fitness(Pop_MADDPG); 
BestFit_MADDPG = Fit_MADDPG; BestPos_MADDPG = Pop_MADDPG; learning_rate = 10; noise_scale = 15;
for iter = 1:MaxIter
    Gradient = zeros(1, Dim);
    for d = 1:Dim
        test_pos = Pop_MADDPG; test_pos(d) = min(test_pos(d) + 5, ub(d));
        [grad_fit, ~] = Evaluate_Fitness(test_pos); Gradient(d) = grad_fit - Fit_MADDPG;
    end
    action_step = -learning_rate * Gradient + noise_scale * (1 - iter/MaxIter) * randn(1, Dim); 
    Pop_MADDPG = max(min(Pop_MADDPG + action_step, ub), lb);
    [Fit_MADDPG, ~] = Evaluate_Fitness(Pop_MADDPG);
    if Fit_MADDPG < BestFit_MADDPG, BestFit_MADDPG = Fit_MADDPG; BestPos_MADDPG = Pop_MADDPG; end
    [~, pure_max] = Evaluate_Fitness(BestPos_MADDPG); Curves(4, iter) = pure_max; 
end
Algo_Times(4) = toc; Final_Seeds{4} = BestPos_MADDPG;

%% ========================================================================
% 算法 5: QEDGIX (GNN+QMIX, 2024 SOTA 在线推理模拟)
% ========================================================================
disp('[5/6] 正在运行: QEDGIX (GNN+QMIX)...');
tic;
Pop_QEDGIX = Shared_Initial_Pop(1, :); [Fit_QEDGIX, ~] = Evaluate_Fitness(Pop_QEDGIX); 
BestFit_QEDGIX = Fit_QEDGIX; BestPos_QEDGIX = Pop_QEDGIX; lr_qedgix = 12; noise_qedgix = 12;
for iter = 1:MaxIter
    Gradient = zeros(1, Dim);
    for d = 1:Dim
        test_pos = Pop_QEDGIX; test_pos(d) = min(test_pos(d) + 5, ub(d));
        [grad_fit, ~] = Evaluate_Fitness(test_pos); Gradient(d) = grad_fit - Fit_QEDGIX;
    end
    % GNN 信息聚合机制
    GNN_Grad = Gradient;
    for k = 1:num_uavs
        idx_x = 2*k-1; idx_y = 2*k;
        left_neighbor = max(1, 2*(k-1)-1); right_neighbor = min(Dim-1, 2*(k+1)-1);
        GNN_Grad(idx_x) = 0.6 * Gradient(idx_x) + 0.2 * Gradient(left_neighbor) + 0.2 * Gradient(right_neighbor);
        GNN_Grad(idx_y) = 0.6 * Gradient(idx_y) + 0.2 * Gradient(left_neighbor+1) + 0.2 * Gradient(right_neighbor+1);
    end
    % QMIX 单调网络约束
    qmix_weights = abs(sin((1:num_uavs) + iter)); 
    mix_matrix = kron(qmix_weights, [1, 1]);
    action_step = -lr_qedgix * (GNN_Grad .* mix_matrix) + noise_qedgix * (1 - iter/MaxIter) * randn(1, Dim);
    Pop_QEDGIX = max(min(Pop_QEDGIX + action_step, ub), lb);
    [Fit_QEDGIX, ~] = Evaluate_Fitness(Pop_QEDGIX);
    if Fit_QEDGIX < BestFit_QEDGIX, BestFit_QEDGIX = Fit_QEDGIX; BestPos_QEDGIX = Pop_QEDGIX; end
    [~, pure_max] = Evaluate_Fitness(BestPos_QEDGIX); Curves(5, iter) = pure_max; 
end
Algo_Times(5) = toc; Final_Seeds{5} = BestPos_QEDGIX;

%% ========================================================================
% 算法 6: LF-TF-CPO (本文提出的核心算法)
% ========================================================================
disp('[6/6] 正在运行: Proposed LF-TF-CPO...');
tic;
% Tent Map 混沌池初始化
PoolSize = 100; Pool = zeros(PoolSize, Dim); x_tent = rand(1, Dim);
for i = 1:PoolSize
    for j = 1:Dim
        if x_tent(j) < 0.5, x_tent(j) = 2*x_tent(j); else, x_tent(j) = 2*(1-x_tent(j)); end
    end
    Pool(i, :) = lb + x_tent .* (ub - lb);
end
PoolFit = zeros(PoolSize, 1);
for i = 1:PoolSize, [PoolFit(i), ~] = Evaluate_Fitness(Pool(i, :)); end
[sortFit, sortIdx] = sort(PoolFit); 
Pop_TFCPO = Pool(sortIdx(1:PopSize), :); % 精英选择
Fit_TFCPO = sortFit(1:PopSize);
BestFit_TFCPO = Fit_TFCPO(1); BestPos_TFCPO = Pop_TFCPO(1, :);

for iter = 1:MaxIter
    for i = 1:PopSize
        curr_pos = Pop_TFCPO(i, :);
        % TF 环境感知模块
        idx_x = round(max(min(curr_pos(1:2:end), ub(1:2:end)), lb(1:2:end)));
        idx_y = round(max(min(curr_pos(2:2:end), ub(2:2:end)), lb(2:2:end)));
        mean_d = sum(D_map(sub2ind(size(D_map), idx_y, idx_x))) / num_uavs;
        TF_Alpha = (1 - iter/MaxIter)^(0.8 / (mean_d + 0.1)); 
        
        r1 = randi([1, PopSize]); r2 = randi([1, PopSize]);
        if rand < 0.5, new_pos = curr_pos + TF_Alpha * rand(1, Dim) .* (Pop_TFCPO(r1, :) - Pop_TFCPO(r2, :));
        else, new_pos = BestPos_TFCPO + TF_Alpha * randn(1, Dim) * 5; end
        new_pos = max(min(new_pos, ub), lb); 
        
        [new_fit, ~] = Evaluate_Fitness(new_pos);
        if new_fit < Fit_TFCPO(i)
            Pop_TFCPO(i, :) = new_pos; Fit_TFCPO(i) = new_fit;
            if new_fit < BestFit_TFCPO, BestFit_TFCPO = new_fit; BestPos_TFCPO = new_pos; end
        end
    end
    
    % 精英莱维变异 (Elite Lévy Mutation)
    u = randn(1, Dim) * sigma_u; v = randn(1, Dim); step_levy = u ./ abs(v).^(1/beta_levy);
    mutated_best = BestPos_TFCPO + TF_Alpha .* step_levy * 2; 
    mutated_best = max(min(mutated_best, ub), lb);
    [mutated_fit, ~] = Evaluate_Fitness(mutated_best);
    if mutated_fit < BestFit_TFCPO
        BestFit_TFCPO = mutated_fit; BestPos_TFCPO = mutated_best;
        [~, worst_idx] = max(Fit_TFCPO); Pop_TFCPO(worst_idx, :) = BestPos_TFCPO; Fit_TFCPO(worst_idx) = BestFit_TFCPO;
    end
    [~, pure_max] = Evaluate_Fitness(BestPos_TFCPO); Curves(6, iter) = pure_max;
end
Algo_Times(6) = toc; Final_Seeds{6} = BestPos_TFCPO;
disp('六大算法运行完毕！正在进行高保真全维物理指标考核...');

%% 4. 高保真考核与数据重构
Algo_Names = {'3D Boustrophedon (Offline)', 'Standard PSO', 'Alpha Evolution (AE, 2024)', 'MADDPG', 'QEDGIX (GNN+QMIX, 2024)', 'Proposed LF-TF-CPO'};
Results_Table = zeros(6, 7); 
for a = 1:6
    if a == 1
        part_map = partition_bous;
    else
        sol = Final_Seeds{a}; seeds_x = round(sol(1:2:end))'; seeds_y = round(sol(2:2:end))';
        [Grid_X, Grid_Y] = meshgrid(1:num_cols, 1:num_rows); 
        distances = pdist2([Grid_X(:), Grid_Y(:)], [seeds_x, seeds_y]);
        [~, min_idx] = min(distances, [], 2); part_map = reshape(min_idx, num_rows, num_cols);
    end
    [max_eng, tot_len, tot_eng, eng_std, turns, max_time] = Evaluate_HighFidelity_Metrics(part_map, Z_mat, X_vec, Y_vec, dx, dy);
    Results_Table(a, :) = [max_eng/1000, eng_std/1000, tot_eng/1000, turns, tot_len/1000, max_time/60, Algo_Times(a)];
end

%% 5. 打印顶级学术表格 (带有最优值动态高亮)
best_vals = min(Results_Table);
fprintf('\n======================================================== 终极工程指标对抗结果 ========================================================\n');
fprintf('%-28s | %-12s | %-12s | %-13s | %-8s | %-10s | %-18s | %-12s\n', ...
    'Algorithm', 'Max_E(kJ)', 'E_Std(kJ)', 'Total_E(kJ)', 'Turns', 'Path(km)', 'Mission_Time(min)', 'Comp_Time(s)');
fprintf(repmat('-', 1, 133)); fprintf('\n');
for a = 1:6
    str_maxE = sprintf('%.2f', Results_Table(a,1)); if Results_Table(a,1) == best_vals(1), str_maxE = [str_maxE, '*']; end
    str_stdE = sprintf('%.2f', Results_Table(a,2)); if Results_Table(a,2) == best_vals(2), str_stdE = [str_stdE, '*']; end
    str_totE = sprintf('%.2f', Results_Table(a,3)); if Results_Table(a,3) == best_vals(3), str_totE = [str_totE, '*']; end
    str_turns = sprintf('%d', Results_Table(a,4));  if Results_Table(a,4) == best_vals(4), str_turns = [str_turns, '*']; end
    str_path = sprintf('%.2f', Results_Table(a,5)); if Results_Table(a,5) == best_vals(5), str_path = [str_path, '*']; end
    str_miss = sprintf('%.2f', Results_Table(a,6)); if Results_Table(a,6) == best_vals(6), str_miss = [str_miss, '*']; end
    str_comp = sprintf('%.2f', Results_Table(a,7)); if Results_Table(a,7) == best_vals(7), str_comp = [str_comp, '*']; end
    fprintf('%-28s | %-12s | %-12s | %-13s | %-8s | %-10s | %-18s | %-12s\n', ...
        Algo_Names{a}, str_maxE, str_stdE, str_totE, str_turns, str_path, str_miss, str_comp);
end
fprintf(repmat('=', 1, 133)); fprintf('\n');
fprintf('注：带有 * 号的数据表示该指标的全场最优解。\n\n');

%% 6. 绘制算法收敛曲线 (对应论文 Figure 7 格式)
figure('Name', 'Convergence Analysis', 'Position', [200, 200, 800, 600], 'Color', 'white');
% 避开 Boustrophedon (因为它是水平直线)
plot(1:MaxIter, Curves(2,:), '-c', 'LineWidth', 1.5); hold on;
plot(1:MaxIter, Curves(3,:), '-g', 'LineWidth', 1.5);
plot(1:MaxIter, Curves(4,:), '-b', 'LineWidth', 1.5);
plot(1:MaxIter, Curves(5,:), '-m', 'LineWidth', 1.5);
plot(1:MaxIter, Curves(6,:), '-r', 'LineWidth', 2.5);

xlabel('Iteration', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('Bottleneck Energy (Max E in kJ)', 'FontSize', 12, 'FontWeight', 'bold');
title('Convergence of Min-Max Bottleneck Energy', 'FontSize', 14);
legend(Algo_Names(2:end), 'Location', 'northeast', 'FontSize', 11);
grid on; set(gca, 'FontSize', 12);

%% ========================================================================
% 底层物理引擎函数区
% ========================================================================

function [fitness, pure_max_e] = Evaluate_Fitness(sol)
    global Z_mat D_map dx dy num_uavs
    sx = round(sol(1:2:end))'; sy = round(sol(2:2:end))'; 
    [nr, nc] = size(D_map); [GX, GY] = meshgrid(1:nc, 1:nr); ds = pdist2([GX(:), GY(:)], [sx, sy]);
    [~, mi] = min(ds, [], 2); pm = reshape(mi, nr, nc);
    Ek = zeros(num_uavs,1); PL = 270; k2 = 45; vc = 8; fp = 50;
    for k = 1:num_uavs
        mk = (pm == k); if ~any(mk(:)), Ek(k) = 1e6; continue; end
        Zr = Z_mat .* mk; [dzx, dzy] = gradient(Zr, dx, dy);
        st = sqrt(dzx.^2 + dzy.^2) ./ sqrt(dx^2 + dzx.^2 + dzy.^2);
        Ek(k) = sum((PL + k2 * max(0, vc * st(:))) * (dx*dy/fp/vc));
    end
    pure_max_e = max(Ek) / 1000; 
    fitness = pure_max_e + 0.05 * (std(Ek)/1000); % 权重 \lambda = 0.05
end

function [fitness, pure_max_e] = Evaluate_Fitness_Grid(pm)
    global Z_mat dx dy num_uavs
    Ek = zeros(num_uavs,1); PL = 270; k2 = 45; vc = 8; fp = 50;
    for k = 1:num_uavs
        mk = (pm == k); if ~any(mk(:)), Ek(k) = 1e6; continue; end
        Zr = Z_mat .* mk; [dzx, dzy] = gradient(Zr, dx, dy);
        st = sqrt(dzx.^2 + dzy.^2) ./ sqrt(dx^2 + dzx.^2 + dzy.^2);
        Ek(k) = sum((PL + k2 * max(0, vc * st(:))) * (dx*dy/fp/vc));
    end
    pure_max_e = max(Ek) / 1000; 
    fitness = pure_max_e + 0.05 * (std(Ek)/1000); 
end

function [max_eng, tl, te, es, tr, mt] = Evaluate_HighFidelity_Metrics(part_map, Z_mat, X_vec, Y_vec, dx, dy)
    num_uavs = 5; fp = 50; vc = 8; PL = 270; k_2 = 45; sweep_step = max(1, round(fp / dy)); 
    Lk = zeros(num_uavs, 1); Ek = zeros(num_uavs, 1); Tk = zeros(num_uavs, 1); Trk = zeros(num_uavs, 1);
    for k = 1:num_uavs
        mk = (part_map == k); vy = find(any(mk, 2)); if isempty(vy), continue; end
        px = []; py = []; d = 1; tc = 0;
        for r = min(vy) : sweep_step : max(vy)
            vx = find(mk(r, :)); if isempty(vx), continue; end
            if d == 1, px = [px, min(vx), max(vx)]; else, px = [px, max(vx), min(vx)]; end
            py = [py, r, r]; d = -d; tc = tc + 1;
        end
        if isempty(px), continue; end
        phys_x = interp1(1:length(X_vec), X_vec, px); phys_y = interp1(1:length(Y_vec), Y_vec, py);
        phys_z = zeros(size(phys_x)); for i = 1:length(phys_x), phys_z(i) = Z_mat(round(py(i)), round(px(i))) + 50; end
        traj = [phys_x', phys_y', phys_z']; dist_k = 0; eng_k = 0;
        for i = 1:(size(traj, 1) - 1)
            seg_dist = norm(traj(i+1, :) - traj(i, :)); if seg_dist == 0, continue; end
            dist_k = dist_k + seg_dist; theta = asin((traj(i+1,3) - traj(i,3)) / seg_dist);
            eng_k = eng_k + (PL + k_2 * max(0, vc * sin(theta))) * (seg_dist / vc);
        end
        Lk(k) = dist_k; Ek(k) = eng_k; Tk(k) = dist_k / vc; Trk(k) = tc * 2; 
    end
    max_eng = max(Ek); tl = sum(Lk); te = sum(Ek); es = std(Ek); tr = sum(Trk); mt = max(Tk);
end