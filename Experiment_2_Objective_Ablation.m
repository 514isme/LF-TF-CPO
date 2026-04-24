% =========================================================================
% LF-TF-CPO 算法专项机制验证实验 (对应论文 Section 4.3 & Figure 8)
% 核心对抗：传统总能耗最小化 (Min-Sum) vs 本文瓶颈能耗极小化 (Min-Max)
% 实验目标：揭示传统 Min-Sum 在复杂环境中导致个体过载坠毁的致命缺陷
% 核心技术：引入 PRNG 时空锚点，确保 Min-Max 结果与主干对比实验绝对对齐
% =========================================================================

clear; clc; close all;

%% 1. 环境底座加载
disp('正在加载环境与地形矩阵 (Mountain_Env_3km.mat)...');
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
num_uavs = 5;

%% 2. 实验全局参数
Dim = 10; PopSize = 15; MaxIter = 80;
lb = zeros(1, Dim); ub = zeros(1, Dim);
lb(1:2:end) = round(length(X_vec) * 0.05); lb(2:2:end) = round(length(Y_vec) * 0.05);
ub(1:2:end) = round(length(X_vec) * 0.95); ub(2:2:end) = round(length(Y_vec) * 0.95);

beta_levy = 1.5;
sigma_u = (gamma(1+beta_levy)*sin(pi*beta_levy/2)/(gamma((1+beta_levy)/2)*beta_levy*2^((beta_levy-1)/2)))^(1/beta_levy);

Curves_MaxE = zeros(2, MaxIter);
Curves_TotE = zeros(2, MaxIter);
Final_Results = zeros(2, 5); 
Individual_Energies = zeros(2, num_uavs); 

%% 3. PRNG 时空锚点提取 (全盘复刻对比实验消耗链)
disp('正在推演随机熵流失，锁定主对比实验的时空锚点...');
Sync_RNG_State();
Target_RNG_State = rng; 
disp('时空锚点已锁定！开始执行目标函数对抗验证...');

%% 4. 执行两套不同价值观的优化内核
Scheme_Names = {'Traditional Min-Sum (Total Energy)', 'Proposed Min-Max (Bottleneck Energy)'};

for scheme = 1:2
    fprintf('\n------------------------------------------------------\n');
    fprintf('正在运行 %s 架构...\n', Scheme_Names{scheme});
    tic;
    
    % 【绝对公平】：两套方案都从同一个时空锚点起跑
    rng(Target_RNG_State); 
    
    % 共享的精英混沌池生成 (消耗相应的 rand，保持主序列推进)
    PoolSize = 100; Pool = zeros(PoolSize, Dim); x_tent = rand(1, Dim);
    for i = 1:PoolSize
        for j = 1:Dim
            if x_tent(j) < 0.5, x_tent(j) = 2*x_tent(j); else, x_tent(j) = 2*(1-x_tent(j)); end
        end
        Pool(i, :) = lb + x_tent .* (ub - lb);
    end
    
    % 根据不同的目标函数 (Scheme) 评估初始池
    PoolFit = zeros(PoolSize, 1);
    for i = 1:PoolSize
        [fit_minmax, ~, pure_sum, ~] = Evaluate_Fitness_Dual(Pool(i, :));
        if scheme == 1
            PoolFit(i) = pure_sum; % Scheme 1 贪婪追求总能耗极小化
        else
            PoolFit(i) = fit_minmax; % Scheme 2 追求瓶颈极小化与方差平滑
        end
    end
    
    [sortFit, sortIdx] = sort(PoolFit); 
    Pop = Pool(sortIdx(1:PopSize), :); 
    Fit = sortFit(1:PopSize);
    BestFit = Fit(1); BestPos = Pop(1, :);
    
    % 主循环 (完美复刻 LF-TF-CPO 游走逻辑)
    for iter = 1:MaxIter
        for i = 1:PopSize
            curr_pos = Pop(i, :);
            
            idx_x = round(max(min(curr_pos(1:2:end), ub(1:2:end)), lb(1:2:end)));
            idx_y = round(max(min(curr_pos(2:2:end), ub(2:2:end)), lb(2:2:end)));
            mean_d = sum(D_map(sub2ind(size(D_map), idx_y, idx_x))) / num_uavs;
            TF_Alpha = (1 - iter/MaxIter)^(0.8 / (mean_d + 0.1)); 
            
            r1 = randi([1, PopSize]); r2 = randi([1, PopSize]);
            if rand < 0.5
                new_pos = curr_pos + TF_Alpha * rand(1, Dim) .* (Pop(r1, :) - Pop(r2, :));
            else
                new_pos = BestPos + TF_Alpha * randn(1, Dim) * 5; 
            end
            new_pos = max(min(new_pos, ub), lb); 
            
            [new_minmax, ~, new_pure_sum, ~] = Evaluate_Fitness_Dual(new_pos);
            if scheme == 1, new_fit = new_pure_sum; else, new_fit = new_minmax; end
            
            if new_fit < Fit(i)
                Pop(i, :) = new_pos; Fit(i) = new_fit;
                if new_fit < BestFit, BestFit = new_fit; BestPos = new_pos; end
            end
        end
        
        % 莱维飞行变异
        u = randn(1, Dim) * sigma_u; v = randn(1, Dim); step_levy = u ./ abs(v).^(1/beta_levy);
        mutated_best = BestPos + TF_Alpha .* step_levy * 2; 
        mutated_best = max(min(mutated_best, ub), lb);
        
        [mut_minmax, ~, mut_pure_sum, ~] = Evaluate_Fitness_Dual(mutated_best);
        if scheme == 1, mut_fit = mut_pure_sum; else, mut_fit = mut_minmax; end
        
        if mut_fit < BestFit
            BestFit = mut_fit; BestPos = mutated_best;
            [~, worst_idx] = max(Fit); Pop(worst_idx, :) = BestPos; Fit(worst_idx) = BestFit;
        end
        
        % 记录真实物理数据用于绘图分析
        [~, curr_pure_max, curr_pure_sum, ~] = Evaluate_Fitness_Dual(BestPos);
        Curves_MaxE(scheme, iter) = curr_pure_max;
        Curves_TotE(scheme, iter) = curr_pure_sum;
    end
    
    comp_time = toc;
    
    % 高保真指标提取
    sol = BestPos; seeds_x = round(sol(1:2:end))'; seeds_y = round(sol(2:2:end))';
    [nr, nc] = size(D_map); [Grid_X, Grid_Y] = meshgrid(1:nc, 1:nr); distances = pdist2([Grid_X(:), Grid_Y(:)], [seeds_x, seeds_y]);
    [~, min_idx] = min(distances, [], 2); part_map = reshape(min_idx, nr, nc);
    
    [max_eng, tl, te, es, tr, mt, Ek_array] = Evaluate_HighFidelity_Metrics(part_map, Z_mat, X_vec, Y_vec, dx, dy);
    
    Final_Results(scheme, :) = [max_eng/1000, te/1000, es/1000, tr, comp_time];
    Individual_Energies(scheme, :) = Ek_array / 1000;
end

%% 5. 打印专项对比学术表
fprintf('\n================== 目标函数专项消融对比表 (全文数据闭环版) ==================\n');
fprintf('%-38s | %-10s | %-12s | %-12s | %-8s\n','Optimization Objective','Max_E(kJ)','Total_E(kJ)','E_Std(kJ)','Turns');
fprintf(repmat('-',1,88)); fprintf('\n');
for g = 1:2
    fprintf('%-38s | %-10.2f | %-12.2f | %-12.2f | %-8d\n', Scheme_Names{g}, Final_Results(g,1), Final_Results(g,2), Final_Results(g,3), Final_Results(g,4));
end
fprintf(repmat('=',1,88)); fprintf('\n');

%% 6. 绘制高逼格可视化对比图
% 图 1: 个体能耗分布柱状图 (对应揭示 Crash 风险)
figure('Name', 'Individual UAV Energy Distribution', 'Color', 'white', 'Position', [100 100 700 450]);
bar_data = [Individual_Energies(1, :); Individual_Energies(2, :)]';
b = bar(bar_data, 'grouped');
b(1).FaceColor = [0.5 0.5 0.5]; 
b(2).FaceColor = [0.85 0.325 0.098]; 

% 绘制 950kJ 坠毁红线 (严密呼应审稿人关切的Capacity Threshold)
safe_threshold = 950;
yline(safe_threshold, 'r--', 'LineWidth', 2.5, 'Label', 'Capacity Threshold (Crash Risk Line)', 'LabelHorizontalAlignment', 'left', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');

title('Energy Distribution among UAV Swarm', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('UAV ID (1 to 5)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('True Energy Consumption (kJ)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Traditional Min-Sum', 'Proposed Min-Max', 'Location', 'northwest', 'FontSize', 11);
grid on; set(gca, 'FontSize', 11);

% 图 2: 双 Y 轴轨迹对冲图 (揭示 Pareto Trade-off)
figure('Name', 'Pareto Indicator Trajectories', 'Color', 'white', 'Position', [850 100 700 450]);
yyaxis left;
plot(1:MaxIter, Curves_MaxE(1, :), 'k--', 'LineWidth', 2); hold on;
plot(1:MaxIter, Curves_MaxE(2, :), '-o', 'LineWidth', 2.5, 'Color', [0.85 0.325 0.098], 'MarkerIndices', 1:10:MaxIter);
ylabel('Bottleneck Energy Max_E (kJ)', 'FontWeight', 'bold');
set(gca, 'YColor', [0.85 0.325 0.098]);

yyaxis right;
plot(1:MaxIter, Curves_TotE(1, :), '-s', 'LineWidth', 2.5, 'Color', [0 0.447 0.741], 'MarkerIndices', 5:10:MaxIter); hold on;
plot(1:MaxIter, Curves_TotE(2, :), 'k:', 'LineWidth', 2);
ylabel('Total Swarm Energy (kJ)', 'FontWeight', 'bold');
set(gca, 'YColor', [0 0.447 0.741]);

title('Divergence of Objectives: Max_E vs Total_E', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Iteration', 'FontWeight', 'bold');
legend('Min-Sum: Max_E (Crash Risk)', 'Min-Max: Max_E (Safe)', 'Min-Sum: Total_E (Greedy)', 'Min-Max: Total_E (Trade-off)', 'Location', 'best', 'FontSize', 11);
grid on; set(gca, 'FontSize', 11);

%% ========================================================================
% 辅助同步函数 (原封不动还原第一组对比实验的伪随机消耗)
% ========================================================================
function Sync_RNG_State()
    rng(2026); PopSize = 15; Dim = 10; MaxIter = 80;
    rand(PopSize, Dim); % 初始种群
    for iter = 1:MaxIter, for i = 1:PopSize, rand(1, Dim); rand(1, Dim); end, end % PSO
    for iter = 1:MaxIter, for i = 1:PopSize, randi(PopSize); randi(PopSize); randn(1, Dim); end, end % AE
    for iter = 1:MaxIter, randn(1, Dim); end % MADDPG
    for iter = 1:MaxIter, randn(1, Dim); end % QEDGIX
end

%% ========================================================================
% 评估引擎区
% ========================================================================
function [fit_minmax, pure_max_e, pure_sum_e, std_e] = Evaluate_Fitness_Dual(sol)
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
    pure_sum_e = sum(Ek) / 1000;
    std_e = std(Ek) / 1000;
    fit_minmax = pure_max_e + 0.05 * std_e;
end

function [max_eng, tl, te, es, tr, mt, Ek_array] = Evaluate_HighFidelity_Metrics(part_map, Z_mat, X_vec, Y_vec, dx, dy)
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
    Ek_array = Ek; 
end