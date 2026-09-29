%% fit_temp_bias_raw.m
% 在**原始物理量域**按显式配置的「轴向 + 阶数」拟合变阶温度模型，
% 输出**通用温补系数矩阵**（未拟合轴整行置 0，可直接下发使用）。
%
% 作用域：本脚本在**未做 24 位标定补偿**的原始域拟合/补偿，系数与补偿对象同域，
% 供「24 位置标定前先剔除温漂」以及运行时前端使用。
% （旧的标定域 poly3 路线已从正式工具树删除，仅保留在 Git 历史中。）
%
% 模型:  b(T) = c0 + c1*dT + ... + c_o*dT^o    （各轴阶数 o 单独配置）
%        dT = T - Tref,  Tref = 首个温度采样点（基准温度）
%        补偿时只减去温度相关项 c1*dT + ... + c_o*dT^o，保留 c0（不补 0 阶）
%
% 配置:  cfg.axisOrder = [ax ay az gx gy gz]，取值 0..5 的整数：
%          0    = 该轴**不拟合** → 输出矩阵对应行整行为 0，
%                 下游 polyval 得到 0，等价于「该轴不做温补」；
%          1..5 = 拟合到该阶。
%        系数矩阵统一补齐到 cfg.maxOrder=5 阶（右对齐, polyfit 约定
%        [c5 c4 c3 c2 c1 c0]，未使用的高阶项补 0）。
%
% 重要:  cfg.axisOrder 是**唯一权威配置**。fit_temp_order_selection.m 的推荐
%        阶数只作参考打印，不覆盖本配置。
%        下游 24 位置标定只使用 c1..c_o（不用 c0）——c0 是基准温度 Tref 下的
%        常值零偏（加计内还含重力投影），属于 24 位置标定要求解的未知量。
%
% 输入:  <sessionDir>/imu.csv  (长时静态, ~100 Hz)
% 输出:
%   data/calib24/temp_coeffs_raw.mat / .csv        通用温补系数矩阵（未拟合轴行=0）
%   <sessionDir>/imu_tempcomp_multiorder.csv       全分辨率温补后数据(原始域)
%   <sessionDir>/fig_raw_tempcomp_compare.png      温补前后块均值对比
%   <sessionDir>/fig_raw_temp_fit.png              变阶温度模型拟合效果

clear; clc; close all;

%% ======================== USER CONFIG =============================
projRoot    = fileparts(fileparts(mfilename('fullpath')));
sessionName = '20260926005735';                                  % 拟合数据（长时静态）
sessionDir  = fullfile(projRoot, 'data', 'decoded', sessionName);
imuFile     = fullfile(sessionDir, 'imu.csv');
outDir      = fullfile(projRoot, 'data', 'calib24');             % 系数输出

% 要拟合的轴向与对应阶数：[ax ay az gx gy gz]
%   0    = 不拟合（输出该行整行为 0）；1..5 = 拟合阶数
cfg.axisOrder = [0 0 3 5 5 0];

cfg.decim    = 50;      % 拟合降采样因子（每 decim 点取 1 点参与最小二乘）
cfg.blockSec = 600;     % 块均值对比窗口 [s]
cfg.maxOrder = 5;       % 系数矩阵列数 = maxOrder + 1

outCsv    = fullfile(outDir, 'temp_coeffs_raw.csv');
outMat    = fullfile(outDir, 'temp_coeffs_raw.mat');
outImuC   = fullfile(sessionDir, 'imu_tempcomp_multiorder.csv');
outFigCmp = fullfile(sessionDir, 'fig_raw_tempcomp_compare.png');
outFigFit = fullfile(sessionDir, 'fig_raw_temp_fit.png');

assert(numel(cfg.axisOrder) == 6, 'cfg.axisOrder 必须是 6 个元素 [ax ay az gx gy gz]');
assert(all(cfg.axisOrder >= 0 & cfg.axisOrder <= cfg.maxOrder & ...
           cfg.axisOrder == fix(cfg.axisOrder)), ...
       'cfg.axisOrder 只能是 0..%d 的整数', cfg.maxOrder);

%% ---------------- 列索引（与 imu.csv 表头对应）----------------
% 1: sample, 6: time_s, 7: dt_s
% 15: ax_m_s2, 16: ay_m_s2, 17: az_m_s2, 18: temp_deg_c,
% 19: gx_deg_h, 20: gy_deg_h, 21: gz_deg_h
colTemp = 18;
colAxes = [15 16 17 19 20 21];
axNames = {'ax(m/s2)','ay(m/s2)','az(m/s2)','gx(deg/h)','gy(deg/h)','gz(deg/h)'};

%% ---------------- 读取原始数据 ----------------
fprintf('读取数据: %s\n', imuFile);
tic;
M = readmatrix(imuFile, 'NumHeaderLines', 1);
fprintf('读取完成: %d 行, %.1f s\n', size(M,1), toc);

valid = all(isfinite(M(:, [1 6 7 colTemp colAxes])), 2);
assert(all(valid), '原始数据含非有限值，请先审查');
M = M(valid, :);

N    = size(M, 1);
temp = M(:, colTemp);
RAW  = M(:, colAxes);            % 原始域数据（未做任何标定补偿）
sourceInfo = dir(imuFile);
sourceTimeRange = [M(1,6), M(end,6)];
fprintf('有效样本: %d (%.2f h)\n', N, (M(N,6)-M(1,6))/3600);

nAx = size(RAW, 2);
assert(numel(cfg.axisOrder) == nAx, 'cfg.axisOrder 长度必须等于轴数 %d', nAx);

%% ---------------- 选阶参考（只打印，不覆盖配置）----------------
selectionFile = fullfile(outDir, 'temp_order_selection.mat');
if isfile(selectionFile)
    sel = load(selectionFile);
    if isfield(sel, 'bestOrder') && numel(sel.bestOrder) == nAx
        fprintf('\n【参考】选阶推荐 [ax ay az gx gy gz] = %s\n', mat2str(sel.bestOrder(:)'));
        fprintf('       本次配置 cfg.axisOrder          = %s\n', mat2str(cfg.axisOrder(:)'));
        if ~isequal(sel.bestOrder(:)', cfg.axisOrder(:)')
            fprintf('       两者不同，以 cfg.axisOrder 为准。\n');
        end
    end
    if isfield(sel, 'sourceFile') && ~strcmp(sel.sourceFile, imuFile)
        warning('选阶文件来自其它数据源（%s），仅作参考。', sel.sourceFile);
    end
    if isfield(sel, 'decim') && sel.decim ~= cfg.decim
        warning('选阶 decim=%d 与本次 decim=%d 不同（仅影响拟合子集）。', sel.decim, cfg.decim);
    end
else
    fprintf('\n未找到 %s，跳过选阶参考（配置以 cfg.axisOrder 为准）。\n', selectionFile);
end

%% ---------------- 降采样拟合变阶温度模型 ----------------
if cfg.decim > 1
    idx     = 1:cfg.decim:N;
    tempFit = temp(idx);
    YFit    = RAW(idx, :);
    fprintf('\n拟合降采样: %d / %d 样本 (1/%d)\n', numel(tempFit), N, cfg.decim);
else
    tempFit = temp;  YFit = RAW;
end

T0 = tempFit(1);                  % 基准温度 = 首个温度采样点
dT = tempFit - T0;
fprintf('基准温度 T0 = %.4f °C, 温度范围 [%.2f, %.2f] °C\n', T0, min(temp), max(temp));

coef   = zeros(nAx, cfg.maxOrder+1);   % 统一 (maxOrder+1) 列, [c5 c4 c3 c2 c1 c0]
rmsRes = nan(nAx, 1);

fprintf('\n========== 原始域变阶温度模型拟合结果 (maxOrder=%d) ==========\n', cfg.maxOrder);
fprintf('模型: b(T) = c0 + c1*dT + ... + c_o*dT^o,  dT = T - %.4f\n\n', T0);
fprintf('%-12s %5s %14s %14s %14s %14s %14s %14s %12s\n', ...
        'axis', 'ord', 'c0', 'c1', 'c2', 'c3', 'c4', 'c5', 'RMS残差');
for k = 1:nAx
    o = cfg.axisOrder(k);
    if o == 0
        fprintf('%-12s %5d   （不拟合：输出行整行置 0）\n', axNames{k}, 0);
        continue;
    end
    Xk = zeros(numel(dT), o+1);
    for m = o:-1:0
        Xk(:, o-m+1) = dT.^m;              % 列顺序 [c_o ... c1 c0]
    end
    p = Xk \ YFit(:,k);                    % 最小二乘
    coef(k, (cfg.maxOrder+1-o):(cfg.maxOrder+1)) = p';   % 右对齐写入统一列布局
    r = YFit(:,k) - Xk*p;
    rmsRes(k) = sqrt(mean(r.^2));
    fprintf('%-12s %5d %14.6e %14.6e %14.6e %14.6e %14.6e %14.6e %12.4f\n', ...
            axNames{k}, o, coef(k,6), coef(k,5), coef(k,4), ...
            coef(k,3), coef(k,2), coef(k,1), rmsRes(k));
end

% 补偿即用形式：常数项显式置 0，规避 polyval 把 [c5..c1] 当成低一阶多项式
TCpoly = [coef(:, 1:end-1), zeros(nAx, 1)];   % [c5 c4 c3 c2 c1 0]
ord    = cfg.axisOrder;

%% ---------------- 温漂补偿（全分辨率，不补 0 阶）----------------
% 只减温度相关项 c1*dT + ... + c_o*dT^o（各轴用自己的阶数），保留常数零偏 c0；
% 未拟合轴整行为 0，此处自然得到 drift = 0。
dTf   = temp - T0;
drift = zeros(N, nAx);
for m = 1:cfg.maxOrder
    drift = drift + dTf.^m * coef(:, cfg.maxOrder+1-m)';   % coef(:, 6-m) 即该轴 c_m
end
Ytc = RAW - drift;
fprintf('\n温漂补偿完成(全分辨率): b_tc = b_raw - [c1*dT + ... + c_o*dT^o]\n');

%% ---------------- 两级对比: 原始 / 温补后 ----------------
block = max(1, round(cfg.blockSec * 100));    % 100 Hz 标称采样率
nb    = floor(N / block);
tH    = ((1:nb)' - 0.5) * cfg.blockSec / 3600;
bm    = @(v) mean(reshape(v(1:nb*block), block, nb), 1)';

stages     = {RAW, Ytc};
stageNames = {'原始 raw', '温补后 raw+TC'};

fprintf('\n========== 温漂补偿前后: %d s 块均值标准差 ==========\n', cfg.blockSec);
fprintf('%-12s %16s %16s %14s\n', 'axis', stageNames{1}, stageNames{2}, '改善');
statTable = zeros(nAx, 2);
for k = 1:nAx
    for s = 1:2
        m = bm(stages{s}(:,k));
        statTable(k,s) = std(m - median(m));
    end
    fprintf('%-12s %16.5f %16.5f %13.1f%%\n', axNames{k}, ...
        statTable(k,1), statTable(k,2), ...
        100*(1 - statTable(k,2)/max(statTable(k,1), eps)));
end

figure('Name','温漂补偿前后块均值对比','Position',[60 60 1400 760]);
for k = 1:nAx
    subplot(2,3,k); hold on;
    m0 = bm(RAW(:,k));  m1 = bm(Ytc(:,k));
    plot(tH, m0 - median(m0), '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.1);
    plot(tH, m1 - median(m1), '-', 'Color', [0.85 0.25 0.20], 'LineWidth', 1.1);
    title(sprintf('%s (order %d)', axNames{k}, ord(k)));
    xlabel('时间 (h)'); ylabel('块均值残差 (去中位数)');
    grid on;
    if k == 1, legend(stageNames, 'Location', 'best'); end
end
sgtitle(sprintf('原始域温补前后 (%d s 块均值, 各自去中位数)', cfg.blockSec));
saveas(gcf, outFigCmp);

%% ---------------- 温度模型拟合效果预览图 ----------------
figure('Name','原始域变阶温度模型拟合','Position',[80 80 1200 700]);
for k = 1:nAx
    subplot(2,3,k);
    if ord(k) == 0
        text(0.5, 0.5, sprintf('%s\n未拟合（该行置 0）', axNames{k}), ...
             'Units','normalized','HorizontalAlignment','center');
        title(sprintf('%s (order 0, 不补偿)', axNames{k}));
        axis off;
        continue;
    end
    scatter(dT, YFit(:,k), 2, '.', 'MarkerFaceAlpha',0.15); hold on;
    Ts = linspace(min(dT), max(dT), 300)';   % 列向量，保证维度匹配
    plot(Ts, polyval(coef(k,:), Ts), 'r-', 'LineWidth', 1.5);
    title(sprintf('%s (order %d, RMS %.3f)', axNames{k}, ord(k), rmsRes(k)));
    xlabel('dT = T - T0 (\circC)'); grid on;
end
sgtitle(sprintf('原始域 零偏温度模型拟合 (cfg.axisOrder = [%s])', num2str(ord, '%d ')));
saveas(gcf, outFigFit);

%% ---------------- 输出 6 轴系数 ----------------
coefTable = array2table([ord(:), coef(:,6), coef(:,5), coef(:,4), ...
                         coef(:,3), coef(:,2), coef(:,1), rmsRes], ...
    'VariableNames', {'order','c0','c1','c2','c3','c4','c5','rms_residual'}, ...
    'RowNames', {'ax','ay','az','gx','gy','gz'});
fprintf('\n通用温补系数矩阵 (未拟合轴整行为 0, dT = T - T0):\n');
disp(coefTable);

if ~exist(outDir, 'dir'); mkdir(outDir); end

Tout = table(axNames', ord(:), coef(:,6), coef(:,5), coef(:,4), ...
             coef(:,3), coef(:,2), coef(:,1), rmsRes, ...
    'VariableNames', {'axis','order','c0','c1','c2','c3','c4','c5','rms_residual'});
writetable(Tout, outCsv);

Tref = T0;
domain = 'raw';
Tmin = min(temp); Tmax = max(temp);
sourceFile = imuFile; sourceBytes = sourceInfo.bytes; sourceSamples = N;
units = {'m/s^2','m/s^2','m/s^2','deg/h','deg/h','deg/h'};
decim = cfg.decim;
maxOrder = cfg.maxOrder;
axisOrder = cfg.axisOrder;
coefficientConvention = ['descending powers; c0 fitted but excluded from compensation; ' ...
                         'unfitted axes (axisOrder==0) have all-zero rows'];
save(outMat, 'coef', 'TCpoly', 'ord', 'axisOrder', 'maxOrder', 'Tref', 'Tmin', 'Tmax', ...
    'axNames', 'decim', 'domain', 'sourceFile', 'sourceBytes', 'sourceSamples', ...
    'sourceTimeRange', 'units', 'selectionFile', 'coefficientConvention');

fprintf('系数已保存: %s\n', outCsv);
fprintf('系数已保存: %s\n', outMat);

%% ---------------- 导出全分辨率温补后数据 ----------------
% 列: sample,time_s,dt_s,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,gx_deg_h,gy_deg_h,gz_deg_h
% 注意: 这是"原始域温补后"数据，未做 24 位标定补偿（保留 c0 与标度/非正交误差）。
%       若要换算成物理量，需再套用 24 位置标定参数: a_cal = Ca*(a_tc - ba)
fprintf('导出温补后全量数据: %s\n', outImuC);
tic;
E = table(M(:,1), M(:,6), M(:,7), ...
          Ytc(:,1), Ytc(:,2), Ytc(:,3), temp, ...
          Ytc(:,4), Ytc(:,5), Ytc(:,6), ...
    'VariableNames', {'sample','time_s','dt_s', ...
                      'ax_m_s2','ay_m_s2','az_m_s2','temp_deg_c', ...
                      'gx_deg_h','gy_deg_h','gz_deg_h'});
writetable(E, outImuC);
fprintf('导出完成: %d 行, %.1f s\n', height(E), toc);

%% ---------------- 交接信息 ----------------
enableStr = strjoin(arrayfun(@(v) sprintf('%d', v > 0), cfg.axisOrder(:)', ...
                             'UniformOutput', false), ',');
fprintf('\n下游 24 位置标定只使用 c1..c_o（不补 c0）。\n');
fprintf('步骤③ 三方案 Allan 对比请用:  --coeff %s  --enable %s\n', ...
        'data\\calib24\\temp_coeffs_raw.csv', enableStr);
