%% fit_temp_bias_poly3.m
% 静态数据 3 阶温度模型拟合 IMU 零偏（原始数据先用 24 位置标定结果补偿系统误差）
%
% 流程:
%   1) 读取 24 位置标定结果 calib24_result.mat (struct result)
%        加速度计: a_cal = Ca * (a_raw - ba)      [m/s^2]
%        陀螺仪  : g_cal = g_raw - gyroBiasMean   [deg/h, 仅零偏补偿]
%   2) 3 阶温度模型拟合零偏残差:
%        b(T) = c0 + c1*dT + c2*dT^2 + c3*dT^3
%        dT = T - T0,  T0 = 首个温度采样点（基准温度）
%   3) 各轴独立最小二乘求解，输出 6 轴 (ax,ay,az,gx,gy,gz) 系数列表
%   4) 三级对比: 原始 raw / 24位标定补偿 cal / 标定+温漂补偿 cal+temp
%      (10 分钟块均值时序图 + 标准差统计)
%   5) 导出全量"标定+温漂补偿"数据 imu_compensated.csv
%      (供 tools/allan_compare_before_after.py 做 Allan 前后对比)
%
% 数据: data/decoded/20260926005735/imu.csv (21h 室外静态, ~100 Hz)

clear; clc; close all;

%% ---------------- 路径 ----------------
dataDir   = 'C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260926005735';
imuFile   = fullfile(dataDir, 'imu.csv');
calibFile = 'C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\calib24_result.mat';

% 降采样因子：仅用于温度模型拟合，加速最小二乘且不影响精度
decim = 50;                     % 设为 1 使用全部数据拟合

outCsv   = fullfile(dataDir, 'temp_bias_poly3_coeffs.csv');
outMat   = fullfile(dataDir, 'temp_bias_poly3_coeffs.mat');
outImuC  = fullfile(dataDir, 'imu_compensated.csv');   % 全量补偿后数据

% 块均值窗口(对比图用)
blockSec = 600;                 % 10 分钟

%% ---------------- 列索引（与 imu.csv 表头对应）----------------
% 1: sample, 6: time_s, 7: dt_s
% 15: ax_m_s2, 16: ay_m_s2, 17: az_m_s2, 18: temp_deg_c,
% 19: gx_deg_h, 20: gy_deg_h, 21: gz_deg_h
colTemp = 18;
colAxes = [15 16 17 19 20 21];   % ax ay az gx gy gz
axNames = {'ax(m/s2)','ay(m/s2)','az(m/s2)','gx(deg/h)','gy(deg/h)','gz(deg/h)'};

%% ---------------- 读取 24 位置标定结果 ----------------
fprintf('读取标定结果: %s\n', calibFile);
cal = load(calibFile);
if isstruct(cal) && isfield(cal, 'result')
    cal = cal.result;           % 解包 struct result
end
ba   = cal.ba(:)';              % 加速度计零偏 (m/s^2),      1x3
Ca   = cal.Ca;                  % 加速度计补偿矩阵 raw->cal, 3x3
gb   = cal.gyroBiasMean_deg_h(:)';   % 陀螺零偏 (deg/h),     1x3

fprintf('  加速度计零偏 ba        = [%9.4f %9.4f %9.4f] m/s^2\n', ba);
fprintf('  加速度计补偿阵 Ca 对角 = [%9.6f %9.6f %9.6f]\n', diag(Ca));
fprintf('  陀螺零偏 (24位均值)    = [%10.1f %10.1f %10.1f] deg/h\n', gb);

%% ---------------- 读取原始数据 ----------------
fprintf('读取数据: %s\n', imuFile);
tic;
M = readmatrix(imuFile, 'NumHeaderLines', 1);
fprintf('读取完成: %d 行, 耗时 %.1f s\n', size(M,1), toc);

% 去除 NaN / 无效行
valid = all(~isnan(M(:, [colTemp, colAxes])), 2);
M = M(valid, :);

N     = size(M, 1);
temp  = M(:, colTemp);
RAW   = M(:, colAxes);           % N x 6 原始数据（全程分辨率）
fprintf('有效样本: %d (%.2f h)\n', N, (M(N,6)-M(1,6))/3600);

%% ---------------- 24 位置标定补偿（系统误差, 全分辨率）----------------
aRaw = RAW(:, 1:3);              % N x 3, m/s^2
gRaw = RAW(:, 4:6);              % N x 3, deg/h

aCal = (aRaw - ba) * Ca.';       % a_cal = Ca * (a_raw - ba)
gCal = gRaw - gb;                % g_cal = g_raw - gyroBias
Y    = [aCal, gCal];             % N x 6  24位标定补偿后
clear aRaw gRaw aCal gCal
fprintf('24位标定补偿完成: a_cal = Ca*(a_raw - ba), g_cal = g_raw - gb\n');

%% ---------------- 降采样拟合 3 阶温度模型 ----------------
if decim > 1
    idx = 1:decim:N;
    tempFit = temp(idx);
    YFit    = Y(idx, :);
    fprintf('拟合降采样: %d / %d 样本 (1/%d)\n', numel(tempFit), N, decim);
else
    tempFit = temp;  YFit = Y;
end

T0 = tempFit(1);                  % 基准温度 = 首个温度采样点
dT = tempFit - T0;
fprintf('基准温度 T0 = %.4f °C, 温度范围 [%.2f, %.2f] °C\n', ...
        T0, min(temp), max(temp));

X = [dT.^3, dT.^2, dT, ones(size(dT))];   % [c3 c2 c1 c0] 设计矩阵

nAx    = size(YFit, 2);
coef   = zeros(nAx, 4);           % [c3 c2 c1 c0] 每轴
rmsRes = zeros(nAx, 1);

fprintf('\n========== 3 阶温度模型零偏拟合结果（标定补偿后） ==========\n');
fprintf('模型: b(T) = c0 + c1*dT + c2*dT^2 + c3*dT^3,  dT = T - %.4f\n\n', T0);
fprintf('%-12s %15s %15s %15s %15s %12s\n', ...
        'axis', 'c0', 'c1', 'c2', 'c3', 'RMS残差');
for k = 1:nAx
    p = X \ YFit(:,k);            % 最小二乘 (等价 polyfit(dT, y, 3))
    coef(k,:) = p';
    r = YFit(:,k) - X*p;
    rmsRes(k) = sqrt(mean(r.^2));
    fprintf('%-12s %15.6e %15.6e %15.6e %15.6e %12.4f\n', ...
            axNames{k}, p(4), p(3), p(2), p(1), rmsRes(k));
end

%% ---------------- 温漂补偿（全分辨率）----------------
% 去掉温度相关项 c1*dT + c2*dT^2 + c3*dT^3，保留常数零偏 c0
dTf  = temp - T0;
Ytc  = Y - (dTf.^3*coef(:,1)' + dTf.^2*coef(:,2)' + dTf*coef(:,3)');
fprintf('\n温漂补偿完成(全分辨率): b_comp = b_cal - [c1*dT + c2*dT^2 + c3*dT^3]\n');

%% ---------------- 三级对比: 原始 / 标定 / 标定+温漂 ----------------
block = max(1, round(blockSec * 100));    % 100 Hz 标称采样率
nb    = floor(N / block);
tH    = ((1:nb)' - 0.5) * blockSec / 3600;   % 小时
bm    = @(v) mean(reshape(v(1:nb*block), block, nb), 1)';   % N x 1 -> nb x 1

stages = {RAW, Y, Ytc};
stageNames = {'原始 raw', '24位标定', '标定+温漂'};

% 块均值标准差统计（各 stage 先去自身中位数）
fprintf('\n========== 三级对比: %d s 块均值标准差 ==========\n', blockSec);
fprintf('%-12s', 'axis');
fprintf('%16s', stageNames{1}, stageNames{2}, stageNames{3});
fprintf('%14s\n', '改善(标定+温漂 vs 标定)');
statTable = zeros(nAx, 3);
for k = 1:nAx
    for s = 1:3
        m = bm(stages{s}(:,k));
        statTable(k,s) = std(m - median(m));
    end
    fprintf('%-12s %16.5f %16.5f %16.5f %13.1f%%\n', axNames{k}, ...
        statTable(k,1), statTable(k,2), statTable(k,3), ...
        100*(1 - statTable(k,3)/max(statTable(k,2), eps)));
end

figure('Name','三级对比: 原始/标定/标定+温漂','Position',[60 60 1400 760]);
for k = 1:nAx
    subplot(2,3,k); hold on;
    cols = [0.55 0.55 0.55; 0.20 0.45 0.85; 0.85 0.25 0.20];
    for s = 1:3
        m = bm(stages{s}(:,k));
        plot(tH, m - median(m), '-', 'Color', cols(s,:), 'LineWidth', 1.1);
    end
    title(axNames{k});
    xlabel('时间 (h)');
    ylabel('块均值残差 (去中位数)');
    grid on;
    if k == 1, legend(stageNames, 'Location', 'best'); end
end
sgtitle(sprintf('三级对比 (%d s 块均值, 各自去中位数)', blockSec));

%% ---------------- 温度模型拟合效果预览图 ----------------
figure('Name','3阶温度模型零偏拟合','Position',[80 80 1200 700]);
for k = 1:nAx
    subplot(2,3,k);
    scatter(dT, YFit(:,k), 2, '.', 'MarkerFaceAlpha',0.15); hold on;
    Ts = linspace(min(dT), max(dT), 300)';   % 列向量，保证维度匹配
    plot(Ts, polyval(coef(k,:), Ts), 'r-', 'LineWidth', 1.5);
    title(sprintf('%s (RMS %.3f)', axNames{k}, rmsRes(k)));
    xlabel('dT = T - T0 (\circC)'); grid on;
end
sgtitle('24位标定补偿后 零偏温度模型拟合 (b = c0 + c1·dT + c2·dT^2 + c3·dT^3)');

%% ---------------- 输出 6 轴系数列表 ----------------
coefTable = array2table([coef(:,[4 3 2 1]), rmsRes], ...
    'VariableNames', {'c0','c1','c2','c3','rms_residual'}, ...
    'RowNames', {'ax','ay','az','gx','gy','gz'});
fprintf('\n6 轴系数列表 [c0 c1 c2 c3] (dT = T - T0):\n');
disp(coefTable);

Tout = table(axNames', coef(:,4), coef(:,3), coef(:,2), coef(:,1), rmsRes, ...
    'VariableNames', {'axis','c0','c1','c2','c3','rms_residual'});
writetable(Tout, outCsv);
fprintf('系数已保存: %s\n', outCsv);

Tref = T0;
save(outMat, 'coef', 'Tref', 'axNames', 'decim', 'ba', 'Ca', 'gb');
fprintf('系数已保存: %s\n', outMat);

%% ---------------- 导出全量补偿后数据（供 Allan 对比工具）----------------
% 列: sample,time_s,dt_s,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,gx_deg_h,gy_deg_h,gz_deg_h
fprintf('导出补偿后全量数据: %s\n', outImuC);
tic;
E = table(M(:,1), M(:,6), M(:,7), ...
          Ytc(:,1), Ytc(:,2), Ytc(:,3), temp, ...
          Ytc(:,4), Ytc(:,5), Ytc(:,6), ...
    'VariableNames', {'sample','time_s','dt_s', ...
                      'ax_m_s2','ay_m_s2','az_m_s2','temp_deg_c', ...
                      'gx_deg_h','gy_deg_h','gz_deg_h'});
writetable(E, outImuC);
fprintf('导出完成: %d 行, 耗时 %.1f s\n', height(E), toc);
