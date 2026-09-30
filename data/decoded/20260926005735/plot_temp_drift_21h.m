%% plot_temp_drift_21h.m
% ========================================================================
% 21 小时静态 IMU 数据：陀螺 / 加计 与 温度 的时序对照（yyaxis 双轴）
%
%   图1: 左轴 gx/gy/gz [deg/s]，右轴温度 [degC]
%   图2: 左轴 ax/ay/az [mg]  ，右轴温度 [degC]
%
%   - IMU 输出做 60 s 滑动平均平滑
%   - IEEE 期刊风格出图（Times New Roman / 四边全框 / 刻度朝内 / 无网格）
%   - 图尺寸 widthIn=6.0, heightIn=4.2, fontSize=18, 600 dpi（指定值）
%
% 期望文件: imu.csv（与本脚本同目录，21 h 静态采集）
% 输出:
%   figs/fig_21h_gyro_temp.png
%   figs/fig_21h_accel_temp.png
%
% ========================================================================

clc;
clear;
close all;

scriptDir = fileparts(mfilename('fullpath'));

%% ============================== CONFIG ===============================

imuFile   = fullfile(scriptDir, 'imu.csv');
figDir    = fullfile(scriptDir, 'figs');
smoothSec = 60;                 % 平滑窗口 [s]
gLocal    = 9.7932;             % m/s^2 -> mg 换算

% 图尺寸（指定值）
widthIn  = 6.0;
heightIn = 4.2;
fontSize = 18;
dpi      = 600;

% IEEE 配色：左轴 3 通道 + 右轴温度
ieeeC = [[0, 114, 178]/255;    % gx / ax    蓝
         [230, 159, 0]/255;    % gy / ay    橙
         [0, 158, 115]/255;    % gz / az    绿
         [213, 94, 0]/255];   % 温度        红

lineW = 1.2;

if ~isfile(imuFile)
    error('imu.csv not found: %s', imuFile);
end

%% ============================== READ =================================

fprintf('Reading %s ...\n', imuFile);
tic
% T = readtable(imuFile, 'SelectedVariableNames', ...
%     {'time_s','dt_s', ...
%      'gx_deg_h','gy_deg_h','gz_deg_h', ...
%      'ax_m_s2','ay_m_s2','az_m_s2', ...
%      'temp_deg_c'});
T = readtable(imuFile);
fprintf('  %d samples, span %.2f h (%.0f s), read time %.1f s\n', ...
    height(T), (T.time_s(end)-T.time_s(1))/3600, ...
    T.time_s(end)-T.time_s(1), toc);

%% ========================= 60 s SMOOTHING ============================

fs   = 1/mean(T.dt_s);
nWin = max(3, round(smoothSec*fs));

fprintf('Sampling rate ~ %.2f Hz -> smoothing window = %d samples (%.0f s)\n', ...
    fs, nWin, smoothSec);

t_h  = T.time_s/3600;                          % 横轴：小时

% 60 s 平滑后再减去各自全程静态均值（消常值零偏，突出温漂细节）
zm = @(x) x - mean(x);

gx_s = zm(movmean(T.gx_deg_h/3600,       nWin));   % deg/h -> deg/s
gy_s = zm(movmean(T.gy_deg_h/3600,       nWin));
gz_s = zm(movmean(T.gz_deg_h/3600,       nWin));

ax_s = zm(movmean(T.ax_m_s2/gLocal*1000, nWin));   % m/s^2 -> mg
ay_s = zm(movmean(T.ay_m_s2/gLocal*1000, nWin));
az_s = zm(movmean(T.az_m_s2/gLocal*1000, nWin));

tp_s = movmean(T.temp_deg_c,             nWin);    % 温度不减均值

%% ==================== FIG 1: GYRO / TEMPERATURE ======================

fig1 = figure('Name','21h static IMU: gyro vs temperature');

hold on;
h1 = plot(t_h, gx_s, 'Color',ieeeC(1,:), 'LineWidth',lineW);
h2 = plot(t_h, gy_s, 'Color',ieeeC(2,:), 'LineWidth',lineW);
h3 = plot(t_h, gz_s, 'Color',ieeeC(3,:), 'LineWidth',lineW);

yyaxis right
h4 = plot(t_h, tp_s, 'Color',ieeeC(4,:), 'LineWidth',lineW);
ylabel('Temperature (degC)');
xlim([0 24])
yyaxis left
ylabel('Gyro (deg/s)');
xlabel('Time (h)');
legend([h1 h2 h3 h4], {'G_x','G_y','G_z','Temperature'}, 'Location','southeast');

save_fig(fig1, figDir, 'fig_21h_gyro_temp', heightIn, widthIn, fontSize, dpi);

%% ==================== FIG 2: ACCELEROMETER / TEMPERATURE =============

fig2 = figure('Name','21h static IMU: accelerometer vs temperature');

figure(fig2);
hold on;
h1 = plot(t_h, ax_s, 'Color',ieeeC(1,:), 'LineWidth',lineW);
h2 = plot(t_h, ay_s, 'Color',ieeeC(2,:), 'LineWidth',lineW);
h3 = plot(t_h, az_s, 'Color',ieeeC(3,:), 'LineWidth',lineW);

yyaxis right
h4 = plot(t_h, tp_s, 'Color',ieeeC(4,:), 'LineWidth',lineW);
ylabel('Temperature (degC)');
xlim([0 24])
yyaxis left

ylabel('Acce (mg)');
xlabel('Time (h)');
legend([h1 h2 h3 h4], {'A_x','A_y','A_z','Temperature'}, 'Location','southeast');

save_fig(fig2, figDir, 'fig_21h_accel_temp', heightIn, widthIn, fontSize, dpi);

fprintf('\nFinished. Figures saved in: %s\n', figDir);

%% =====================================================================
%%                             LOCAL FUNCTIONS
%% =====================================================================

function fp = save_fig(fh, figDir, baseName, heightIn, widthIn, fontSize, dpi)
% 按 IEEE 期刊规范美化并保存 figure（PNG）。
    apply_ieee_style(fh,figDir, widthIn, heightIn, fontSize);

    fp = fullfile(figDir, [baseName '.png']);
    try
        exportgraphics(fh, fp, 'Resolution', dpi);
    catch
        print(fh, fp, '-dpng', sprintf('-r%d', dpi));
    end
    fprintf('Saved: %s\n', fp);
end


function apply_ieee_style(fh,figDir, widthIn, heightIn, fontSize)
% IEEE 期刊风格统一设置。
% 注意：axes 背景用 'none'（透明）而不是 'w'——否则在 yyaxis 双轴图中，
% 上层的右轴白底会遮住左轴曲线。
    if ~exist(figDir,'dir'), mkdir(figDir); end %#ok<NASGU>

    set(fh, 'Color','w', ...
        'Units','inches', ...
        'Position',[1 1 widthIn*1.4 heightIn*1.4], ...   % 屏幕显示放大
        'PaperUnits','inches', ...
        'PaperPosition',[0 0 widthIn heightIn], ...
        'PaperSize',[widthIn heightIn]);

    ax = findall(fh,'Type','axes');
    if ~isempty(ax)
        set(ax, ...
            'FontName','Times New Roman', ...
            'FontSize',fontSize, ...
            'Box','on', ...
            'TickDir','in', ...
            'TickLength',[0.014 0.014], ...
            'LineWidth',0.75, ...
            'XMinorTick','on', ...
            'YMinorTick','on', ...
            'XGrid','off', ...
            'YGrid','off', ...
            'Color','none');
    end
    ax.YAxis(1).Color = [0 0 0];
    ax.YAxis(2).Color = [0 0 0];
    lg = findall(fh,'Type','legend');
    if ~isempty(lg)
        set(lg, ...
            'Box','off', ...
            'FontName','Times New Roman', ...
            'FontSize',max(fontSize-2,6));
    end
end
