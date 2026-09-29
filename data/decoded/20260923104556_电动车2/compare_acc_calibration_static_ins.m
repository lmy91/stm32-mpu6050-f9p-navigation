%% compare_acc_calibration_static_ins.m
% ========================================================================
% 纯 INS 静态传播三路对比：原始输出 / 原始+温补 / 原始+温补+24位标定
%
% Case 1: raw output
%         不做任何补偿。
%
% Case 2: raw + temperature compensation
%         drift(T) = sum(c_m*dT^m, m=1..各轴阶数)，不补 0 阶 c0
%         加计: a_tc = a_raw - driftAcc(T)
%         陀螺: g_tc = g_raw - driftGyr(T)
%
% Case 3: raw + temp comp + 24-position acc calibration
%         在 Case 2 的基础上再套用标定结果:
%         dv_cal = Ca * (dv_tc - ba * dt)
%
% 处理链与 tools/calib24_static_numbered_tempcomp.m 完全一致：
%   原始输出 -> 温度补偿 -> 24 位加计标定
%
% 本脚本刻意保持与当前松组合实验一致的部分：
%   - 相同的 GNSS 筛选
%   - 相同的静止区间: GNSS 有效序号 20:200
%   - 相同的导航起始 AVP 定义:
%       * 固定姿态 [-1.3230, -4.8818, 0] deg
%       * 最近的 GNSS 有效速度/位置
%   - 相同的 RFU 映射: [R,F,U] = [-sensor-Y, sensor-X, sensor-Z]
%   - 陀螺零偏处理（三个 case 统一 = 减去静止段均值），温补在前：
%       Case 1      : 未温补的陀螺 -> 减静止段均值
%       Case 2 / 3  : 温补后的陀螺 -> 减静止段均值
%     即"温度补偿"永远发生在"减静止段均值"之前，因此三个 case 的
%     静态均值本身取值不同（温补后的均值会明显更小）。
%
% IMPORTANT:
%   初始化后没有 KF / GNSS 更新 / ZUPT / 反馈，只调用 insupdate()。
%
% 期望文件:
%   gnss.csv
%   imu.csv
%   data/calib24/temp_coeffs_raw.mat                 (温补系数矩阵)
%   data/calib24/calib24_result_tempcomp_azgxgy.mat  (24 位标定结果)
%
% 输出:
%   static_ins_acc_cal_compare.mat
%   static_ins_acc_cal_summary.csv
%
% ========================================================================

clc;
clear;
close all;

glvs;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(fileparts(scriptDir)));

%% ============================== CONFIG ===============================

cfg.gnssFile = fullfile(scriptDir, 'gnss.csv');
cfg.imuFile  = fullfile(scriptDir, 'imu.csv');

% ---- 温补 / 标定产物（与 tools/calib24 流水线一致）----
cfg.calib24Dir = fullfile(repoRoot, 'data', 'calib24');

% 温补系数矩阵（fit_temp_bias_raw.m 产物），6x6 [c5..c0]，行序 [ax ay az gx gy gz]
cfg.tempCoeffFile = fullfile(cfg.calib24Dir, 'temp_coeffs_raw.mat');

% 24 位标定结果（calib24_static_numbered_tempcomp.m 产物，内部已含温补系数）
cfg.calibFile = fullfile(cfg.calib24Dir, 'calib24_result_tempcomp_azgxgy.mat');

% Keep the SAME indices as the current LC experiment.
cfg.n_start_gnss        = 200;
cfg.n_static_begin_gnss = 20;
cfg.n_static_end_gnss   = 200;

% Same PSINS IMU batching as current experiment.
cfg.nn = 2;
cfg.ts = 0.01;

% Fixed attitude from the user's controlled A/B experiment.
% PSINS order here follows the current script: [pitch; roll; yaw].
cfg.att0_deg = [-1.3230; -4.8818; 0.0];

% false = exactly follow the LC experiment's GNSS-derived vel0.
% true  = use [0;0;0] m/s, which is physically cleaner for a static test.
cfg.forceZeroInitialVelocity = false;

% 图片输出目录（IEEE 规范图导出到此，默认脚本同级的 figs/）
cfg.figDir = fullfile(scriptDir, 'figs');

%% ============================== READ ================================

fprintf('\n============================================================\n');
fprintf(' Pure static INS: raw / +temp comp / +temp comp + calib\n');
fprintf('============================================================\n');

gnss0 = readtable(cfg.gnssFile);
imu0  = readtable(cfg.imuFile);

gnss_valid = build_gnss_valid(gnss0);
imu_valid_raw = build_imu_valid(imu0);

if cfg.n_start_gnss > size(gnss_valid,1)
    error('n_start_gnss exceeds valid GNSS count.');
end

if cfg.n_static_end_gnss > size(gnss_valid,1)
    error('Static GNSS index exceeds valid GNSS count.');
end

%% ======================= LOAD TEMP COEFFICIENTS =====================

if ~isfile(cfg.tempCoeffFile)
    error(['Temp coefficient file not found:\n%s\n' ...
           'Run tools/fit_temp_bias_raw.m first.'], cfg.tempCoeffFile);
end

TC = load(cfg.tempCoeffFile);

if ~isfield(TC,'coef') || size(TC.coef,1) ~= 6
    error('temp_coeffs_raw.mat 必须包含 6 行的 coef 矩阵.');
end

TCs = struct();
TCs.TCpoly = [TC.coef(:,1:end-1), zeros(6,1)];   % [c5..c1 0]，常数项显式给 0
if isfield(TC,'ord')
    TCs.ord = TC.ord(:)';
else
    TCs.ord = TC.axisOrder(:)';
end
TCs.tcActive = TCs.ord > 0;
TCs.Tref     = TC.Tref;
if isfield(TC,'Tmin'), TCs.Tmin = TC.Tmin; else, TCs.Tmin = NaN; end
if isfield(TC,'Tmax'), TCs.Tmax = TC.Tmax; else, TCs.Tmax = NaN; end

if any(imu_valid_raw(:,9) < TCs.Tmin-1e-9 | imu_valid_raw(:,9) > TCs.Tmax+1e-9)
    error('IMU temperature exceeds fitted range [%.4f, %.4f] degC.', ...
        TCs.Tmin, TCs.Tmax);
end

fprintf('\nTemperature compensation coefficients:\n');
fprintf('  file : %s\n', cfg.tempCoeffFile);
fprintf('  Tref = %.4f degC, fit range [%.4f, %.4f] degC\n', ...
    TCs.Tref, TCs.Tmin, TCs.Tmax);
fprintf('  axis order  [ax ay az gx gy gz]: %d %d %d %d %d %d\n', TCs.ord);
fprintf('  active axes [ax ay az gx gy gz]: %d %d %d %d %d %d\n', +TCs.tcActive);
fprintf('  TCpoly [c5..c1 0]:\n');
disp(TCs.TCpoly);

%% ========================= LOAD CALIBRATION ==========================

if ~isfile(cfg.calibFile)
    error(['Calibration result not found:\n%s\n' ...
           'Run tools/calib24_static_numbered_tempcomp.m first.'], cfg.calibFile);
end

calib = load(cfg.calibFile);

if ~isfield(calib,'result') || ...
   ~isfield(calib.result,'ba') || ...
   ~isfield(calib.result,'Ca')
    error('Calibration MAT must contain result.ba and result.Ca.');
end

ba_cal = calib.result.ba(:);
Ca_cal = calib.result.Ca;

% ---- 自检：标定内置温补系数 / Tref 是否与本次温补一致 ----
if isfield(calib.result,'tempCoeff')
    dmax = max(abs(calib.result.tempCoeff(:) - TCs.TCpoly(:)));
    if dmax > 1e-9
        error(['标定结果内嵌的温补系数与本次温补所用系数不一致 ' ...
               '(max|diff| = %.3g)。'], dmax);
    end
else
    error('标定结果缺少 result.tempCoeff，无法验证参数绑定。');
end
if isfield(calib.result,'Tref') && abs(calib.result.Tref - TCs.Tref) > 1e-6
    error('标定结果 Tref = %.6f 与温补 Tref = %.6f 不一致。', ...
        calib.result.Tref, TCs.Tref);
end
if ~isfield(calib.result,'tcActive') || ...
        ~isequal(logical(calib.result.tcActive(:)'), logical(TCs.tcActive))
    error('标定结果的有效温补轴与温补系数不一致。');
end

fprintf('\nCalibration parameters:\n');
fprintf('ba [m/s^2] =\n');
disp(ba_cal);
fprintf('Ca =\n');
disp(Ca_cal);

%% ======================= COMMON INITIAL AVP ==========================

% Reproduce the normal navigation start time from the LC experiment.
t_start_nav = gnss_valid(cfg.n_start_gnss,2);

% In the LC code, imu1 is cut with imu_time >= t_start, so reproduce
% the actual first IMU timestamp used there.
idx_imu_nav0 = find(imu_valid_raw(:,2) >= t_start_nav, 1, 'first');

if isempty(idx_imu_nav0)
    error('No IMU sample found at/after normal navigation start.');
end

t0_nav = imu_valid_raw(idx_imu_nav0,2);

[~,idx0] = min(abs(gnss_valid(:,2)-t0_nav));

pos0 = [ ...
    gnss_valid(idx0,3)*pi/180;
    gnss_valid(idx0,4)*pi/180;
    gnss_valid(idx0,5)];

vel0_gnss = [ ...
    gnss_valid(idx0,7);
    gnss_valid(idx0,6);
    -gnss_valid(idx0,8)];

if cfg.forceZeroInitialVelocity
    vel0 = [0;0;0];
else
    vel0 = vel0_gnss;
end

att0 = cfg.att0_deg*pi/180;

avp0 = [att0; vel0; pos0];

fprintf('\nCommon initial AVP used by ALL cases:\n');
fprintf('t0(nav experiment) = %.6f s\n', t0_nav);
fprintf('att0 [pitch roll yaw] = %.6f  %.6f  %.6f deg\n', ...
    cfg.att0_deg(1), cfg.att0_deg(2), cfg.att0_deg(3));
fprintf('vel0 [E N U] = %.6f  %.6f  %.6f m/s\n', ...
    vel0(1),vel0(2),vel0(3));
fprintf('GNSS vel at t0 [E N U] = %.6f  %.6f  %.6f m/s\n', ...
    vel0_gnss(1),vel0_gnss(2),vel0_gnss(3));
fprintf('pos0 [lat lon h] = %.9f  %.9f  %.3f\n', ...
    pos0(1)*180/pi,pos0(2)*180/pi,pos0(3));

if ~cfg.forceZeroInitialVelocity && norm(vel0) > 0.05
    fprintf(['WARNING: common initial GNSS speed is %.4f m/s although the replayed\n' ...
             '         IMU segment is static. Compare Delta-v=v-v0 first.\n'], ...
             norm(vel0));
end

%% ========================= STATIC INTERVAL ===========================

t_static_start = gnss_valid(cfg.n_static_begin_gnss,2);
t_static_end   = gnss_valid(cfg.n_static_end_gnss,2);

fprintf('\nStatic replay interval:\n');
fprintf('start = %.6f s\n',t_static_start);
fprintf('end   = %.6f s\n',t_static_end);
fprintf('span  = %.3f s\n',t_static_end-t_static_start);

%% =========================== RUN CASE 1 ==============================

fprintf('\n============================================================\n');
fprintf('CASE 1: raw output (no compensation)\n');
fprintf('============================================================\n');

R1 = run_static_ins_case( ...
    imu_valid_raw, ...
    t_static_start,t_static_end, ...
    avp0, ...
    false,false, TCs,ba_cal,Ca_cal,cfg, ...
    "Case 1: raw");

%% =========================== RUN CASE 2 ==============================

fprintf('\n============================================================\n');
fprintf('CASE 2: raw + temperature compensation\n');
fprintf('============================================================\n');

R2 = run_static_ins_case( ...
    imu_valid_raw, ...
    t_static_start,t_static_end, ...
    avp0, ...
    true,false, TCs,ba_cal,Ca_cal,cfg, ...
    "Case 2: raw + temp comp");

%% =========================== RUN CASE 3 ==============================

fprintf('\n============================================================\n');
fprintf('CASE 3: raw + temperature compensation + 24-pos acc calibration\n');
fprintf('============================================================\n');

R3 = run_static_ins_case( ...
    imu_valid_raw, ...
    t_static_start,t_static_end, ...
    avp0, ...
    true,true, TCs,ba_cal,Ca_cal,cfg, ...
    "Case 3: raw + temp comp + calib");

%% ============================= COMPARE ===============================

% Common grid: use Case 3 time and interpolate the other two if needed.
t = R3.t;

R1_vel = interp1(R1.t,R1.vn,t,'linear','extrap');
R1_att = interp1(R1.t,R1.att,t,'linear','extrap');
R1_pos = interp1(R1.t,R1.pos,t,'linear','extrap');

R2_vel = interp1(R2.t,R2.vn,t,'linear','extrap');
R2_att = interp1(R2.t,R2.att,t,'linear','extrap');
R2_pos = interp1(R2.t,R2.pos,t,'linear','extrap');

R3_vel = R3.vn;
R3_att = R3.att;
R3_pos = R3.pos;

% Direct differences relative to raw (Case 1)
velDiff_21 = R2_vel - R1_vel;
velDiff_31 = R3_vel - R1_vel;

attDiff_21 = R2_att - R1_att;
attDiff_21 = atan2(sin(attDiff_21),cos(attDiff_21));
attDiffDeg_21 = attDiff_21*180/pi;

attDiff_31 = R3_att - R1_att;
attDiff_31 = atan2(sin(attDiff_31),cos(attDiff_31));
attDiffDeg_31 = attDiff_31*180/pi;

posDiffENU_21 = zeros(numel(t),3);
posDiffENU_31 = zeros(numel(t),3);

for i = 1:numel(t)
    posDiffENU_21(i,:) = llh_difference_enu(R2_pos(i,:)',R1_pos(i,:)')';
    posDiffENU_31(i,:) = llh_difference_enu(R3_pos(i,:)',R1_pos(i,:)')';
end

%% ========================= COMMAND SUMMARY ===========================

fprintf('\n============================================================\n');
fprintf('              PURE INS STATIC COMPARISON\n');
fprintf('============================================================\n');

fprintf('\nGyro static mean removed [R F U, deg/h]:\n');
fprintf('Case 1 raw                 : %.6f  %.6f  %.6f   (no temp comp)\n', ...
    R1.gyro_static_deg_h);
fprintf('Case 2 raw + temp comp     : %.6f  %.6f  %.6f   (temp comp first)\n', ...
    R2.gyro_static_deg_h);
fprintf('Case 3 raw + TC + calib    : %.6f  %.6f  %.6f   (temp comp first)\n', ...
    R3.gyro_static_deg_h);

fprintf('\nStatic specific-force magnitude:\n');
fprintf('Case 1 raw                 : %.8f m/s^2\n',norm(R1.acc_mean));
fprintf('Case 2 raw + temp comp     : %.8f m/s^2\n',norm(R2.acc_mean));
fprintf('Case 3 raw + TC + calib    : %.8f m/s^2\n',norm(R3.acc_mean));

fprintf('\nVelocity change relative to COMMON initial v0\n');
fprintf('                     E          N          U        3D\n');
fprintf('Case 1 raw       %9.4f  %9.4f  %9.4f  %9.4f\n', ...
    R1.final_dv, norm(R1.final_dv));
fprintf('Case 2 +TC       %9.4f  %9.4f  %9.4f  %9.4f\n', ...
    R2.final_dv, norm(R2.final_dv));
fprintf('Case 3 +TC+cal   %9.4f  %9.4f  %9.4f  %9.4f\n', ...
    R3.final_dv, norm(R3.final_dv));
fprintf('RMS 1 raw        %9.4f  %9.4f  %9.4f  %9.4f\n', ...
    R1.dv_rms_axis, R1.dv_rms_3d);
fprintf('RMS 2 +TC        %9.4f  %9.4f  %9.4f  %9.4f\n', ...
    R2.dv_rms_axis, R2.dv_rms_3d);
fprintf('RMS 3 +TC+cal    %9.4f  %9.4f  %9.4f  %9.4f\n', ...
    R3.dv_rms_axis, R3.dv_rms_3d);

fprintf('\nPosition displacement from COMMON initial position\n');
fprintf('                     E          N          U        3D\n');
fprintf('Case 1 raw       %9.3f  %9.3f  %9.3f  %9.3f\n', ...
    R1.final_dp_enu, norm(R1.final_dp_enu));
fprintf('Case 2 +TC       %9.3f  %9.3f  %9.3f  %9.3f\n', ...
    R2.final_dp_enu, norm(R2.final_dp_enu));
fprintf('Case 3 +TC+cal   %9.3f  %9.3f  %9.3f  %9.3f\n', ...
    R3.final_dp_enu, norm(R3.final_dp_enu));
fprintf('RMS 1 raw        %9.3f  %9.3f  %9.3f  %9.3f\n', ...
    R1.dp_rms_axis, R1.dp_rms_3d);
fprintf('RMS 2 +TC        %9.3f  %9.3f  %9.3f  %9.3f\n', ...
    R2.dp_rms_axis, R2.dp_rms_3d);
fprintf('RMS 3 +TC+cal    %9.3f  %9.3f  %9.3f  %9.3f\n', ...
    R3.dp_rms_axis, R3.dp_rms_3d);

fprintf('\nDirect difference at final epoch (v2 relative to raw):\n');
fprintf('Delta-v_(+TC  ) [E N U] = %.6f  %.6f  %.6f m/s\n', ...
    velDiff_21(end,1),velDiff_21(end,2),velDiff_21(end,3));
fprintf('Delta-v_(+TCcal) [E N U] = %.6f  %.6f  %.6f m/s\n', ...
    velDiff_31(end,1),velDiff_31(end,2),velDiff_31(end,3));
fprintf('Delta-p_(+TC  ) [E N U] = %.4f  %.4f  %.4f m\n', ...
    posDiffENU_21(end,1),posDiffENU_21(end,2),posDiffENU_21(end,3));
fprintf('Delta-p_(+TCcal) [E N U] = %.4f  %.4f  %.4f m\n', ...
    posDiffENU_31(end,1),posDiffENU_31(end,2),posDiffENU_31(end,3));
fprintf('Delta-att_(+TC  ) [pitch roll yaw] = %.6f  %.6f  %.6f deg\n', ...
    attDiffDeg_21(end,1),attDiffDeg_21(end,2),attDiffDeg_21(end,3));
fprintf('Delta-att_(+TCcal) [pitch roll yaw] = %.6f  %.6f  %.6f deg\n', ...
    attDiffDeg_31(end,1),attDiffDeg_31(end,2),attDiffDeg_31(end,3));

%% =========================== SUMMARY TABLE ===========================

Metric = [
    "Final velocity-change norm"
    "Velocity-change RMS 3D"
    "Final position-displacement norm"
    "Position-displacement RMS 3D"
    ];

Unit = [
    "m/s"
    "m/s"
    "m"
    "m"
    ];

Raw_v = [
    norm(R1.final_dv)
    R1.dv_rms_3d
    norm(R1.final_dp_enu)
    R1.dp_rms_3d
    ];

TC_v = [
    norm(R2.final_dv)
    R2.dv_rms_3d
    norm(R2.final_dp_enu)
    R2.dp_rms_3d
    ];

TCC_v = [
    norm(R3.final_dv)
    R3.dv_rms_3d
    norm(R3.final_dp_enu)
    R3.dp_rms_3d
    ];

Impr_TC_pct     = 100*(Raw_v-TC_v)./Raw_v;
Impr_TC_Cal_pct = 100*(Raw_v-TCC_v)./Raw_v;

summaryTable = table( ...
    Metric,Unit,Raw_v,TC_v,TCC_v,Impr_TC_pct,Impr_TC_Cal_pct, ...
    'VariableNames', ...
    {'Metric','Unit','Raw','Raw_TC','Raw_TC_Cal', ...
     'Impr_TC_pct','Impr_TC_Cal_pct'});

disp(summaryTable);

%% ============================== PLOTS ================================
% IEEE 期刊作图规范：
%   - 白底、无网格、四边全框、刻度朝内、Times New Roman
%   - 单栏宽 3.5 in、字号 8 pt、PNG 600 dpi
%   - 三条曲线用色盲友好颜色 + 不同线型双重区分，黑白打印仍可辨识
%   - 静止真值为共同初始状态：速度变化、位置变化和姿态变化均作为误差
%   - 每个量的三轴误差与三维模值分开成图

if ~isempty(cfg.figDir) && ~exist(cfg.figDir,'dir')
    mkdir(cfg.figDir);
end

figFiles = {};   % 记录已保存的图片，最后统一打印

legendLabels = {'Raw','TC','TC + calib.'};

% IEEE 曲线样式：颜色 + 线型双重区分
ieeeLW = 1.15;
ieeeC  = [0   0   0;          % 1) raw                        黑   实线
          0 114 178;          % 2) + temp comp                蓝   虚线
          213 94 0] / 255;    % 3) + temp comp + 24-pos calib 朱红 点划线
ieeeLS = {'-','--','-.'};

tPlot = R3.t-R3.t(1);
seriesVelocity = {R1.dv, R2.dv, R3.dv};
seriesPosition = {R1.dpENU, R2.dpENU, R3.dpENU};
seriesAttitude = {R1.datt_deg, R2.datt_deg, R3.datt_deg};

% 1-2. Velocity error: three axes + separate 3-D norm
fig1 = plot_three_axis_error(tPlot, seriesVelocity, ...
    {'East','North','Up'}, ...
    {'e_{v,E} / (m s^{-1})','e_{v,N} / (m s^{-1})','e_{v,U} / (m s^{-1})'}, ...
    legendLabels, ieeeC, ieeeLS, ieeeLW, 'Static velocity error');
figFiles{end+1} = save_fig(fig1, cfg.figDir, ...
    'fig_static_01_velocity_error_axes', 5.2, 3.5, 8, 600);

fig2 = plot_error_norm(tPlot, seriesVelocity, ...
    '|e_v|_2 / (m s^{-1})', 'Static velocity-error norm', ...
    legendLabels, ieeeC, ieeeLS, ieeeLW);
figFiles{end+1} = save_fig(fig2, cfg.figDir, ...
    'fig_static_02_velocity_error_norm', 2.65, 3.5, 8, 600);

% 3-4. Position error: three axes + separate 3-D norm
fig3 = plot_three_axis_error(tPlot, seriesPosition, ...
    {'East','North','Up'}, ...
    {'e_{p,E} / m','e_{p,N} / m','e_{p,U} / m'}, ...
    legendLabels, ieeeC, ieeeLS, ieeeLW, 'Static position error');
figFiles{end+1} = save_fig(fig3, cfg.figDir, ...
    'fig_static_03_position_error_axes', 5.2, 3.5, 8, 600);

fig4 = plot_error_norm(tPlot, seriesPosition, ...
    '|e_p|_2 / m', 'Static position-error norm', ...
    legendLabels, ieeeC, ieeeLS, ieeeLW);
figFiles{end+1} = save_fig(fig4, cfg.figDir, ...
    'fig_static_04_position_error_norm', 2.65, 3.5, 8, 600);

% 5-6. Attitude error: three axes + separate 3-D norm
fig5 = plot_three_axis_error(tPlot, seriesAttitude, ...
    {'Pitch','Roll','Yaw'}, ...
    {'e_{pitch} / deg','e_{roll} / deg','e_{yaw} / deg'}, ...
    legendLabels, ieeeC, ieeeLS, ieeeLW, 'Static attitude error');
figFiles{end+1} = save_fig(fig5, cfg.figDir, ...
    'fig_static_05_attitude_error_axes', 5.2, 3.5, 8, 600);

fig6 = plot_error_norm(tPlot, seriesAttitude, ...
    '|e_{att}|_2 / deg', 'Static attitude-error norm', ...
    legendLabels, ieeeC, ieeeLS, ieeeLW);
figFiles{end+1} = save_fig(fig6, cfg.figDir, ...
    'fig_static_06_attitude_error_norm', 2.65, 3.5, 8, 600);

% 只保留本次定义的六张静态图；动态图不受影响。
expectedStatic = string(figFiles(:));
oldStatic = dir(fullfile(cfg.figDir, 'fig_static_*.png'));
for i = 1:numel(oldStatic)
    oldPath = string(fullfile(oldStatic(i).folder, oldStatic(i).name));
    if ~any(oldPath == expectedStatic)
        delete(oldPath);
    end
end

%% =============================== SAVE ================================

cmp.t = t;
cmp.velDiff_21 = velDiff_21;
cmp.velDiff_31 = velDiff_31;
cmp.posDiffENU_21 = posDiffENU_21;
cmp.posDiffENU_31 = posDiffENU_31;
cmp.attDiffDeg_21 = attDiffDeg_21;
cmp.attDiffDeg_31 = attDiffDeg_31;

save(fullfile(scriptDir, 'static_ins_acc_cal_compare.mat'), ...
    'R1','R2','R3','cmp','summaryTable','cfg','ba_cal','Ca_cal','TCs','avp0');

writetable(summaryTable, fullfile(scriptDir, 'static_ins_acc_cal_summary.csv'));

fprintf('\nSaved:\n');
fprintf('  static_ins_acc_cal_compare.mat\n');
fprintf('  static_ins_acc_cal_summary.csv\n');
if ~isempty(figFiles)
    fprintf('  figures (%d):\n', numel(figFiles));
    for i = 1:numel(figFiles)
        fprintf('    %s\n', figFiles{i});
    end
end
fprintf('\nFinished.\n');

%% =====================================================================
%%                           LOCAL FUNCTIONS
%% =====================================================================

function fh = plot_three_axis_error(tSec, series, axisNames, yLabels, ...
    legendLabels, colors, lineStyles, lineWidth, figName)
% 三轴误差纵向排列；图例占独立布局行，不遮挡曲线。
    fh = figure('Name', figName);
    tl = tiledlayout(fh, 3, 1, 'TileSpacing','compact', 'Padding','compact');
    ax = gobjects(3,1);
    lineHandles = gobjects(3,1);

    for i = 1:3
        ax(i) = nexttile(tl);
        hold(ax(i),'on');
        yline(ax(i), 0, ':', 'Color',[0.65 0.65 0.65], ...
            'LineWidth',0.55, 'HandleVisibility','off');
        for k = 1:3
            h = plot(ax(i), tSec, series{k}(:,i), ...
                'Color',colors(k,:), 'LineStyle',lineStyles{k}, ...
                'LineWidth',lineWidth);
            if i == 1
                lineHandles(k) = h;
            end
        end
        title(ax(i), sprintf('(%c) %s-axis error', char('a'+i-1), axisNames{i}), ...
            'FontWeight','normal');
        ylabel(ax(i), yLabels{i}, 'Interpreter','tex');
        xlim(ax(i), [tSec(1) tSec(end)]);
        if i < 3
            ax(i).XTickLabel = [];
        else
            xlabel(ax(i), 'Elapsed time / s');
        end
    end
    linkaxes(ax,'x');
    lg = legend(ax(1), lineHandles, legendLabels, ...
        'Orientation','horizontal', 'NumColumns',3);
    lg.Layout.Tile = 'north';
end


function fh = plot_error_norm(tSec, series, yLabelText, titleText, ...
    legendLabels, colors, lineStyles, lineWidth)
% 三维欧氏模值单独成图。
    fh = figure('Name', titleText);
    ax = axes(fh);
    hold(ax,'on');
    yline(ax, 0, ':', 'Color',[0.65 0.65 0.65], ...
        'LineWidth',0.55, 'HandleVisibility','off');
    lineHandles = gobjects(3,1);
    for k = 1:3
        lineHandles(k) = plot(ax, tSec, vecnorm(series{k},2,2), ...
            'Color',colors(k,:), 'LineStyle',lineStyles{k}, ...
            'LineWidth',lineWidth);
    end
    xlim(ax, [tSec(1) tSec(end)]);
    xlabel(ax, 'Elapsed time / s');
    ylabel(ax, yLabelText, 'Interpreter','tex');
    title(ax, titleText, 'FontWeight','normal');
    legend(ax, lineHandles, legendLabels, 'Location','best');
end


function fp = save_fig(fh, figDir, baseName, heightIn, widthIn, fontSize, dpi)
% 按 IEEE 期刊规范美化并保存 figure（PNG）。
%   heightIn : 图高 [inch]；省略时按子图数量自动估计
%   widthIn  : 图宽 [inch]，默认 IEEE 单栏 3.5（双栏用 7.16）
%   fontSize : 轴/图例字号 [pt]，默认 8
%   dpi      : 导出分辨率，默认 600
    if nargin < 7 || isempty(dpi),      dpi      = 600; end
    if nargin < 6 || isempty(fontSize), fontSize = 8;   end
    if nargin < 5 || isempty(widthIn),  widthIn  = 3.5; end
    if nargin < 4 || isempty(heightIn)
        nAx = numel(findall(fh,'Type','axes'));
        heightIn = 1.6 + 1.3*max(nAx,1);   % 每个子图约 1.3 in
    end

    apply_ieee_style(fh, widthIn, heightIn, fontSize);

    fp = fullfile(figDir, [baseName '.png']);
    % print 严格遵循 PaperSize/PaperPosition：3.5 in × 600 dpi = 2100 px，
    % 避免 exportgraphics 自动裁边后破坏 IEEE 单栏物理尺寸。
    print(fh, fp, '-dpng', sprintf('-r%d', dpi), '-painters');
end


function apply_ieee_style(fh, widthIn, heightIn, fontSize)
% IEEE 期刊风格统一设置：
%   白底 / Times New Roman / 刻度朝内 / 四边全框 / 无网格 / 细轴线 / 图例无边框
    set(fh, 'Color','w', ...
        'Units','inches', ...
        'Position',[1 1 widthIn heightIn], ...
        'PaperUnits','inches', ...
        'PaperPosition',[0 0 widthIn heightIn], ...
        'PaperSize',[widthIn heightIn], ...
        'PaperPositionMode','manual', ...
        'InvertHardcopy','off');

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
            'Color','w');
    end

    lg = findall(fh,'Type','legend');
    if ~isempty(lg)
        set(lg, ...
            'Box','off', ...
            'FontName','Times New Roman', ...
            'FontSize',max(fontSize-1,6));
    end
end


function gnss_valid = build_gnss_valid(gnss0)

    len = size(gnss0,1);
    gnss_valid = zeros(len,13);
    n = 1;

    for i = 1:len

        if gnss0.fix(i) == 3 && ...
           gnss0.gnss_fix_ok(i) == 1 && ...
           gnss0.num_sv(i) >= 6 && ...
           gnss0.h_acc_m(i) <= 20.0 && ...
           gnss0.pdop(i) <= 6.0

            gnss_valid(n,1) = gnss0.gps_week(i);
            gnss_valid(n,2) = gnss0.gps_tow_ms(i)/1000;
            gnss_valid(n,3) = gnss0.lat_deg(i);
            gnss_valid(n,4) = gnss0.lon_deg(i);

            % Keep exactly the same height field as current LC code.
            gnss_valid(n,5) = gnss0.hmsl_m(i);

            gnss_valid(n,6) = gnss0.vel_n_m_s(i);
            gnss_valid(n,7) = gnss0.vel_e_m_s(i);
            gnss_valid(n,8) = gnss0.vel_d_m_s(i);

            gnss_valid(n,9)  = gnss0.v_acc_m(i);
            gnss_valid(n,10) = gnss0.h_acc_m(i);
            gnss_valid(n,11) = gnss0.s_acc_m_s(i);
            gnss_valid(n,12) = gnss0.pdop(i);

            heading_rad = atan2( ...
                gnss0.vel_e_m_s(i), ...
                gnss0.vel_n_m_s(i));

            if heading_rad < 0
                heading_rad = heading_rad+2*pi;
            end

            gnss_valid(n,13) = heading_rad*180/pi;

            n = n+1;
        end
    end

    gnss_valid(n:end,:) = [];
end


function imu_valid = build_imu_valid(imu0)

    len = size(imu0,1);
    imu_valid = zeros(len,10);

    for i = 1:len

        imu_valid(i,1) = imu0.gps_week(i);
        imu_valid(i,2) = imu0.gps_tow_us(i)/1e6;

        % Gyro: [deg/h] * dt, same storage convention as current script.
        imu_valid(i,3) = imu0.gx_deg_h(i)*imu0.dt_s(i);
        imu_valid(i,4) = imu0.gy_deg_h(i)*imu0.dt_s(i);
        imu_valid(i,5) = imu0.gz_deg_h(i)*imu0.dt_s(i);

        % Accelerometer delta-v in ORIGINAL sensor frame.
        imu_valid(i,6) = imu0.ax_m_s2(i)*imu0.dt_s(i);
        imu_valid(i,7) = imu0.ay_m_s2(i)*imu0.dt_s(i);
        imu_valid(i,8) = imu0.az_m_s2(i)*imu0.dt_s(i);

        imu_valid(i,9)  = imu0.temp_deg_c(i);
        imu_valid(i,10) = imu0.dt_s(i);
    end
end


function imu_out = apply_temp_comp(imu_in, TCs)
% 与 tools/calib24_static_numbered_tempcomp.m 完全一致的温补。
% drift 为物理量(m/s^2, deg/h)，而 imu_in(:,3:8) 存的是"增量"，
% 因此要乘 dt 再相减，量纲才对齐。
%
% 作用于陀螺与加计两者（仅限该轴 ord>0 的轴）。调用方在温补之后
% 才做"减静止段均值"，所以 Case 2/3 的陀螺零偏是温补后的残余均值。
%
%   imu_in(:,3:5) = g*dt  [deg/h * s]
%   imu_in(:,6:8) = a*dt  [m/s]

    dT     = imu_in(:,9) - TCs.Tref;
    dt_vec = imu_in(:,10);

    driftAcc = zeros(size(imu_in,1),3);   % m/s^2
    driftGyr = zeros(size(imu_in,1),3);   % deg/h

    for a = 1:3
        if TCs.tcActive(a)
            driftAcc(:,a) = polyval(TCs.TCpoly(a,:),   dT);
        end
        if TCs.tcActive(a+3)
            driftGyr(:,a) = polyval(TCs.TCpoly(a+3,:), dT);
        end
    end

    imu_out = imu_in;

    imu_out(:,3:5) = imu_out(:,3:5) - driftGyr .* dt_vec;
    imu_out(:,6:8) = imu_out(:,6:8) - driftAcc .* dt_vec;
end


function out = run_static_ins_case( ...
    imu_valid_raw, ...
    t_static_start,t_static_end, ...
    avp0, ...
    useTempComp,useAccCal, TCs,ba_cal,Ca_cal,cfg,caseName)

    global glv

    imu_valid = imu_valid_raw;

    %% Step 1: temperature compensation（整段逐样本，陀螺也在此步温补）
    % 陀螺处理顺序契约：温补 -> 减静止段均值（在下方统一执行）
    %   Case 1   : useTempComp=false -> 用未温补的陀螺
    %   Case 2/3 : useTempComp=true  -> 用温补后的陀螺
    if useTempComp
        imu_valid = apply_temp_comp(imu_valid, TCs);
    end

    %% Step 2: 24-position accelerometer deterministic calibration
    if useAccCal

        dt_all = imu_valid(:,10);
        dv_raw = imu_valid(:,6:8);

        dv_cal = ...
            (Ca_cal * ...
            (dv_raw'-ba_cal*dt_all'))';

        imu_valid(:,6:8) = dv_cal;
    end

    %% Cut EXACT same static interval
    mask = ...
        imu_valid(:,2) >= t_static_start & ...
        imu_valid(:,2) <= t_static_end;

    imu_static = imu_valid(mask,:);

    if size(imu_static,1) < cfg.nn
        error('Static interval contains too few IMU samples.');
    end

    %% Sensor frame -> RFU
    imu_rfu = [ ...
        [-imu_static(:,4), ...
          imu_static(:,3), ...
          imu_static(:,5)]*pi/180/3600, ...
        [-imu_static(:,7), ...
          imu_static(:,6), ...
          imu_static(:,8)], ...
        imu_static(:,2)];

    %% 陀螺零偏：减去静止段均值（三个 case 统一；温补已完成在前）
    %   Case 1     : 未温补陀螺的静止段均值
    %   Case 2 / 3 : 温补后陀螺的静止段均值
    gyro_static = mean(imu_rfu(:,1:3),1);

    imu_rfu(:,1:3) = ...
        imu_rfu(:,1:3)-gyro_static;

    gyro_static_deg_h = gyro_static/glv.dph;   % RFU 三轴，deg/h

    %% Static acceleration diagnostic
    dt_static = imu_static(:,10);

    acc_rfu = ...
        imu_rfu(:,4:6)./dt_static;

    acc_mean = mean(acc_rfu,1);

    fprintf('\n%s\n',caseName);
    fprintf('  temp comp applied before gyro mean removal : %d\n',useTempComp);
    fprintf('Static samples = %d\n',size(imu_static,1));
    fprintf('Gyro static mean removed [R F U, deg/h]: %.6f  %.6f  %.6f\n', ...
        gyro_static_deg_h(1),gyro_static_deg_h(2),gyro_static_deg_h(3));
    fprintf('Mean RFU acceleration:\n');
    fprintf('X = %.6f m/s^2\n',acc_mean(1));
    fprintf('Y = %.6f m/s^2\n',acc_mean(2));
    fprintf('Z = %.6f m/s^2\n',acc_mean(3));
    fprintf('|f| = %.6f m/s^2\n',norm(acc_mean));

    %% Pure INS initialization
    ins = insinit(avp0,cfg.ts);

    nn = cfg.nn;
    len = size(imu_rfu,1);

    nLog = floor(len/nn);

    tlog = zeros(nLog,1);
    att = zeros(nLog,3);
    vn = zeros(nLog,3);
    pos = zeros(nLog,3);

    ii = 1;

    %% PURE INS ONLY
    for k = 1:nn:len-nn+1

        k1 = k+nn-1;

        wvm = imu_rfu(k:k1,1:6);

        ins = insupdate(ins,wvm);

        tlog(ii) = imu_rfu(k1,7);
        att(ii,:) = ins.att';
        vn(ii,:) = ins.vn';
        pos(ii,:) = ins.pos';

        ii = ii+1;
    end

    tlog(ii:end) = [];
    att(ii:end,:) = [];
    vn(ii:end,:) = [];
    pos(ii:end,:) = [];

    %% Drift relative to the COMMON initial AVP
    att0 = avp0(1:3)';
    vel0 = avp0(4:6)';
    pos0 = avp0(7:9);

    dv = vn-vel0;

    datt = att-att0;
    datt = atan2(sin(datt),cos(datt));
    datt_deg = datt*180/pi;

    dpENU = zeros(size(pos,1),3);

    for i = 1:size(pos,1)
        dpENU(i,:) = ...
            llh_difference_enu(pos(i,:)',pos0)';
    end

    %% Metrics
    final_dv = dv(end,:);
    final_dp_enu = dpENU(end,:);

    dv_rms_axis = sqrt(mean(dv.^2,1));
    dv_rms_3d = sqrt(mean(sum(dv.^2,2)));

    dp_rms_axis = sqrt(mean(dpENU.^2,1));
    dp_rms_3d = sqrt(mean(sum(dpENU.^2,2)));

    %% Output
    out.caseName = caseName;
    out.t = tlog;

    out.att = att;
    out.vn = vn;
    out.pos = pos;

    out.dv = dv;
    out.datt_deg = datt_deg;
    out.dpENU = dpENU;

    out.acc_mean = acc_mean;
    out.gyro_static = gyro_static;
    out.gyro_static_deg_h = gyro_static_deg_h;
    out.useTempComp = useTempComp;
    out.useAccCal = useAccCal;

    out.final_dv = final_dv;
    out.final_dp_enu = final_dp_enu;

    out.dv_rms_axis = dv_rms_axis;
    out.dv_rms_3d = dv_rms_3d;

    out.dp_rms_axis = dp_rms_axis;
    out.dp_rms_3d = dp_rms_3d;
end


function dENU = llh_difference_enu(posEst,posRef)
% posEst / posRef:
%   [lat(rad); lon(rad); h(m)]
%
% Output:
%   estimate - reference in local ENU metres.

    a = 6378137.0;
    f = 1/298.257223563;
    e2 = f*(2-f);

    lat = posRef(1);
    h = posRef(3);

    s = sin(lat);

    RN = a/sqrt(1-e2*s^2);

    RM = ...
        a*(1-e2) / ...
        (1-e2*s^2)^(3/2);

    dLat = posEst(1)-posRef(1);
    dLon = posEst(2)-posRef(2);
    dH   = posEst(3)-posRef(3);

    dE = (RN+h)*cos(lat)*dLon;
    dN = (RM+h)*dLat;
    dU = dH;

    dENU = [dE;dN;dU];
end
