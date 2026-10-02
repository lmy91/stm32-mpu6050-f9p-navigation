%% compare_drift_time_windows.m
% ========================================================================
% 纯 INS 静态传播：不同积分窗口 (5/10/30/60/180 s) 下的速度/位置误差
%
% 对每个积分窗口 T，从静止段起点做纯 INS 传播 T 秒（无 KF/GNSS/ZUPT），
% 统计窗口末端的误差（真值 = 静止：速度 0、位置不变）：
%   速度误差  dv = v(T) - v0          [E N U] + 3D 范数
%   位置误差  dp = p(T) 相对 p0 的位移 [E N U] + 3D 范数
%
% 同时对比三个补偿层级（与 compare_acc_calibration_static_ins.m 完全一致）：
%   Case 1: raw                        （不补偿）
%   Case 2: raw + temp comp            （先温补）
%   Case 3: raw + temp comp + 24-pos   （温补后再补标定 Ca, ba）
%
% 陀螺零偏处理契约（与 compare_acc_calibration_static_ins.m 一致）：
%   Case 1     : 未温补的陀螺 -> 减静止段均值
%   Case 2 / 3 : 温补后的陀螺 -> 减静止段均值
%   即温补永远发生在"减静止段均值"之前。
%
% 与 compare_acc_calibration_static_ins.m 的差别：
%   - 初始 AVP 取静止段"起点"处的最近 GNSS 速度/位置（而非 LC 实验的
%     导航起始时刻），这样窗口积分的时间基准与静止真值一致。
%   - Case 3 的标定文件缺失时自动降级为只跑 Case 1/2（不报错中断）。
%
% 期望文件:
%   gnss.csv
%   imu.csv
%   data/calib24/temp_coeffs_raw.mat                 (温补系数矩阵)
%   data/calib24/calib24_result_tempcomp_azgxgy.mat  (24 位标定结果, 可选)
%
% 输出:
%   static_ins_drift_windows.mat
%   static_ins_drift_windows_summary.csv
%   figs/fig_windows_01_velocity_error_3d.png
%   figs/fig_windows_02_position_error_3d.png
%
% ========================================================================

clc;
clear;
close all;

global glv
glvs;

% 代码位于 code/experiments，数据和结果位于会话根目录。
codeDir = fileparts(mfilename('fullpath'));
scriptDir = fileparts(fileparts(codeDir));
repoRoot = fileparts(fileparts(fileparts(scriptDir)));

%% ============================== CONFIG ===============================

cfg.gnssFile = fullfile(scriptDir, 'gnss.csv');
cfg.imuFile  = fullfile(scriptDir, 'imu.csv');

% ---- 温补 / 标定产物（与 tools/calib24 流水线一致）----
cfg.calib24Dir = fullfile(repoRoot, 'data', 'calib24');
cfg.tempCoeffFile = fullfile(cfg.calib24Dir, 'temp_coeffs_raw.mat');
cfg.calibFile     = fullfile(cfg.calib24Dir, 'calib24_result_tempcomp_azgxgy.mat');

% 静止区间（GNSS 有效序号），与 compare_acc_calibration_static_ins.m 一致
cfg.n_static_begin_gnss = 20;
cfg.n_static_end_gnss   = 200;

% 积分窗口 [s]
cfg.windows = [1 10 30 60 180];

% Same PSINS IMU batching as current experiment.
cfg.nn = 2;
cfg.ts = 0.01;

% Fixed attitude (PSINS order: [pitch; roll; yaw])，与 compare 脚本一致
cfg.att0_deg = [-1.3230; -4.8818; 0.0];

% false = 用静止段起点处的最近 GNSS 速度作为 v0
% true  = 用 [0;0;0] m/s
cfg.forceZeroInitialVelocity = false;

% ---- 输出 ----
cfg.resultDir = fullfile(scriptDir, 'results', 'static_ins_drift_windows');
cfg.outMat = fullfile(cfg.resultDir, 'static_ins_drift_windows.mat');
cfg.outCsv = fullfile(cfg.resultDir, 'static_ins_drift_windows_summary.csv');
cfg.figDir = fullfile(scriptDir, 'figs');

%% ============================== READ ================================

fprintf('\n============================================================\n');
fprintf(' Pure INS static drift vs integration window (5/10/30/60/180 s)\n');
fprintf('============================================================\n');

gnss0 = readtable(cfg.gnssFile);
imu0  = readtable(cfg.imuFile);

gnss_valid    = build_gnss_valid(gnss0);
imu_valid_raw = build_imu_valid(imu0);

if cfg.n_static_end_gnss > size(gnss_valid,1)
    error('Static GNSS index exceeds valid GNSS count.');
end

%% ======================= LOAD TEMP COEFFICIENTS =====================

if ~isfile(cfg.tempCoeffFile)
    error(['Temp coefficient file not found:\n%s\n' ...
           'Run tools/calibration/fit_temp_bias_raw.m first.'], cfg.tempCoeffFile);
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

fprintf('\nTemperature compensation coefficients:\n');
fprintf('  file : %s\n', cfg.tempCoeffFile);
fprintf('  Tref = %.4f degC, fit range [%.4f, %.4f] degC\n', ...
    TCs.Tref, TCs.Tmin, TCs.Tmax);
fprintf('  axis order  [ax ay az gx gy gz]: %d %d %d %d %d %d\n', TCs.ord);
fprintf('  active axes [ax ay az gx gy gz]: %d %d %d %d %d %d\n', +TCs.tcActive);

%% ========================= LOAD CALIBRATION ==========================

haveCalib = isfile(cfg.calibFile);

if haveCalib
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
            warning(['标定结果内嵌的温补系数与本次温补所用系数不一致 ' ...
                     '(max|diff| = %.3g)，温补与标定并非同一套系数。'], dmax);
        end
    end
    if isfield(calib.result,'Tref') && abs(calib.result.Tref - TCs.Tref) > 1e-6
        warning('标定结果 Tref = %.6f 与温补 Tref = %.6f 不一致。', ...
            calib.result.Tref, TCs.Tref);
    end

    fprintf('\nCalibration parameters:\n');
    fprintf('ba [m/s^2] =\n'); disp(ba_cal);
    fprintf('Ca =\n');          disp(Ca_cal);
else
    ba_cal = [];
    Ca_cal = [];
    warning(['Calibration result not found:\n%s\n' ...
             'Case 3 (raw + temp comp + calib) 将被跳过；' ...
             '请先运行 tools/calibration/calib24_static_numbered_tempcomp.m。'], cfg.calibFile);
end

%% ====================== CASE DEFINITION ==============================

caseNames = {'1 raw', '2 raw + temp comp', '3 raw + temp comp + calib'};
useTC     = [false        true                   true];
useCal    = [false        false                  true ];

if ~haveCalib
    caseNames = caseNames(1:2);
    useTC     = useTC(1:2);
    useCal    = useCal(1:2);
end

nCase = numel(caseNames);

%% ====================== COMMON INITIAL AVP ===========================
% 初始状态取静止段"起点"处：姿态用固定值，速度/位置用最近的 GNSS。

t_static_start = gnss_valid(cfg.n_static_begin_gnss,2);
t_static_end   = gnss_valid(cfg.n_static_end_gnss,2);

fprintf('\nStatic replay interval:\n');
fprintf('start = %.6f s\n',t_static_start);
fprintf('end   = %.6f s\n',t_static_end);
fprintf('span  = %.3f s\n',t_static_end-t_static_start);

if (t_static_end - t_static_start) < max(cfg.windows)
    error(['Static interval span (%.1f s) is shorter than the largest ' ...
           'window (%.0f s).'], t_static_end-t_static_start, max(cfg.windows));
end

idx_imu0 = find(imu_valid_raw(:,2) >= t_static_start, 1, 'first');
if isempty(idx_imu0)
    error('No IMU sample found at/after static interval start.');
end
t0_static = imu_valid_raw(idx_imu0,2);

[~,iG0] = min(abs(gnss_valid(:,2) - t0_static));

pos0 = [ ...
    gnss_valid(iG0,3)*pi/180;
    gnss_valid(iG0,4)*pi/180;
    gnss_valid(iG0,5)];

vel0_gnss = [ ...
    gnss_valid(iG0,7);
    gnss_valid(iG0,6);
    -gnss_valid(iG0,8)];

if cfg.forceZeroInitialVelocity
    vel0 = [0;0;0];
else
    vel0 = vel0_gnss;
end

att0 = cfg.att0_deg*pi/180;

avp0 = [att0; vel0; pos0];

fprintf('\nCommon initial AVP (static interval start):\n');
fprintf('t0 = %.6f s\n', t0_static);
fprintf('att0 [pitch roll yaw] = %.6f  %.6f  %.6f deg\n', ...
    cfg.att0_deg(1), cfg.att0_deg(2), cfg.att0_deg(3));
fprintf('vel0 [E N U] = %.6f  %.6f  %.6f m/s\n', vel0(1),vel0(2),vel0(3));
fprintf('GNSS vel at t0 [E N U] = %.6f  %.6f  %.6f m/s\n', ...
    vel0_gnss(1),vel0_gnss(2),vel0_gnss(3));
fprintf('pos0 [lat lon h] = %.9f  %.9f  %.3f\n', ...
    pos0(1)*180/pi,pos0(2)*180/pi,pos0(3));

if ~cfg.forceZeroInitialVelocity && norm(vel0) > 0.05
    fprintf(['WARNING: initial GNSS speed is %.4f m/s although the replayed\n' ...
             '         IMU segment is static. dv = v(T)-v0 removes this offset.\n'], ...
             norm(vel0));
end

%% ==================== PREPARE SEGMENTS (3 CASES) ====================
% 每个 case 只准备一次整段静止数据（RFU + 陀螺零偏减除），
% 之后不同窗口只是从同一份数据里截取前 T 秒。

fprintf('\n============================================================\n');
fprintf(' Prepare static segments (per case)\n');
fprintf('============================================================\n');

SEG = cell(nCase,1);
for c = 1:nCase
    SEG{c} = prepare_segment(imu_valid_raw, ...
        t_static_start,t_static_end, useTC(c),useCal(c), TCs,ba_cal,Ca_cal, cfg);
end

%% ========================== RUN WINDOWS ==============================

nWin  = numel(cfg.windows);
nAx   = 4;                      % E N U 3D

dvEnd = zeros(nCase, nWin, nAx);   % 速度误差 (v(T)-v0)
dpEnd = zeros(nCase, nWin, nAx);   % 位置误差 (相对 p0 的 ENU 位移)
Tact  = zeros(nCase, nWin);        % 实际窗口时长

fprintf('\n============================================================\n');
fprintf(' Pure INS propagation over %d windows x %d cases\n', nWin, nCase);
fprintf('============================================================\n');

for c = 1:nCase
    for w = 1:nWin
        R = propagate_window(SEG{c}, t0_static, cfg.windows(w), avp0, cfg);

        Tact(c,w)      = R.T_actual;
        dvEnd(c,w,:)   = R.dv;
        dpEnd(c,w,:)   = R.dp;

        fprintf('%-26s  T=%5.1f s (actual %6.2f s)  |dv|3D=%9.5f m/s  |dp|3D=%9.3f m\n', ...
            caseNames{c}, cfg.windows(w), R.T_actual, R.dv(4), R.dp(4));
    end
end

%% =========================== SUMMARY TABLE ===========================

Win_s  = repelem(cfg.windows(:)', nCase);
CaseC  = repmat(string(caseNames(:)'), 1, nWin);
Win_s  = Win_s(:);
CaseC  = CaseC(:);

% dvEnd/dpEnd 的尺寸是 (nCase, nWin, 4)。MATLAB 列主序展开时第 1 维
% (case) 变化最快，因此直接 reshape 得到的行序是：
%   (w1,c1) (w1,c2) ... (w1,cN) | (w2,c1) ... | (wN,c1) ...
% 即"窗口在外层、case 在内层"，与上面 Win_s/CaseC 的标签顺序严格一致。
% （注意：这里不能先 permute 成 (nWin,nCase,4) 再 reshape，否则会变成
%   "case 在外层、窗口在内层"，与标签交叉错位——曾导致汇总表索引错乱。）
dvMat  = reshape(dvEnd, [], nAx);    % [nWin*nCase x 4]，行序同 Win_s/CaseC
dpMat  = reshape(dpEnd, [], nAx);    % [nWin*nCase x 4]，行序同 Win_s/CaseC

summaryTable = table(Win_s, CaseC, ...
    dvMat(:,1), dvMat(:,2), dvMat(:,3), dvMat(:,4), ...
    dpMat(:,1), dpMat(:,2), dpMat(:,3), dpMat(:,4), ...
    'VariableNames', {'Window_s','Case', ...
     'dvE_mps','dvN_mps','dvU_mps','dv3D_mps', ...
     'dpE_m','dpN_m','dpU_m','dp3D_m'});

fprintf('\n============================================================\n');
fprintf('          PURE INS STATIC DRIFT vs INTEGRATION WINDOW\n');
fprintf('  dv = v(T)-v0 [m/s]     dp = displacement from p0 [m]\n');
fprintf('============================================================\n');
fprintf('%7s  %-26s | %-30s | %-30s\n', ...
    'Window','Case','Velocity error [m/s]  E     N     U    3D', ...
                          'Position error [m]    E     N     U    3D');
fprintf('%7s  %-26s |%-30s|%-30s\n', '-------','--------------------------', ...
    '------------------------------','------------------------------');

% 行序与 summaryTable 一致：窗口分组、组内按 case
for r = 1:height(summaryTable)
    fprintf('%6.0fs  %-26s | %7.2f %6.2f %6.2f %8.2f | %10.2f %9.2f %9.2f %10.2f\n', ...
        summaryTable.Window_s(r), char(summaryTable.Case(r)), ...
        summaryTable.dvE_mps(r), summaryTable.dvN_mps(r), ...
        summaryTable.dvU_mps(r), summaryTable.dv3D_mps(r), ...
        summaryTable.dpE_m(r),  summaryTable.dpN_m(r), ...
        summaryTable.dpU_m(r),  summaryTable.dp3D_m(r));
end

disp(summaryTable);

%% ============================== PLOTS ================================
% IEEE 期刊风格：误差随积分窗口的增长曲线（横轴对数）。

if ~isempty(cfg.figDir) && ~exist(cfg.figDir,'dir')
    mkdir(cfg.figDir);
end

figFiles = {};

ieeeC  = [0.00 0.00 0.00;
          0.00 0.45 0.74;
          0.85 0.33 0.10];
ieeeLS = {'-','--','-.'};
ieeeMK = {'o','s','^'};
ieeeLW = 1.0;

legendLabels = string(caseNames(:)');

% 1. 3D velocity error vs window
fig1 = figure('Name','Velocity error vs integration window');
hold on;
for c = 1:nCase
    semilogx(cfg.windows, squeeze(dvEnd(c,:,4)), ...
        'Color',ieeeC(c,:), 'LineStyle',ieeeLS{c}, 'LineWidth',ieeeLW, ...
        'Marker',ieeeMK{c}, 'MarkerSize',4, 'MarkerFaceColor','w');
end
grid off;
xlabel('Integration window / s');
ylabel('3D velocity error / m/s');
title('Pure INS static velocity error');
legend(legendLabels,'Location','northwest');
figFiles{end+1} = save_fig(fig1, cfg.figDir, 'fig_windows_01_velocity_error_3d');

% 2. 3D position error vs window
fig2 = figure('Name','Position error vs integration window');
hold on;
for c = 1:nCase
    semilogx(cfg.windows, squeeze(dpEnd(c,:,4)), ...
        'Color',ieeeC(c,:), 'LineStyle',ieeeLS{c}, 'LineWidth',ieeeLW, ...
        'Marker',ieeeMK{c}, 'MarkerSize',4, 'MarkerFaceColor','w');
end
grid off;
xlabel('Integration window / s');
ylabel('3D position error / m');
title('Pure INS static position error');
legend(legendLabels,'Location','northwest');
figFiles{end+1} = save_fig(fig2, cfg.figDir, 'fig_windows_02_position_error_3d');

%% =============================== SAVE ================================

res.windows = cfg.windows;
res.caseNames = caseNames;
res.Tact = Tact;
res.dvEnd = dvEnd;
res.dpEnd = dpEnd;

if ~exist(cfg.resultDir, 'dir')
    mkdir(cfg.resultDir);
end
save(cfg.outMat, 'res','summaryTable','cfg','avp0','TCs','ba_cal','Ca_cal', ...
    't0_static','t_static_start','t_static_end');

writetable(summaryTable, cfg.outCsv);

fprintf('\nSaved:\n');
fprintf('  %s\n', cfg.outMat);
fprintf('  %s\n', cfg.outCsv);
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
            gnss_valid(n,5) = gnss0.hmsl_m(i);

            gnss_valid(n,6) = gnss0.vel_n_m_s(i);
            gnss_valid(n,7) = gnss0.vel_e_m_s(i);
            gnss_valid(n,8) = gnss0.vel_d_m_s(i);

            gnss_valid(n,9)  = gnss0.v_acc_m(i);
            gnss_valid(n,10) = gnss0.h_acc_m(i);
            gnss_valid(n,11) = gnss0.s_acc_m_s(i);
            gnss_valid(n,12) = gnss0.pdop(i);

            heading_rad = atan2(gnss0.vel_e_m_s(i), gnss0.vel_n_m_s(i));
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

        imu_valid(i,3) = imu0.gx_deg_h(i)*imu0.dt_s(i);
        imu_valid(i,4) = imu0.gy_deg_h(i)*imu0.dt_s(i);
        imu_valid(i,5) = imu0.gz_deg_h(i)*imu0.dt_s(i);

        imu_valid(i,6) = imu0.ax_m_s2(i)*imu0.dt_s(i);
        imu_valid(i,7) = imu0.ay_m_s2(i)*imu0.dt_s(i);
        imu_valid(i,8) = imu0.az_m_s2(i)*imu0.dt_s(i);

        imu_valid(i,9)  = imu0.temp_deg_c(i);
        imu_valid(i,10) = imu0.dt_s(i);
    end
end


function imu_out = apply_temp_comp(imu_in, TCs)
% 与 tools/calibration/calib24_static_numbered_tempcomp.m 一致的温补。
% drift 为物理量(m/s^2, deg/h)，imu_in(:,3:8) 为增量，需乘 dt。
% 作用于陀螺与加计（仅限该轴 ord>0 的轴）；调用方在其后才减静止段均值。

    dT     = imu_in(:,9) - TCs.Tref;
    dt_vec = imu_in(:,10);

    driftAcc = zeros(size(imu_in,1),3);
    driftGyr = zeros(size(imu_in,1),3);

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


function seg = prepare_segment( ...
    imu_valid_raw, t_static_start,t_static_end, ...
    useTempComp,useAccCal, TCs,ba_cal,Ca_cal, cfg)

% 准备某个 case 的整段静止数据：
%   温补(整段) -> 加计标定(整段) -> 截静止段 -> 转 RFU -> 减陀螺静止段均值

    global glv

    imu_valid = imu_valid_raw;

    if useTempComp
        imu_valid = apply_temp_comp(imu_valid, TCs);
    end

    if useAccCal
        dt_all = imu_valid(:,10);
        dv_raw = imu_valid(:,6:8);
        imu_valid(:,6:8) = (Ca_cal * (dv_raw' - ba_cal*dt_all'))';
    end

    mask = imu_valid(:,2) >= t_static_start & imu_valid(:,2) <= t_static_end;
    imu_static = imu_valid(mask,:);

    if size(imu_static,1) < cfg.nn
        error('Static interval contains too few IMU samples.');
    end

    imu_rfu = [ ...
        [-imu_static(:,4), imu_static(:,3), imu_static(:,5)]*pi/180/3600, ...
        [-imu_static(:,7), imu_static(:,6), imu_static(:,8)], ...
        imu_static(:,2)];

    % 陀螺零偏：减静止段均值（温补已完成在前）
    gyro_static = mean(imu_rfu(:,1:3),1);
    imu_rfu(:,1:3) = imu_rfu(:,1:3) - gyro_static;

    seg.t   = imu_rfu(:,7);
    seg.imu = imu_rfu(:,1:6);
    seg.gyro_static_deg_h = gyro_static/glv.dph;
    seg.useTempComp = useTempComp;
    seg.useAccCal   = useAccCal;

    % 诊断输出
    dt_static = imu_static(:,10);
    acc_rfu   = imu_rfu(:,4:6)./dt_static;
    acc_mean  = mean(acc_rfu,1);

    fprintf('\nSegment prepared:\n');
    fprintf('  temp comp before gyro mean removal : %d\n', useTempComp);
    fprintf('  acc calib applied                  : %d\n', useAccCal);
    fprintf('  static samples                     : %d\n', size(imu_static,1));
    fprintf('  gyro static mean removed [R F U, deg/h]: %.6f  %.6f  %.6f\n', ...
        seg.gyro_static_deg_h(1), seg.gyro_static_deg_h(2), seg.gyro_static_deg_h(3));
    fprintf('  mean RFU acc [X Y Z] = %.6f  %.6f  %.6f m/s^2, |f| = %.6f\n', ...
        acc_mean(1), acc_mean(2), acc_mean(3), norm(acc_mean));
end


function out = propagate_window(seg, t0, T, avp0, cfg)
% 从 seg 中截取 [t0, t0+T] 做纯 INS 传播，返回末端误差。

    mask = seg.t >= t0 & seg.t <= t0 + T;
    imu  = seg.imu(mask,:);
    tt   = seg.t(mask);

    if size(imu,1) < cfg.nn
        error('Window %.0f s contains too few IMU samples.', T);
    end

    ins = insinit(avp0, cfg.ts);

    nn = cfg.nn;
    for k = 1:nn:size(imu,1)-nn+1
        ins = insupdate(ins, imu(k:k+nn-1,:));
    end

    vel0 = avp0(4:6)';
    pos0 = avp0(7:9);

    dv = ins.vn' - vel0;
    dp = llh_difference_enu(ins.pos, pos0)';

    out.dv = [dv, norm(dv)];
    out.dp = [dp, norm(dp)];
    out.T_actual = tt(end) - tt(1);
    out.vn_end = ins.vn';
    out.pos_end = ins.pos;
end


function dENU = llh_difference_enu(posEst,posRef)
% [lat(rad); lon(rad); h(m)] -> estimate - reference in local ENU metres.

    a = 6378137.0;
    f = 1/298.257223563;
    e2 = f*(2-f);

    lat = posRef(1);
    h = posRef(3);

    s = sin(lat);

    RN = a/sqrt(1-e2*s^2);
    RM = a*(1-e2) / (1-e2*s^2)^(3/2);

    dLat = posEst(1)-posRef(1);
    dLon = posEst(2)-posRef(2);
    dH   = posEst(3)-posRef(3);

    dENU = [ (RN+h)*cos(lat)*dLon; (RM+h)*dLat; dH ];
end


function fp = save_fig(fh, figDir, baseName, heightIn, widthIn, fontSize, dpi)
% 按 IEEE 期刊规范美化并保存 figure（PNG）。
    if nargin < 7 || isempty(dpi),      dpi      = 600; end
    if nargin < 6 || isempty(fontSize), fontSize = 8;   end
    if nargin < 5 || isempty(widthIn),  widthIn  = 3.5; end
    if nargin < 4 || isempty(heightIn)
        nAx = numel(findall(fh,'Type','axes'));
        heightIn = 1.6 + 1.3*max(nAx,1);
    end

    apply_ieee_style(fh, widthIn, heightIn, fontSize);

    fp = fullfile(figDir, [baseName '.png']);
    try
        exportgraphics(fh, fp, 'Resolution', dpi);
    catch
        print(fh, fp, '-dpng', sprintf('-r%d', dpi));
    end
end


function apply_ieee_style(fh, widthIn, heightIn, fontSize)
% IEEE 期刊风格：白底 / Times New Roman / 刻度朝内 / 四边全框 / 无网格
    set(fh, 'Color','w', ...
        'Units','inches', ...
        'Position',[1 1 widthIn*1.4 heightIn*1.4], ...
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
