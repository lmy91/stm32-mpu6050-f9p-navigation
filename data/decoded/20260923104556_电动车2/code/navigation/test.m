clc
clear
close all

% 代码位于 code/navigation，数据和结果位于会话根目录。
code_dir = fileparts(mfilename('fullpath'));
script_dir = fileparts(fileparts(code_dir));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(fullfile(repo_root,'tools','visualization'));  % position_to_kml

%% =========================================================
% 补偿开关与系数文件
%
% 处理链（与 tools/calibration/calib24_static_numbered_tempcomp.m 完全一致）：
%   原始输出 -> 温度补偿(temp comp) -> 24 位加计标定
%
% 温度补偿: drift(T) = sum(c_m*dT^m, m=1..各轴阶数), dT = T - Tref
%           不补 0 阶 c0；未拟合轴整行为 0 => polyval 得 0 => 该轴不补偿
% 加计标定: dv_cal = Ca * (dv_tc - ba*dt)
% =========================================================
temp_com     = 1;   % 1 = 先对原始输出做温度补偿
acc_bias_com = 1;   % 1 = 温补之后再套用 24 位加计标定结果

calib_dir = fullfile(repo_root, 'data', 'calib24');

% 温补系数矩阵（fit_temp_bias_raw.m 产物），6x6 [c5..c0]，行序 [ax ay az gx gy gz]
tempCoeffFile = fullfile(calib_dir, 'temp_coeffs_raw.mat');

% 24 位标定结果（calib24_static_numbered_tempcomp.m 产物，内部已含温补系数）
calibResultFile = fullfile(calib_dir, 'calib24_result_tempcomp_azgxgy.mat');

% 0930 最终按轴温补 Allan 参数（GX/GY/AZ 已温补，GZ/AX/AY 未温补）
% 这里只再用于 ARW/VRW 测量白噪声，不再用 BI/平台区间设置 GM 参数。
allanParamFile = fullfile(repo_root, 'data', 'allan_results', ...
    'mpu_21.1h_20260930', 'allan_parameters.csv');

% 21 h 原始数据完成温补、24 位置标定后的自相关 GM 候选参数。
% 独立分段/跨数据验证见 tools/noise_analysis/gm_validate_parameters.m；此处不自动调参。
gmParamFile = fullfile(repo_root, 'data', 'decoded', '20260926005735', ...
    'gm_autocorrelation', 'gm_parameters.csv');

% 图片输出目录（会话根目录下的 figs/）
fig_dir = fullfile(script_dir, 'figs');

% Google Earth：1 Hz组合导航轨迹；默认贴地显示，坐标仍保留HMSL高度。
% 改为 'absolute' 可按海拔显示三维轨迹（本数据高度来自GNSS hmsl_m）。
kml_file = fullfile(script_dir,'kml','combined_navigation_1hz.kml');
kml_altitude_mode = 'clampToGround';

%% IEEE 绘图默认样式（白底细线 + 黑白可辨配色）
%  注意：下面两行会改变本 MATLAB 会话中后续新建图表的默认配色与线宽。
set(0,'DefaultAxesColorOrder', [0.00 0.00 0.00;
                                0.00 0.45 0.74;
                                0.85 0.33 0.10;
                                0.49 0.18 0.56;
                                0.47 0.67 0.19]);
set(0,'DefaultLineLineWidth', 1.0);

%% GNSS
gnss0 = readtable(fullfile(script_dir, 'gnss.csv'));
len = size(gnss0,1);
n = 1;
gnss_valid = zeros(len,13);
for i = 1:len
    if gnss0.fix(i) == 3 && gnss0.gnss_fix_ok(i) == 1 ...
            && gnss0.num_sv(i) >= 6 && gnss0.h_acc_m(i) <= 20.0 ...
            && gnss0.pdop(i) <= 6.0
        gnss_valid(n,1) = gnss0.gps_week(i);
        gnss_valid(n,2) = gnss0.gps_tow_ms(i)/1000;
        gnss_valid(n,3) = gnss0.lat_deg(i);
        gnss_valid(n,4) = gnss0.lon_deg(i);
        gnss_valid(n,5) = gnss0.hmsl_m(i);
        gnss_valid(n,6) = gnss0.vel_n_m_s(i);
        gnss_valid(n,7) = gnss0.vel_e_m_s(i);
        gnss_valid(n,8) = gnss0.vel_d_m_s(i);
        gnss_valid(n,9) = gnss0.v_acc_m(i);
        gnss_valid(n,10) = gnss0.h_acc_m(i);
        gnss_valid(n,11) = gnss0.s_acc_m_s(i);
        gnss_valid(n,12) = gnss0.pdop(i);

        heading_rad = atan2(gnss0.vel_e_m_s(i), gnss0.vel_n_m_s(i));
        if heading_rad < 0
            heading_rad = heading_rad + 2*pi;
        end
        heading_deg = heading_rad * 180 / pi;
        gnss_valid(n,13) = heading_deg;
        n = n+1;
    end
end
gnss_valid(n:end,:) = [];
% gnss1 = gnss_valid(25:end,:);
% gnss2 = [gnss1(:,7) gnss1(:,6) -gnss1(:,8) gnss1(:,3)*pi/180 gnss1(:,4)*pi/180 gnss1(:,5) gnss1(:,2)];
% gnss2 = gnss2(2:end,:);
%% IMU
imu0 = readtable(fullfile(script_dir, 'imu.csv'));
len = size(imu0,1);

imu_valid = zeros(len,10);
for i = 1:len
    imu_valid(i,1) = imu0.gps_week(i);
    imu_valid(i,2) = imu0.gps_tow_us(i)/1000000;
    imu_valid(i,3) = imu0.gx_deg_h(i)*imu0.dt_s(i);
    imu_valid(i,4) = imu0.gy_deg_h(i)*imu0.dt_s(i);
    imu_valid(i,5) = imu0.gz_deg_h(i)*imu0.dt_s(i);
    imu_valid(i,6) = imu0.ax_m_s2(i)*imu0.dt_s(i);
    imu_valid(i,7) = imu0.ay_m_s2(i)*imu0.dt_s(i);
    imu_valid(i,8) = imu0.az_m_s2(i)*imu0.dt_s(i);
    imu_valid(i,9) = imu0.temp_deg_c(i);
    imu_valid(i,10) = imu0.dt_s(i);

end
%% =========================================================
% 温度补偿（第一步：先温补）
% =========================================================
% imu_valid 列约定：
%   (:,3:5) = g*dt   [deg/h * s]，原始传感器帧
%   (:,6:8) = a*dt   [m/s]      ，原始传感器帧（速度增量）
%   (:,9)   = temp_deg_c
%   (:,10)  = dt_s
% 温补系数与 calib24 脚本同源：drift 是物理量(m/s^2, deg/h)，
% 因此这里要乘 dt 换算成增量后再相减，量纲才与 imu_valid 一致。
if temp_com == 1

    if ~isfile(tempCoeffFile)
        error(['Temp coefficient file not found:\n%s\n' ...
               'Run tools/calibration/fit_temp_bias_raw.m first.'], tempCoeffFile);
    end

    TC = load(tempCoeffFile);
    assert(isfield(TC,'coef') && size(TC.coef,1)==6, ...
        'temp_coeffs_raw.mat 必须包含 6 行的 coef 矩阵.');

    TCpoly = [TC.coef(:,1:end-1), zeros(6,1)];   % [c5..c1 0]，常数项显式给 0
    if isfield(TC,'ord')
        ordTC = TC.ord(:)';
    else
        ordTC = TC.axisOrder(:)';
    end
    tcActive = ordTC > 0;
    Tref     = TC.Tref;

    if any(imu_valid(:,9) < TC.Tmin-1e-9 | imu_valid(:,9) > TC.Tmax+1e-9)
        error('IMU temperature exceeds fitted range [%.4f, %.4f] degC.', ...
            TC.Tmin, TC.Tmax);
    end

    dT     = imu_valid(:,9) - Tref;
    dt_vec = imu_valid(:,10);

    driftAcc = zeros(len,3);   % m/s^2
    driftGyr = zeros(len,3);   % deg/h
    for a = 1:3
        if tcActive(a)
            driftAcc(:,a) = polyval(TCpoly(a,   :), dT);
        end
        if tcActive(a+3)
            driftGyr(:,a) = polyval(TCpoly(a+3, :), dT);
        end
    end

    % 陀螺: g*dt - drift*dt
    imu_valid(:,3:5) = imu_valid(:,3:5) - driftGyr .* dt_vec;
    % 加计: a*dt - drift*dt
    imu_valid(:,6:8) = imu_valid(:,6:8) - driftAcc .* dt_vec;

    fprintf('========================================\n');
    fprintf('温度补偿已应用 (temp comp)\n');
    fprintf('  系数文件 : %s\n', tempCoeffFile);
    fprintf('  Tref = %.4f degC, 拟合温区 [%.4f, %.4f] degC\n', ...
        Tref, TC.Tmin, TC.Tmax);
    fprintf('  各轴阶数   [ax ay az gx gy gz] : %d %d %d %d %d %d\n', ordTC);
    fprintf('  有效温补轴 [ax ay az gx gy gz] : %d %d %d %d %d %d\n', +tcActive);
    fprintf('========================================\n');

end
if acc_bias_com == 1
    %% =========================================================
    % 24-position deterministic accelerometer calibration
    % Calibration parameters are expressed in original CSV sensor frame
    % ==========================================================

    if ~isfile(calibResultFile)
        error(['Calibration result not found:\n%s\n' ...
               'Run tools/calibration/calib24_static_numbered_tempcomp.m first.'], ...
               calibResultFile);
    end

    calib = load(calibResultFile);

    ba_cal = calib.result.ba(:);     % [m/s^2], 3x1
    Ca_cal = calib.result.Ca;        % 3x3

    % ---- 自检：标定内置温补系数 / Tref 是否与本次温补一致 ----
    if temp_com == 1
        if isfield(calib.result,'tempCoeff')
            dmax = max(abs(calib.result.tempCoeff(:) - TCpoly(:)));
            if dmax > 1e-9
                error(['标定结果内嵌的温补系数与本次温补所用系数不一致 ' ...
                       '(max|diff| = %.3g)。'], dmax);
            end
        else
            error('标定结果缺少 result.tempCoeff，无法验证参数绑定。');
        end
        if isfield(calib.result,'Tref') && ...
                abs(calib.result.Tref - Tref) > 1e-6
            error('标定结果 Tref = %.6f 与温补 Tref = %.6f 不一致。', ...
                calib.result.Tref, Tref);
        end
        if ~isfield(calib.result,'tcActive') || ...
                ~isequal(logical(calib.result.tcActive(:)'), logical(tcActive))
            error('标定结果的有效温补轴与温补系数不一致。');
        end
    else
        warning(['标定结果基于"温补后"的数据求解，但当前 temp_com = 0（未温补）；' ...
                 'Ca/ba 与原始数据不匹配，结果仅作参考。']);
    end

    fprintf('Loaded accelerometer calibration:\n');

    fprintf('ba =\n');
    disp(ba_cal);

    fprintf('Ca =\n');
    disp(Ca_cal);
    % =========================================================
    % Accelerometer 24-position calibration（第二步：补标定结果）
    %
    % imu_valid(:,6:8) 此时已经是"温补后"的 delta-v（原始传感器帧）
    %
    % dv_cal = Ca * (dv_tc - ba * dt)
    % ==========================================================

    dt_all = imu_valid(:,10);           % N x 1

    dv_raw = imu_valid(:,6:8);          % N x 3

    dv_cal = ...
        (Ca_cal * ...
        (dv_raw' - ba_cal * dt_all'))';

    imu_valid(:,6:8) = dv_cal;
end
%% IMU desired start
n_start_gnss = 200;
n_end_gnss = 800;

n_static_begin_gnss = 20;
n_static_end_gnss = 200;

t_static_start = gnss_valid(n_static_begin_gnss,2);
t_static_end   = gnss_valid(n_static_end_gnss,2);

t_start = gnss_valid(n_start_gnss,2);
t_end   = gnss_valid(n_end_gnss,2);

imu_mask = imu_valid(:,2) >= t_start & ...
    imu_valid(:,2) <= t_end;

imu1 = imu_valid(imu_mask,:); % 前左上 增量

%% IMU静止区间
imu_static_mask = imu_valid(:,2) >= t_static_start & ...
    imu_valid(:,2) <= t_static_end;

imu_static = imu_valid(imu_static_mask,:); % 前左上 增量

%% GNSS cut using the SAME time interval
gnss_mask = gnss_valid(:,2) >= t_start & ...
    gnss_valid(:,2) <= t_end;

gnss1 = gnss_valid(gnss_mask,:);

gnss2 = [ ...
    gnss1(:,7), ...              % VE
    gnss1(:,6), ...              % VN
    -gnss1(:,8), ...              % VU
    gnss1(:,3)*pi/180, ...       % lat
    gnss1(:,4)*pi/180, ...       % lon
    gnss1(:,5), ...              % h
    gnss1(:,2)];                 % GPS TOW


fprintf('IMU first  : %.6f\n', imu1(1,2));
fprintf('GNSS first : %.6f\n', gnss1(1,2));
fprintf('First GNSS - IMU = %.6f s\n', ...
    gnss1(1,2)-imu1(1,2));
%%
% M = [0 -1 0;
%      1 0 0;
%      0 0 1];
imu2 = [[-imu1(:,4) imu1(:,3) imu1(:,5)]*pi/180/3600 ...
    [-imu1(:,7) imu1(:,6) imu1(:,8)] imu1(:,2)];  % 右前上 增量

imu_static1 = [[-imu_static(:,4) imu_static(:,3) imu_static(:,5)]*pi/180/3600 ...
    [-imu_static(:,7) imu_static(:,6) imu_static(:,8)] imu_static(:,2)];

imu_dt = imu1(:,10);
static_dt = imu_static(:,10);
assert(all(isfinite(imu_dt)) && all(imu_dt>0) && ...
       all(isfinite(static_dt)) && all(static_dt>0), ...
    'IMU integration intervals must be finite and positive.');
assert(all(diff(imu1(:,2))>0), 'IMU GPS timestamps must be strictly increasing.');
% First estimate a time-weighted static angular rate [rad/s], then remove
% that rate times each actual integration interval [s] from its increment.
gyro_static_rate = sum(imu_static1(:,1:3),1) / sum(static_dt);
imu2(:,1:3) = imu2(:,1:3)-gyro_static_rate.*imu_dt;

%% 加表零偏
dt = imu_static(:,10);
acc_rfu = imu_static1(:,4:6)./dt;

acc_mean = mean(acc_rfu,1);
acc_std  = std(acc_rfu,0,1);

fprintf('Mean RFU acceleration:\n');
fprintf('X = %.6f m/s^2\n',acc_mean(1));
fprintf('Y = %.6f m/s^2\n',acc_mean(2));
fprintf('Z = %.6f m/s^2\n',acc_mean(3));

fprintf('|f| = %.6f m/s^2\n',norm(acc_mean));
%%
glvs;
psinstypedef(186);
% ts is only the initialization value; every propagation uses its recorded dt.
[nn, ts, nts] = nnts(2, median(imu_dt));
%% MPU6050 first-order GM parameters from calibrated autocorrelation
% eb/db 是独立的初始协方差，不由 Allan BI 直接覆盖。
% eb=20 deg/h：减去启机静态均值后，按地球自转量级设置。
% db=5000 ug：暂用工程初值，后续由加速度计零偏重复性实验替换。
eb = [20;20;20];
db = [5000;5000;5000];

assert(isfile(allanParamFile), '0930 Allan parameter file not found: %s', allanParamFile);
A = readtable(allanParamFile, 'TextType','string');
requiredAllan = {'sensor','axis','arw_vrw'};
assert(all(ismember(requiredAllan,A.Properties.VariableNames)), ...
    '0930 Allan parameter CSV is missing required columns.');

axisName = ["x","y","z"];
arw_g_sensor = zeros(3,1);       % deg/sqrt(h)
vrw_a_sensor = zeros(3,1);       % m/s/sqrt(h)
for a = 1:3
    ig = strcmpi(A.sensor,'gyro') & strcmpi(A.axis,axisName(a));
    ia = strcmpi(A.sensor,'accel') & strcmpi(A.axis,axisName(a));
    assert(nnz(ig)==1 && nnz(ia)==1, ...
        '0930 Allan file must have exactly one row per sensor axis.');
    arw_g_sensor(a) = A.arw_vrw(ig);
    vrw_a_sensor(a) = A.arw_vrw(ia);
end

% 原始传感器坐标 [X,Y,Z] -> 右-前-上 RFU [-Y,X,Z]。
% 噪声标准差和时间常数只需交换 X/Y，符号不影响方差。
rfu = [2 1 3];
web = arw_g_sensor(rfu);                         % deg/sqrt(h)
wdb = vrw_a_sensor(rfu) / (60*glv.ug);          % ug/sqrt(Hz)

% 旧 GM 参数来源（已停用）：
%   sqrtR0G = [12.3918; 10.5314; 5.54271];    % deg/h
%   TauG    = [9.37; 27.19; 58.18];           % s
%   sqrtR0A = [48.5205; 40.8116; 109.239];    % ug
%   TauA    = [58.18; 196.39; 23.35];         % s
% 后续曾使用的 Allan 工程近似（同样停用）：
%   sqrtR0G = Allan bias_instability;
%   TauG    = sqrt(bias_tau_start_s*bias_tau_end_s);
%   sqrtR0A = 1000*Allan bias_instability;
%   TauA    = sqrt(bias_tau_start_s*bias_tau_end_s);
% 以上把 Allan BI 与平台区间当作一阶 GM 参数，仅为早期工程近似。

assert(isfile(gmParamFile), 'GM autocorrelation parameter file not found: %s', gmParamFile);
G = readtable(gmParamFile, 'TextType','string');
requiredGM = {'Sensor','Axis','GMStdEngineering','EngineeringUnit', ...
    'CorrelationTime_s','Reliable','FitStatus'};
assert(all(ismember(requiredGM,G.Properties.VariableNames)), ...
    'GM autocorrelation CSV is missing required columns.');

gm_g_sensor = zeros(3,1);        % deg/h, continuous GM stationary sigma
tau_g_sensor = zeros(3,1);       % s
gm_a_sensor = zeros(3,1);        % mg, continuous GM stationary sigma
tau_a_sensor = zeros(3,1);       % s
gmReliable = false(6,1);
for a = 1:3
    ig = strcmpi(G.Sensor,'gyro') & strcmpi(G.Axis,axisName(a));
    ia = strcmpi(G.Sensor,'accel') & strcmpi(G.Axis,axisName(a));
    assert(nnz(ig)==1 && nnz(ia)==1, ...
        'GM CSV must have exactly one row per sensor axis.');
    assert(strcmpi(G.EngineeringUnit(ig),'deg/h') && ...
           strcmpi(G.EngineeringUnit(ia),'mg'), ...
        'GM CSV unit mismatch: gyro must be deg/h and accel must be mg.');
    assert(strcmpi(G.FitStatus(ig),'OK') && strcmpi(G.FitStatus(ia),'OK'), ...
        'GM autocorrelation fit failed for sensor axis %s.', axisName(a));

    gm_g_sensor(a) = G.GMStdEngineering(ig);
    tau_g_sensor(a) = G.CorrelationTime_s(ig);
    gm_a_sensor(a) = G.GMStdEngineering(ia);
    tau_a_sensor(a) = G.CorrelationTime_s(ia);
    gmReliable(a) = logical(G.Reliable(ia));
    gmReliable(a+3) = logical(G.Reliable(ig));
end
assert(all(isfinite([gm_g_sensor; tau_g_sensor; gm_a_sensor; tau_a_sensor])) && ...
       all([gm_g_sensor; tau_g_sensor; gm_a_sensor; tau_a_sensor] > 0), ...
    'GM autocorrelation parameters must be finite and positive.');
if ~all(gmReliable)
    warning(['Some GM axes are marked Reliable=false under the identification-script criteria. ' ...
        'All GM values remain candidates pending independent validation; ' ...
        'see tools/noise_analysis/gm_validate_parameters.m.']);
end

sqrtR0G = gm_g_sensor(rfu);                      % deg/h, GM stationary sigma
TauG = tau_g_sensor(rfu);                        % s, autocorrelation time
sqrtR0A = 1000*gm_a_sensor(rfu);                 % mg -> ug, GM stationary sigma
TauA = tau_a_sensor(rfu);                        % s, autocorrelation time

imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA);

fprintf('Bias model: first-order Gauss-Markov using candidate calibrated-autocorrelation parameters\n');
fprintf('Independent GM validation: tools/noise_analysis/gm_validate_parameters.m (no automatic retuning)\n');
fprintf('Allan white-noise parameter file: %s\n', allanParamFile);
fprintf('GM autocorrelation parameter file: %s\n', gmParamFile);
fprintf('Gyro ARW RFU [x y z] = %.6f %.6f %.6f deg/sqrt(h)\n', web);
fprintf('Accel VRW RFU [x y z] = %.3f %.3f %.3f ug/sqrt(Hz)\n', wdb);
fprintf('Gyro GM sigma RFU [x y z] = %.6f %.6f %.6f deg/h\n', sqrtR0G);
fprintf('Gyro tau RFU [x y z] = %.3f %.3f %.3f s\n', TauG);
fprintf('Accel GM sigma RFU [x y z] = %.6f %.6f %.6f ug\n', sqrtR0A);
fprintf('Accel tau RFU [x y z] = %.3f %.3f %.3f s\n', TauA);
% Initial navigation-state uncertainty
davp0 = avperrset( ...
    [120;120;600], ...      % arcmin = 2°,2°,10°
    [0.5;0.5;0.5], ...      % m/s
    [5;5;10]);              % m


t0 = imu1(1,2);

[~,idx0] = min(abs(gnss_valid(:,2)-t0));

fprintf('Initial AVP GNSS dt = %.6f s\n', ...
    gnss_valid(idx0,2)-t0);

pos0 = [ ...
    gnss_valid(idx0,3)*pi/180;
    gnss_valid(idx0,4)*pi/180;
    gnss_valid(idx0,5)];

vel0 = [ ...
    gnss_valid(idx0,7);
    gnss_valid(idx0,6);
    -gnss_valid(idx0,8)];

yaw = 0*pi/180;
[attsb, qnb] = alignsb(imu_static1, pos0 ,yaw);
att0 = attsb;
avp0 = [att0;vel0;pos0];

ins = insinit(avp0, ts);
ins.tauG = imuerr.taug;
ins.tauA = imuerr.taua;

lever0 = [-0.14;0.16;0.50];     % RFU实际杆臂

ins.lever = lever0;
ins = inslever(ins);

lever_std = [0.02;0.02;0.05];   % 测量不确定度

% Initial GNSS R uses the first receiver-reported accuracy epoch.
% gnss1 columns: 9=v_acc_m, 10=h_acc_m, 11=s_acc_m_s.
r0 = gnss_measurement_std(gnss1(1,11),gnss1(1,10),gnss1(1,9),ins);
kf = kfinit(ins, davp0, imuerr, lever_std, r0);

%% ZUPT configuration
% State:
% 1:3   phi
% 4:6   dv
% 7:9   dp
% 10:12 eb
% 13:15 db
% 16:18 lever

H_zupt = [zeros(3,3), eye(3), zeros(3,12)];

sigma_zupt = 0.05;                % m/s
R_zupt = diag([sigma_zupt, ...
    sigma_zupt, ...
    sigma_zupt].^2);

% Stop detector
v_enter = 0.15;                   % m/s
v_exit  = 0.35;                   % m/s
gyro_static_th = 1.5;             % deg/s

static_count = 0;
zupt_active = false;

% Preserve the previous effective ZUPT rate (50 Hz / 10 = 5 Hz), but use
% GPS time rather than a sample counter when the IMU intervals vary.
zupt_period_s = 0.2;
next_zupt_time = imu1(1,2)-imu_dt(1)+zupt_period_s;

zupt_log = zeros(fix(len/nn),2);  % [time, active]
iz = 1;


%%
imu = imu2; % 右前上 增量
gps = gnss2;
imugpssyn(imu(:,7), gps(:,7));


len = length(imu);

num_steps = ceil(len/nn);  % include a final unpaired sample, if present
xfb_log = prealloc(num_steps, kf.n+1);
[avp, xkpk] = prealloc(num_steps, 10, 2*kf.n+1);

bias_log = zeros(num_steps,7);
zupt_log = zeros(num_steps,2);
avpL = zeros(num_steps,10);
nav_position_log = zeros(num_steps,4);  % 每次传播后天线端 [lat(rad),lon(rad),h(m),GPS TOW(s)]
timebar(nn, len, '18-state SINS/GPS.');

ki = 1;
iz = 1;

static_count = 0;
zupt_active = false;
next_zupt_time = imu1(1,2)-imu_dt(1)+zupt_period_s;

for k = 1:nn:len

    k1 = min(k + nn - 1, len);

    wvm = imu(k:k1,1:6);
    dt_pair = imu_dt(k:k1);
    t   = imu(k1,end);

    %% =========================================
    % 1. SINS propagation
    %% =========================================
    [ins, Cnb_mid] = insupdate_actual_time(ins, wvm, dt_pair);
    nts = ins.nts;

    %% =========================================
    % 2. KF prediction
    %% =========================================
    % kffk uses ins.nts, including its existing large-interval expm branch.
    % Qt remains in RFU/body noise coordinates; Gammak rotates only the
    % gyro/accelerometer white-noise blocks into navigation coordinates.
    % The GM driving noises and their bias states remain in body coordinates.
    kf.nts = nts;
    kf.Phikk_1 = kffk(ins);
    kf.Qk = kf.Qt*nts;
    kf.Gammak = eye(kf.n);
    kf.Gammak(1:3,1:3) = -Cnb_mid;
    kf.Gammak(4:6,4:6) = Cnb_mid;
    kf = kfupdate(kf);

    meas_updated = false;

    %% =========================================
    % 3. GNSS update
    %% =========================================
    [kgps, dt_sync] = imugpssyn(k, k1, 'F');

    if kgps > 0

        vnGPS  = gps(kgps,1:3)';
        posGPS = gps(kgps,4:6)';

        ins = inslever(ins);

        % Epoch-wise GNSS measurement model and covariance.
        % Velocity: receiver s_acc for E/N/U.
        % Position: h_acc for horizontal, v_acc for vertical; horizontal
        % metre-level accuracy is converted to latitude/longitude radians.
        kf.Hk = kfhk(ins);
        r_gnss = gnss_measurement_std( ...
            gnss1(kgps,11), ...  % speed RMSE [m/s]
            gnss1(kgps,10), ...  % horizontal position RMSE [m]
            gnss1(kgps,9), ...   % vertical position RMSE [m]
            ins);
        kf.Rk = diag(r_gnss.^2);

        zk = [ ...
            ins.vnL - ins.an*dt_sync - vnGPS;
            ins.posL - ins.Mpvvn*dt_sync - posGPS ];

        kf = kfupdate(kf, zk, 'M');

        meas_updated = true;

        %% -----------------------------------------
        % GNSS-based stop detection
        %% -----------------------------------------
        v_horizontal = hypot(vnGPS(1), vnGPS(2));

        if v_horizontal < v_enter

            static_count = static_count + 1;

            % 2 consecutive GNSS epochs
            if static_count >= 2
                zupt_active = true;
            end

        elseif v_horizontal > v_exit

            % leave stationary state immediately
            static_count = 0;
            zupt_active = false;

        end
    end

    %% =========================================
    % 4. IMU quiet check
    %% =========================================

    % wvm contains angular increments in rad
    % sum over this block / its actual duration => rad/s
    omega_b = sum(wvm(:,1:3),1)' / nts;

    gyro_norm_dps = norm(omega_b) / glv.deg;

    imu_quiet = gyro_norm_dps < gyro_static_th;

    %% =========================================
    % 5. ZUPT update
    %% =========================================

    zupt_due = t >= next_zupt_time-1e-9;
    if zupt_due
        % Advance past this epoch without producing repeated measurements
        % when an integration block spans more than one scheduling period.
        periods_elapsed = floor((t-next_zupt_time+1e-9)/zupt_period_s)+1;
        next_zupt_time = next_zupt_time+periods_elapsed*zupt_period_s;
    end

    if zupt_active && imu_quiet && zupt_due

        % Measurement:
        %
        % true velocity = 0
        %
        % residual = INS velocity - 0
        zk_zupt = ins.vn;

        kf.Hk = H_zupt;
        kf.Rk = R_zupt;

        kf = kfupdate(kf, zk_zupt, 'M');

        meas_updated = true;
    end

    %% =========================================
    % 6. Feedback
    %% =========================================

    if meas_updated

        % save pre-feedback KF state
        xk_before = kf.xk;
        Pk_before = diag(kf.Pxk);

        [kf, ins, xfb] = kffeedback(kf, ins, 1, 'avpedL');

        % feedback 后 attitude / velocity / lever 都可能变化
        % 所以重新计算一次杆臂端点
        ins = inslever(ins);

        avp(ki,:) = [ins.avp', t];

        avpL(ki,:) = [ ...
            ins.att', ...
            ins.vnL', ...
            ins.posL', ...
            t];

        %% Logs
        xkpk(ki,:) = [xk_before; Pk_before; t]';

        xfb_log(ki,:) = [xfb; t]';

        avp(ki,:) = [ins.avp', t];

        bias_log(ki,:) = [ ...
            ins.eb'/glv.dph, ...
            ins.db'/glv.ug, ...
            t ];

        ki = ki + 1;
    end

    %% =========================================
    % 7. ZUPT state log
    %% =========================================

    % 连续记录组合导航位置，不只记录有GNSS/ZUPT反馈的时刻。
    % 后处理按整数GPS秒插值为1 Hz，与现有avpL使用相同的杆臂端位置。
    ins = inslever(ins);
    nav_position_log(iz,:) = [ins.posL',t];

    zupt_log(iz,:) = [t, ...
        double(zupt_active && imu_quiet)];


    iz = iz + 1;
    timebar;
end
%% Trim
avp(ki:end,:) = [];
xkpk(ki:end,:) = [];
xfb_log(ki:end,:) = [];
bias_log(ki:end,:) = [];

zupt_log(iz:end,:) = [];
avpL(ki:end,:) = [];
nav_position_log(iz:end,:) = [];

%% Google Earth KML：实际时间轴上的严格1 Hz位置结果，不外推端点
[pos_kml_1hz,time_kml_1hz] = position_to_kml( ...
    nav_position_log(:,1:3),nav_position_log(:,4),kml_file, ...
    'AngleUnit','rad','SamplePeriod_s',1, ...
    'AltitudeMode',kml_altitude_mode,'Name','Combined navigation (1 Hz)');
fprintf('KML exported: %s (%d points, 1 Hz)\n',kml_file,numel(time_kml_1hz));

%% PSINS 标准图：insplot / kfplot
insplot(avp,'avp');
kfplot(xkpk);
%% IEEE single-column figures (8.8 cm x 7.05 cm)
if ~exist(fig_dir,'dir')
    mkdir(fig_dir);
end

fig_width_cm = 8.8;
fig_height_cm = 7.05;
font_size_pt = 9.5;      % 略大于常规期刊字号，便于 PPT 展示
line_width = 1.25;
color_ins = [0.00 0.35 0.70];
color_gnss = [0.15 0.15 0.15];
color_roll = [0.85 0.33 0.10];

t_ref = gnss1(:,2) - gnss1(1,2);
t_ins = avpL(:,10) - gnss1(1,2);

% GNSS and INS positions in local ENU coordinates [m].
pos_ref = gnss2(1,4:6);
gnss_enu = geodetic_to_local_enu(gnss2(:,4:6), pos_ref);
ins_enu = geodetic_to_local_enu(avpL(:,7:9), pos_ref);

%% Figure 1: horizontal position trajectory
fig_pos = figure('Color','w');
plot(gnss_enu(:,1),gnss_enu(:,2),'--','Color',color_gnss, ...
    'LineWidth',1.05,'DisplayName','GNSS');
hold on;
plot(ins_enu(:,1),ins_enu(:,2),'-','Color',color_ins, ...
    'LineWidth',line_width,'DisplayName','SINS/GNSS');
plot(gnss_enu(1,1),gnss_enu(1,2),'o','Color',[0 0 0], ...
    'MarkerFaceColor','w','MarkerSize',4.2,'HandleVisibility','off');
axis equal;
xlabel('East (m)');
ylabel('North (m)');
ylim([-1200 600])
legend('Location','southeast');
save_ieee_single_column(fig_pos,fig_dir,'fig_ieee_01_position_trajectory', ...
    fig_width_cm,fig_height_cm,font_size_pt,600);

%% Figure 2: east, north and up velocity components
fig_vel = figure('Color','w');
tl_vel = tiledlayout(fig_vel,3,1,'TileSpacing','compact','Padding','compact');
vel_gnss = [gnss1(:,7),gnss1(:,6),-gnss1(:,8)];
vel_ins = avpL(:,4:6);
vel_labels = {'v_E (m/s)','v_N (m/s)','v_U (m/s)'};
for a = 1:3
    ax = nexttile(tl_vel);
    plot(ax,t_ref,vel_gnss(:,a),'--','Color',color_gnss, ...
        'LineWidth',1.0,'DisplayName','GNSS');
    hold(ax,'on');
    plot(ax,t_ins,vel_ins(:,a),'-','Color',color_ins, ...
        'LineWidth',line_width,'DisplayName','SINS/GNSS');
    ylabel(ax,vel_labels{a});
    xlim(ax,[t_ref(1),t_ref(end)]);
    if a < 3
        ax.XTickLabel = [];
    else
        xlabel(ax,'Time (s)');
    end
    if a == 1
        legend(ax,'Location','best','NumColumns',2);
    end
end
save_ieee_single_column(fig_vel,fig_dir,'fig_ieee_02_velocity_components', ...
    fig_width_cm,fig_height_cm,font_size_pt,600);

%% Figure 3: horizontal attitude and heading
fig_att = figure('Color','w');
tl_att = tiledlayout(fig_att,2,1,'TileSpacing','compact','Padding','compact');
att_deg = avpL(:,1:3)/glv.deg;  % [pitch, roll, yaw]
att_deg(:,3) = unwrap(avpL(:,3))/glv.deg;

ax_att_h = nexttile(tl_att);
plot(ax_att_h,t_ins,att_deg(:,1),'-','Color',color_ins, ...
    'LineWidth',line_width,'DisplayName','Pitch');
hold(ax_att_h,'on');
plot(ax_att_h,t_ins,att_deg(:,2),'-','Color',color_roll, ...
    'LineWidth',line_width,'DisplayName','Roll');
ylabel(ax_att_h,'Angle (deg)');
xlim(ax_att_h,[t_ref(1),t_ref(end)]);
ax_att_h.XTickLabel = [];
legend(ax_att_h,'Location','best','NumColumns',2);

ax_att_yaw = nexttile(tl_att);
plot(ax_att_yaw,t_ins,att_deg(:,3),'-','Color',color_ins, ...
    'LineWidth',line_width);
xlabel(ax_att_yaw,'Time (s)');
ylabel(ax_att_yaw,'Heading (deg)');
xlim(ax_att_yaw,[t_ref(1),t_ref(end)]);

save_ieee_single_column(fig_att,fig_dir,'fig_ieee_03_attitude_components', ...
    fig_width_cm,fig_height_cm,font_size_pt,600);

%% Figure 4: bias
fig_att = figure('Color','w');
tl_bias = tiledlayout(fig_att,2,1,'TileSpacing','compact','Padding','compact');

ax_gyro = nexttile(tl_bias);
plot(ax_gyro,t_ins,bias_log(:,1),'-','Color',color_ins, ...
    'LineWidth',line_width,'DisplayName','X');
hold(ax_gyro,'on');
plot(ax_gyro,t_ins,bias_log(:,2),'-','Color',color_gnss, ...
    'LineWidth',line_width,'DisplayName','Y');
hold(ax_gyro,'on');
plot(ax_gyro,t_ins,bias_log(:,3),'-','Color',color_roll, ...
    'LineWidth',line_width,'DisplayName','Z');

ylabel(ax_gyro,'Gyro bias (deg/h)');
xlim(ax_gyro,[t_ref(1),t_ref(end)]);
ax_gyro.XTickLabel = [];
legend(ax_gyro,'Location','best','NumColumns',3);

ax_acce = nexttile(tl_bias);
plot(ax_acce,t_ins,bias_log(:,4),'-','Color',color_ins, ...
    'LineWidth',line_width,'DisplayName','X');
hold(ax_acce,'on');
plot(ax_acce,t_ins,bias_log(:,5),'-','Color',color_gnss, ...
    'LineWidth',line_width,'DisplayName','Y');
hold(ax_acce,'on');
plot(ax_acce,t_ins,bias_log(:,6),'-','Color',color_roll, ...
    'LineWidth',line_width,'DisplayName','Z');

ylabel(ax_acce,'Acce bias (ug)');
xlim(ax_acce,[t_ref(1),t_ref(end)]);

xlabel(ax_acce,'Time (s)');
xlim(ax_acce,[t_ref(1),t_ref(end)]);

save_ieee_single_column(fig_att,fig_dir,'fig_ieee_04_bias', ...
    fig_width_cm,fig_height_cm,font_size_pt,600);
%% =====================================================================
%%                             LOCAL FUNCTIONS
%% =====================================================================

function [ins, Cnb_mid] = insupdate_actual_time(ins,wvm,dt_pair)
% Integrate one/two recorded samples without assuming dt = 0.01 s.
% PSINS cnscl uses equal-duration subsamples. For a two-sample block, assume
% linear angular rate/specific force through the original interval midpoints
% and integrate it over two equal virtual half-intervals within this block.
% The columns of M sum to one, so total angle and delta-v are unchanged.
% Its determinant is T^2/(4*h1*h2), giving the unequal-interval coning/sculling
% coefficient (2/3)*det(M) = T^2/(6*h1*h2). No samples cross block boundaries.
    dt_pair = dt_pair(:);
    n_pair = size(wvm,1);
    assert(n_pair==numel(dt_pair) && any(n_pair==[1,2]), ...
        'Actual-time propagation requires one or two matching IMU intervals.');
    assert(all(isfinite(dt_pair)) && all(dt_pair>0), ...
        'Actual-time propagation requires positive finite IMU intervals.');
    qnb_before = ins.qnb;
    if n_pair==2
        h1 = dt_pair(1);
        h2 = dt_pair(2);
        assert(max(h1,h2)/min(h1,h2)<=4, ...
            'IMU interval ratio exceeds 4; check gaps/timing before linear two-sample propagation.');
        M = [(3*h1+h2)/(4*h1), (h2-h1)/(4*h2); ...
             (h1-h2)/(4*h1), (h1+3*h2)/(4*h2)];
        wvm = M*wvm;
    end
    ins.ts = sum(dt_pair)/n_pair;
    ins = insupdate(ins,wvm);
    % Quaternion midpoint (with matching signs) is a rotation matrix, unlike
    % an arithmetic mean of Cnb matrices. Use it for the white-noise mapping.
    if dot(qnb_before,ins.qnb)<0
        qnb_before = -qnb_before;
    end
    qnb_mid = qnb_before+ins.qnb;
    Cnb_mid = q2mat(qnb_mid/norm(qnb_mid));
end

function r = gnss_measurement_std(speed_rmse,horizontal_rmse,vertical_rmse,ins)
% Build the six-element GNSS measurement standard-deviation vector for
% [vE,vN,vU,lat,lon,h]. Receiver accuracies are treated as 1-sigma RMSE.
    acc = [speed_rmse,horizontal_rmse,vertical_rmse];
    assert(all(isfinite(acc)) && all(acc>0), ...
        'GNSS receiver accuracy values must be finite and positive.');
    r = [repmat(speed_rmse,3,1); ...
         horizontal_rmse/ins.eth.RMh; ...
         horizontal_rmse/ins.eth.clRNh; ...
         vertical_rmse];
end

function enu = geodetic_to_local_enu(pos,pos0)
% Convert [lat(rad), lon(rad), h(m)] to local [E,N,U] about pos0.
    a = 6378137.0;
    f = 1/298.257223563;
    e2 = f*(2-f);
    lat0 = pos0(1);
    h0 = pos0(3);
    den = sqrt(1-e2*sin(lat0)^2);
    RN = a/den;
    RM = a*(1-e2)/den^3;
    dlat = pos(:,1)-lat0;
    dlon = atan2(sin(pos(:,2)-pos0(2)),cos(pos(:,2)-pos0(2)));
    enu = [dlon*(RN+h0)*cos(lat0), dlat*(RM+h0), pos(:,3)-h0];
end

function fp = save_ieee_single_column(fh,fig_dir,base_name, ...
        width_cm,height_cm,font_size_pt,dpi)
% Exact IEEE single-column canvas with PPT-friendly font and line weights.
    set(fh,'Color','w','Renderer','painters', ...
        'Units','centimeters','Position',[2 2 width_cm height_cm], ...
        'PaperUnits','centimeters','PaperPosition',[0 0 width_cm height_cm], ...
        'PaperSize',[width_cm height_cm]);

    ax = findall(fh,'Type','axes');
    set(ax,'FontName','Times New Roman','FontSize',font_size_pt, ...
        'Box','on','TickDir','in','TickLength',[0.012 0.012], ...
        'LineWidth',0.8,'XGrid','off','YGrid','off','Color','w', ...
        'Layer','top');
    for i = 1:numel(ax)
        ax(i).LabelFontSizeMultiplier = 1.0;
        ax(i).TitleFontSizeMultiplier = 1.0;
    end

    lg = findall(fh,'Type','legend');
    if ~isempty(lg)
        set(lg,'Box','off','FontName','Times New Roman', ...
            'FontSize',max(font_size_pt-1,8));
    end

    fp = fullfile(fig_dir,[base_name '.png']);
    % print respects PaperSize/PaperPosition exactly; exportgraphics would
    % tightly crop the canvas and change the requested 8.8 cm x 7.05 cm size.
    print(fh,fp,'-dpng',sprintf('-r%d',dpi),'-painters');
end
