clc
clear
close all

% test_bias_random_walk.m
% 在 test.m 的 18 状态 SINS/GNSS 滤波基础上，将陀螺仪和加速度计零偏
% 由一阶 Gauss-Markov 过程改为随机游走，并用 Allan RRW 系数设置零偏过程噪声。

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));

%% =========================================================
% 补偿开关与系数文件
%
% 处理链（与 tools/calib24_static_numbered_tempcomp.m 完全一致）：
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
allanParamFile = fullfile(repo_root, 'data', 'allan_results', ...
    'mpu_21.1h_20260930', 'allan_parameters.csv');

% 单独输出，避免覆盖 test.m 生成的图。
fig_dir = fullfile(script_dir, 'figs_bias_random_walk');

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
               'Run tools/fit_temp_bias_raw.m first.'], tempCoeffFile);
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
               'Run tools/calib24_static_numbered_tempcomp.m first.'], ...
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

gyro_static = mean(imu_static1(:,1:3));
imu2(:,1:3) = imu2(:,1:3)-gyro_static;

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
[nn, ts, nts] = nnts(2, 0.01);

%% Bias random-walk model from Allan RRW
% 0930 文件已经是需要的混合轴配置，所有 Allan 参数均从该文件读取。
assert(isfile(allanParamFile), '0930 Allan parameter file not found: %s', allanParamFile);
A = readtable(allanParamFile, 'TextType','string');
requiredAllan = {'sensor','axis','arw_vrw','bias_instability','rate_random_walk'};
assert(all(ismember(requiredAllan,A.Properties.VariableNames)), ...
    '0930 Allan parameter CSV is missing required columns.');

axisName = ["x","y","z"];
arw_g_sensor = zeros(3,1);
bi_g_sensor = zeros(3,1);
rrw_g_sensor_deg_h_sqrth = zeros(3,1);
vrw_a_sensor = zeros(3,1);
bi_a_sensor = zeros(3,1);
rrw_a_sensor_mg_sqrth = zeros(3,1);
for a = 1:3
    ig = strcmpi(A.sensor,'gyro') & strcmpi(A.axis,axisName(a));
    ia = strcmpi(A.sensor,'accel') & strcmpi(A.axis,axisName(a));
    assert(nnz(ig)==1 && nnz(ia)==1, ...
        '0930 Allan file must have exactly one row per sensor axis.');
    arw_g_sensor(a) = A.arw_vrw(ig);
    bi_g_sensor(a) = A.bias_instability(ig);
    rrw_g_sensor_deg_h_sqrth(a) = A.rate_random_walk(ig);
    vrw_a_sensor(a) = A.arw_vrw(ia);
    bi_a_sensor(a) = A.bias_instability(ia);
    rrw_a_sensor_mg_sqrth(a) = A.rate_random_walk(ia);
end

% 上面的 IMU 增量已由原始传感器坐标转为右-前-上（RFU）：
%   gyro_rfu = [-gy, gx, gz], accel_rfu = [-ay, ax, az]
% 随机游走的符号不改变方差，但 x/y 轴必须交换。
rrw_g_rfu_deg_h_sqrth = rrw_g_sensor_deg_h_sqrth([2 1 3]);
rrw_a_rfu_mg_sqrth    = rrw_a_sensor_mg_sqrth([2 1 3]);

% 测量白噪声使用 0930 Allan 结果。eb/db 是独立的初始协方差配置，
% 不直接用 BI 替代。eb=20 deg/h 对应减静态均值后的地球自转量级；
% db=5000 ug 暂作工程初值，后续用加速度计零偏重复性标定结果替换。
web = arw_g_sensor([2 1 3]);                              % deg/sqrt(h)
wdb = vrw_a_sensor([2 1 3]) / (60*glv.ug);               % ug/sqrt(Hz)
eb  = [20;20;20];                                        % deg/h
db  = [5000;5000;5000];                                  % ug

% imuerrset 在 Tau=inf 时启用零偏随机游走：
%   db/dt = w_b, Phi_bias = I, Q_bias = diag(K.^2)*dt
% 其陀螺 RRW 入参单位为 deg/h/sqrt(h)，加表 RRW 入参单位为 ug/sqrt(h)。
sqrtR0G = rrw_g_rfu_deg_h_sqrth;
sqrtR0A = 1000 * rrw_a_rfu_mg_sqrth;  % mg/sqrt(h) -> ug/sqrt(h)
TauG    = inf(3,1);
TauA    = inf(3,1);

imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA);

fprintf('Bias model: random walk (TauG = TauA = inf)\n');
fprintf('0930 Allan parameter file: %s\n', allanParamFile);
fprintf('Gyro ARW RFU [x y z] = %.6f %.6f %.6f deg/sqrt(h)\n', web);
fprintf('Accel VRW RFU [x y z] = %.3f %.3f %.3f ug/sqrt(Hz)\n', wdb);
fprintf('Gyro RRW RFU [x y z] = %.3f %.3f %.3f deg/h/sqrt(h)\n', sqrtR0G);
fprintf('Accel RRW RFU [x y z] = %.2f %.2f %.2f ug/sqrt(h)\n', sqrtR0A);
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

r0 = vperrset(0.2, 3.0);
kf = kfinit(ins, davp0, imuerr, lever_std, r0);
% Save normal GNSS measurement covariance
R_gnss = kf.Rk;

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

% KF runs at 50 Hz (nn=2, ts=0.01)
% apply ZUPT at 10 Hz
zupt_interval = 10;
zupt_counter = 0;

zupt_log = zeros(fix(len/nn),2);  % [time, active]
iz = 1;


%%
imu = imu2; % 右前上 增量
gps = gnss2;
imugpssyn(imu(:,7), gps(:,7));


len = length(imu);

xfb_log = prealloc(fix(len/nn), kf.n+1);
[avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);

bias_log = zeros(fix(len/nn),7);
zupt_log = zeros(fix(len/nn),2);
avpL = zeros(fix(len/nn),10);
timebar(nn, len, '18-state SINS/GPS.');

ki = 1;
iz = 1;

static_count = 0;
zupt_active = false;
zupt_counter = 0;

for k = 1:nn:len-nn+1

    k1 = k + nn - 1;

    wvm = imu(k:k1,1:6);
    t   = imu(k1,end);

    %% =========================================
    % 1. SINS propagation
    %% =========================================
    ins = insupdate(ins, wvm);

    %% =========================================
    % 2. KF prediction
    %% =========================================
    kf.Phikk_1 = kffk(ins);
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

        % Restore normal GNSS measurement model
        kf.Hk = kfhk(ins);
        kf.Rk = R_gnss;

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
    % sum over nn samples / nts => rad/s
    omega_b = sum(wvm(:,1:3),1)' / nts;

    gyro_norm_dps = norm(omega_b) / glv.deg;

    imu_quiet = gyro_norm_dps < gyro_static_th;

    %% =========================================
    % 5. ZUPT update
    %% =========================================

    zupt_counter = zupt_counter + 1;

    if zupt_active && imu_quiet && ...
            mod(zupt_counter, zupt_interval) == 0

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


%% PSINS 标准图：insplot / kfplot（自动保存新产生的 figure）
figsPrev = findobj(0,'Type','figure');

insplot(avp,'avp');
kfplot(xkpk);

if ~isempty(fig_dir) && ~exist(fig_dir,'dir')
    mkdir(fig_dir);
end

figsNow = findobj(0,'Type','figure');
figsNew = figsNow(~ismember(figsNow,figsPrev));
for i = 1:numel(figsNew)
    save_fig(figsNew(i), fig_dir, sprintf('fig_dyn_00_psins_%02d', i));
end
%%
% GNSS
fig1 = figure;

subplot(3,1,1)
plot(gnss1(:,2), gnss1(:,7)); hold on
plot(avpL(:,end), avpL(:,4));
grid off
ylabel('V_E / m/s')
legend('GNSS antenna','INS antenna')

subplot(3,1,2)
plot(gnss1(:,2), gnss1(:,6)); hold on
plot(avpL(:,end), avpL(:,5));
grid off
ylabel('V_N / m/s')

subplot(3,1,3)
plot(gnss1(:,2), -gnss1(:,8)); hold on
plot(avpL(:,end), avpL(:,6));
grid off
ylabel('V_U / m/s')
xlabel('Time / s')

save_fig(fig1, fig_dir, 'fig_dyn_01_gnss_vs_ins_velocity');

fig2 = figure;

subplot(2,1,1)
plot(bias_log(:,end), bias_log(:,1:3))
grid off
ylabel('Gyro bias / deg/h')
legend('x','y','z')

subplot(2,1,2)
plot(bias_log(:,end), bias_log(:,4:6))
grid off
ylabel('Acc bias / ug')
xlabel('Time / s')
legend('x','y','z')

save_fig(fig2, fig_dir, 'fig_dyn_02_kf_bias');

fig3 = figure;

subplot(4,1,1)
plot(avp(:,end),avp(:,4));
ylabel('V_E')
grid off

subplot(4,1,2)
plot(avp(:,end),avp(:,1)/glv.deg);
ylabel('Pitch / deg')
grid off

subplot(4,1,3)
plot(bias_log(:,end),bias_log(:,4));
ylabel('b_{ax} / ug')
grid off

subplot(4,1,4)
plot(gnss1(:,2),hypot(gnss1(:,6),gnss1(:,7)));
ylabel('GNSS speed')
xlabel('Time / s')
grid off

save_fig(fig3, fig_dir, 'fig_dyn_03_velocity_attitude_bias');

%%
fig4 = figure;

subplot(2,1,1)
plot(gnss1(:,2), ...
    hypot(gnss1(:,6),gnss1(:,7)));
grid off
ylabel('GNSS speed / m/s');

subplot(2,1,2)
stairs(zupt_log(:,1),zupt_log(:,2));
grid off
ylim([-0.1 1.1]);
ylabel('ZUPT');
xlabel('GPS TOW / s');

save_fig(fig4, fig_dir, 'fig_dyn_04_gnss_speed_zupt');

%%
fig5 = figure;

plot(gnss1(:,2),gnss1(:,7),'LineWidth',1);
hold on
plot(avp(:,end),avp(:,4),'LineWidth',1);

yyaxis right
stairs(zupt_log(:,1),zupt_log(:,2),'--');

grid off

yyaxis left
ylabel('V_E / m/s');

yyaxis right
ylabel('ZUPT');

xlabel('Time / s');

legend('GNSS','INS/GNSS','ZUPT');

save_fig(fig5, fig_dir, 'fig_dyn_05_ve_zupt');


%% =====================================================================
%%                             LOCAL FUNCTIONS
%% =====================================================================

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
    try
        exportgraphics(fh, fp, 'Resolution', dpi);
    catch
        print(fh, fp, '-dpng', sprintf('-r%d', dpi));
    end
end


function apply_ieee_style(fh, widthIn, heightIn, fontSize)
% IEEE 期刊风格统一设置：
%   白底 / Times New Roman / 刻度朝内 / 四边全框 / 无网格 / 细轴线 / 图例无边框
    set(fh, 'Color','w', ...
        'Units','inches', ...
        'Position',[1 1 widthIn*1.4 heightIn*1.4], ...   % 屏幕显示放大，便于查看
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
