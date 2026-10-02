%% compare_bias_models.m
% 对比一阶 Gauss-Markov (GM) 与随机游走 (RW) 两种 IMU 零偏模型。
%
% 公平性约束：
%   1) 两个模型使用完全相同的 IMU 温补、24 位标定、初始状态、GNSS 和 ZUPT。
%   2) 唯一变量是零偏状态过程：GM 用最新 Allan BI+拟合区间，
%      RW 用同一份最新 Allan RRW。
%   3) 默认在三段车辆持续运动区间停止 GNSS 更新，但保留 GNSS 作为
%      独立参考，避免用“已经参与滤波的 GNSS”评价滤波器。
%
% 输出：
%   bias_model_comparison_summary.csv
%   bias_model_comparison.mat
%   figs_bias_model_comparison/*.png

clc;
clear;
close all;

global glv
glvs;
psinstypedef(186);

% 代码位于 code/experiments，数据和结果位于会话根目录。
code_dir = fileparts(mfilename('fullpath'));
script_dir = fileparts(fileparts(code_dir));
repo_root  = fileparts(fileparts(fileparts(script_dir)));

%% Configuration
cfg.gnss_file = fullfile(script_dir, 'gnss.csv');
cfg.imu_file  = fullfile(script_dir, 'imu.csv');
cfg.temp_coeff_file = fullfile(repo_root, 'data', 'calib24', ...
    'temp_coeffs_raw.mat');
cfg.calib_file = fullfile(repo_root, 'data', 'calib24', ...
    'calib24_result_tempcomp_azgxgy.mat');
cfg.allan_file = fullfile(repo_root, 'data', 'allan_results', ...
    'mpu_21.1h_20260930', 'allan_parameters.csv');

cfg.n_start_gnss = 200;
cfg.n_end_gnss = 800;
cfg.n_static_begin_gnss = 20;
cfg.n_static_end_gnss = 200;

cfg.nn = 2;
cfg.ts = 0.01;
cfg.lever = [-0.14; 0.16; 0.50];
cfg.lever_std = [0.02; 0.02; 0.05];
cfg.r0 = vperrset(0.2, 3.0);
cfg.sigma_zupt = 0.05;
cfg.v_enter = 0.15;
cfg.v_exit = 0.35;
cfg.gyro_static_th_dps = 1.5;
cfg.zupt_interval = 10;

% 相对于导航起点的 GNSS 留出区间 [s]。这三段车速持续大于 1 m/s。
cfg.holdout_windows_s = [120 180; 300 360; 420 480];

cfg.fig_dir = fullfile(script_dir, 'figs_bias_model_comparison');
cfg.result_dir = fullfile(script_dir, 'results', 'bias_model_comparison');
cfg.out_csv = fullfile(cfg.result_dir, 'bias_model_comparison_summary.csv');
cfg.out_mat = fullfile(cfg.result_dir, 'bias_model_comparison.mat');

if ~exist(cfg.result_dir, 'dir')
    mkdir(cfg.result_dir);
end

if ~exist(cfg.fig_dir, 'dir')
    mkdir(cfg.fig_dir);
end

%% Common input and initialization
fprintf('\n============================================================\n');
fprintf(' Bias-model comparison: first-order GM vs random walk\n');
fprintf(' GNSS holdout windows:');
fprintf(' [%g,%g] s', cfg.holdout_windows_s');
fprintf('\n============================================================\n');

data = prepare_common_data(cfg);
allan = load_latest_allan_parameters(cfg.allan_file);

% eb/db 是独立的初始协方差配置，不是 Allan 过程参数；两模型保持相同。
% ARW/VRW、BI、RRW 和 GM 相关时间则全部来自 0930 Allan 文件。
% eb=20 deg/h：陀螺已减启机静态均值，剩余不确定度按地球自转量级设置。
% db=5000 ug：当前工程初值，后续用加速度计零偏重复性标定结果替换。
common.eb  = [20;20;20];
common.db  = [5000;5000;5000];
common.web = allan.gyro_arw_rfu_deg_sqrth;
common.wdb = allan.acc_vrw_rfu_mps_sqrth / (60*glv.ug); % -> ug/sqrt(Hz)

models(1).key = 'GM';
models(1).name = 'First-order GM';
models(1).sqrtR0G = allan.gyro_bi_rfu_deg_h;            % deg/h
models(1).tauG = allan.gyro_tau_rfu_s;                  % s
models(1).sqrtR0A = 1000*allan.acc_bi_rfu_mg;           % ug
models(1).tauA = allan.acc_tau_rfu_s;                   % s

models(2).key = 'RW';
models(2).name = 'Random walk';
models(2).sqrtR0G = allan.gyro_rrw_rfu_deg_h_sqrth;     % deg/h/sqrt(h)
models(2).tauG = inf(3,1);
models(2).sqrtR0A = 1000*allan.acc_rrw_rfu_mg_sqrth;    % ug/sqrt(h)
models(2).tauA = inf(3,1);

fprintf('\n0930 Allan parameter file     : %s\n', cfg.allan_file);
fprintf('ARW RFU [deg/sqrt(h)]       : %.6f %.6f %.6f\n', common.web);
fprintf('VRW RFU [ug/sqrt(Hz)]       : %.3f %.3f %.3f\n', common.wdb);
fprintf('GM gyro BI RFU [deg/h]      : %.6f %.6f %.6f\n', models(1).sqrtR0G);
fprintf('GM gyro tau RFU [s]         : %.3f %.3f %.3f\n', models(1).tauG);
fprintf('GM accel BI RFU [ug]        : %.6f %.6f %.6f\n', models(1).sqrtR0A);
fprintf('GM accel tau RFU [s]        : %.3f %.3f %.3f\n', models(1).tauA);
fprintf('RW gyro RRW RFU [deg/h/sqrt(h)]: %.6f %.6f %.6f\n', models(2).sqrtR0G);
fprintf('RW accel RRW RFU [ug/sqrt(h)]  : %.6f %.6f %.6f\n', models(2).sqrtR0A);

%% Run both models
result_cells = cell(numel(models),1);
for i = 1:numel(models)
    fprintf('\n--- Running %s model ---\n', models(i).name);
    result_cells{i} = run_filter_case(data, cfg, common, models(i));
end
results = vertcat(result_cells{:});

%% Metrics and recommendation
summary = build_summary_table(results);
writetable(summary, cfg.out_csv);

nis_target = 6;  % 6-D GNSS velocity+position measurement
nis_score = abs(log(summary.MeanNIS / nis_target));

[~, win_vel] = min(summary.VelRMSE3D_mps);
[~, win_pos] = min(summary.PosRMSEHorizontal_m);
[~, win_nis] = min(nis_score);
votes = accumarray([win_vel; win_pos; win_nis], 1, [height(summary), 1]);
[best_votes, best_idx] = max(votes);

if best_votes >= 2
    recommendation = sprintf(['Recommended for this data and holdout setup: %s ' ...
        '(%d/3 primary metrics).'], summary.Model(best_idx), best_votes);
else
    recommendation = 'No clear winner; inspect the per-axis errors and repeat with other holdout windows.';
end

fprintf('\n====================== SUMMARY ======================\n');
disp(summary);
fprintf('%s\n', recommendation);
fprintf('CSV : %s\n', cfg.out_csv);
fprintf('MAT : %s\n', cfg.out_mat);
fprintf('Figs: %s\n', cfg.fig_dir);

save(cfg.out_mat, 'cfg', 'models', 'results', 'summary', 'recommendation');
make_comparison_figures(results, summary, cfg);

%% =====================================================================
%% Local functions
%% =====================================================================

function allan = load_latest_allan_parameters(allan_file)
% 0930 文件是最终按轴温补数据的 Allan 结果：GX/GY/AZ 已温补，
% GZ/AX/AY 未温补。ARW/VRW、BI、RRW 和 BI 拟合区间全部从此读取。
    assert(isfile(allan_file), '0930 Allan parameter file not found: %s', allan_file);
    T = readtable(allan_file, 'TextType','string');
    required = {'sensor','axis','arw_vrw','bias_instability', ...
        'bias_tau_start_s','bias_tau_end_s','rate_random_walk'};
    assert(all(ismember(required,T.Properties.VariableNames)), ...
        '0930 Allan parameter CSV is missing required columns.');

    axes_sensor = ["x","y","z"];
    gyro_arw = zeros(3,1);
    gyro_bi = zeros(3,1);
    gyro_tau = zeros(3,1);
    gyro_rrw = zeros(3,1);
    acc_vrw = zeros(3,1);
    acc_bi = zeros(3,1);
    acc_tau = zeros(3,1);
    acc_rrw = zeros(3,1);

    for a = 1:3
        tg = strcmpi(T.sensor,'gyro') & strcmpi(T.axis,axes_sensor(a));
        ta = strcmpi(T.sensor,'accel') & strcmpi(T.axis,axes_sensor(a));
        assert(nnz(tg)==1 && nnz(ta)==1, ...
            '0930 Allan file must have exactly one row per sensor axis.');

        gyro_arw(a) = T.arw_vrw(tg);
        gyro_bi(a) = T.bias_instability(tg);
        gyro_rrw(a) = T.rate_random_walk(tg);
        gyro_tau(a) = sqrt(T.bias_tau_start_s(tg)*T.bias_tau_end_s(tg));
        acc_vrw(a) = T.arw_vrw(ta);
        acc_bi(a) = T.bias_instability(ta);
        acc_rrw(a) = T.rate_random_walk(ta);
        acc_tau(a) = sqrt(T.bias_tau_start_s(ta)*T.bias_tau_end_s(ta));
    end

    assert(all(isfinite([gyro_arw;gyro_bi;gyro_tau;gyro_rrw; ...
        acc_vrw;acc_bi;acc_tau;acc_rrw])), ...
        'Selected latest Allan parameters contain NaN or Inf.');

    rfu = [2 1 3];
    allan.gyro_arw_rfu_deg_sqrth = gyro_arw(rfu);
    allan.gyro_bi_rfu_deg_h = gyro_bi(rfu);
    allan.gyro_tau_rfu_s = gyro_tau(rfu);
    allan.gyro_rrw_rfu_deg_h_sqrth = gyro_rrw(rfu);
    allan.acc_vrw_rfu_mps_sqrth = acc_vrw(rfu);
    allan.acc_bi_rfu_mg = acc_bi(rfu);
    allan.acc_tau_rfu_s = acc_tau(rfu);
    allan.acc_rrw_rfu_mg_sqrth = acc_rrw(rfu);
end

function data = prepare_common_data(cfg)
global glv

    assert(isfile(cfg.gnss_file), 'GNSS file not found: %s', cfg.gnss_file);
    assert(isfile(cfg.imu_file), 'IMU file not found: %s', cfg.imu_file);
    assert(isfile(cfg.temp_coeff_file), 'Temperature coefficient file not found: %s', ...
        cfg.temp_coeff_file);
    assert(isfile(cfg.calib_file), 'Accelerometer calibration file not found: %s', ...
        cfg.calib_file);

    gnss0 = readtable(cfg.gnss_file);
    ok = gnss0.fix == 3 & gnss0.gnss_fix_ok == 1 & ...
         gnss0.num_sv >= 6 & gnss0.h_acc_m <= 20.0 & gnss0.pdop <= 6.0;
    g = gnss0(ok,:);
    gnss_valid = [g.gps_week, g.gps_tow_ms/1000, g.lat_deg, g.lon_deg, ...
        g.hmsl_m, g.vel_n_m_s, g.vel_e_m_s, g.vel_d_m_s, ...
        g.v_acc_m, g.h_acc_m, g.s_acc_m_s, g.pdop];

    assert(cfg.n_end_gnss <= size(gnss_valid,1), ...
        'Requested GNSS end index exceeds valid data length.');

    imu0 = readtable(cfg.imu_file);
    dt = imu0.dt_s;
    imu_valid = [imu0.gps_week, imu0.gps_tow_us/1e6, ...
        imu0.gx_deg_h.*dt, imu0.gy_deg_h.*dt, imu0.gz_deg_h.*dt, ...
        imu0.ax_m_s2.*dt, imu0.ay_m_s2.*dt, imu0.az_m_s2.*dt, ...
        imu0.temp_deg_c, dt];

    % Temperature compensation: active axes are az, gx and gy.
    TC = load(cfg.temp_coeff_file);
    assert(isfield(TC,'coef') && size(TC.coef,1) == 6, ...
        'temp_coeffs_raw.mat must contain a six-row coef matrix.');
    TCpoly = [TC.coef(:,1:end-1), zeros(6,1)];
    if isfield(TC, 'ord')
        ord = TC.ord(:)';
    else
        ord = TC.axisOrder(:)';
    end
    tc_active = ord > 0;
    dT = imu_valid(:,9) - TC.Tref;
    dt_vec = imu_valid(:,10);

    if isfield(TC,'Tmin') && isfield(TC,'Tmax')
        assert(~any(imu_valid(:,9) < TC.Tmin-1e-9 | ...
                    imu_valid(:,9) > TC.Tmax+1e-9), ...
            'IMU temperature is outside the fitted compensation range.');
    end

    drift_acc = zeros(height(imu0),3);
    drift_gyr = zeros(height(imu0),3);
    for a = 1:3
        if tc_active(a)
            drift_acc(:,a) = polyval(TCpoly(a,:), dT);
        end
        if tc_active(a+3)
            drift_gyr(:,a) = polyval(TCpoly(a+3,:), dT);
        end
    end
    imu_valid(:,3:5) = imu_valid(:,3:5) - drift_gyr .* dt_vec;
    imu_valid(:,6:8) = imu_valid(:,6:8) - drift_acc .* dt_vec;

    % 24-position deterministic accelerometer calibration.
    C = load(cfg.calib_file);
    assert(isfield(C,'result') && isfield(C.result,'ba') && isfield(C.result,'Ca'), ...
        'Calibration MAT must contain result.ba and result.Ca.');
    ba_cal = C.result.ba(:);
    Ca_cal = C.result.Ca;
    dv_tc = imu_valid(:,6:8);
    imu_valid(:,6:8) = (Ca_cal * (dv_tc' - ba_cal * dt_vec'))';

    t_static_start = gnss_valid(cfg.n_static_begin_gnss,2);
    t_static_end   = gnss_valid(cfg.n_static_end_gnss,2);
    t_start = gnss_valid(cfg.n_start_gnss,2);
    t_end   = gnss_valid(cfg.n_end_gnss,2);

    imu1 = imu_valid(imu_valid(:,2) >= t_start & imu_valid(:,2) <= t_end,:);
    imu_static = imu_valid(imu_valid(:,2) >= t_static_start & ...
        imu_valid(:,2) <= t_static_end,:);
    gnss1 = gnss_valid(gnss_valid(:,2) >= t_start & gnss_valid(:,2) <= t_end,:);

    assert(~isempty(imu1) && ~isempty(imu_static) && ~isempty(gnss1), ...
        'Selected IMU/GNSS interval is empty.');

    gps = [gnss1(:,7), gnss1(:,6), -gnss1(:,8), ...
           gnss1(:,3)*pi/180, gnss1(:,4)*pi/180, gnss1(:,5), gnss1(:,2)];

    % Sensor frame -> right-forward-up (RFU).
    imu = [[-imu1(:,4), imu1(:,3), imu1(:,5)]*pi/180/3600, ...
           [-imu1(:,7), imu1(:,6), imu1(:,8)], imu1(:,2)];
    imu_static_rfu = [[-imu_static(:,4), imu_static(:,3), imu_static(:,5)] ...
        *pi/180/3600, [-imu_static(:,7), imu_static(:,6), imu_static(:,8)], ...
        imu_static(:,2)];

    gyro_static = mean(imu_static_rfu(:,1:3),1);
    imu(:,1:3) = imu(:,1:3) - gyro_static;

    [~, idx0] = min(abs(gnss_valid(:,2) - imu1(1,2)));
    pos0 = [gnss_valid(idx0,3)*pi/180; ...
            gnss_valid(idx0,4)*pi/180; gnss_valid(idx0,5)];
    vel0 = [gnss_valid(idx0,7); gnss_valid(idx0,6); -gnss_valid(idx0,8)];
    yaw0 = 0;
    [att0, ~] = alignsb(imu_static_rfu, pos0, yaw0);

    [nn, ts, nts] = nnts(cfg.nn, cfg.ts);
    data.nn = nn;
    data.ts = ts;
    data.nts = nts;
    data.imu = imu;
    data.gps = gps;
    data.gnss_ref = gnss1;
    data.avp0 = [att0; vel0; pos0];
    data.t_start = t_start;
    data.t_end = t_end;
    data.tc_active = tc_active;
    data.gyro_static = gyro_static;

    fprintf('Valid evaluation duration: %.1f s, GNSS epochs: %d\n', ...
        t_end-t_start, size(gnss1,1));
    fprintf('Temperature-compensated axes [ax ay az gx gy gz]: %d %d %d %d %d %d\n', ...
        +tc_active);
    fprintf('Removed static gyro mean in RFU [deg/h]: %.3f %.3f %.3f\n', ...
        gyro_static/(pi/180/3600));
    fprintf('Gravity check from static RFU data: %.6f m/s^2\n', ...
        norm(mean(imu_static_rfu(:,4:6)./imu_static(:,10),1)));

    %#ok<NASGU> glv is initialized for PSINS routines.
end

function result = run_filter_case(data, cfg, common, model)
global glv

    imuerr = imuerrset(common.eb, common.db, common.web, common.wdb, ...
        model.sqrtR0G, model.tauG, model.sqrtR0A, model.tauA);

    davp0 = avperrset([120;120;600], [0.5;0.5;0.5], [5;5;10]);
    ins = insinit(data.avp0, data.ts);
    ins.tauG = imuerr.taug;
    ins.tauA = imuerr.taua;
    ins.lever = cfg.lever;
    ins = inslever(ins);

    kf = kfinit(ins, davp0, imuerr, cfg.lever_std, cfg.r0);
    R_gnss = kf.Rk;
    H_zupt = [zeros(3,3), eye(3), zeros(3,12)];
    R_zupt = eye(3) * cfg.sigma_zupt^2;

    imu = data.imu;
    gps = data.gps;
    imugpssyn(imu(:,7), gps(:,7));

    n_steps = floor(size(imu,1)/data.nn);
    nav_log = zeros(n_steps,10);
    bias_log = zeros(n_steps,7);
    bias_std_log = zeros(n_steps,7);
    % [gps_time, relative_time, innovation(6), NIS, holdout, gps_index, diag(S)(6)]
    innov_log = zeros(size(gps,1),17);

    static_count = 0;
    zupt_active = false;
    zupt_counter = 0;
    ki = 1;
    ni = 1;

    for k = 1:data.nn:size(imu,1)-data.nn+1
        k1 = k + data.nn - 1;
        wvm = imu(k:k1,1:6);
        t = imu(k1,7);

        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        meas_updated = false;

        [kgps, dt_sync] = imugpssyn(k, k1, 'F');
        if kgps > 0
            vn_gps = gps(kgps,1:3)';
            pos_gps = gps(kgps,4:6)';
            ins = inslever(ins);
            H_gnss = kfhk(ins);
            zk = [ins.vnL - ins.an*dt_sync - vn_gps; ...
                  ins.posL - ins.Mpvvn*dt_sync - pos_gps];
            S = H_gnss*kf.Pxk*H_gnss' + R_gnss;
            nis = real(zk' * (S \ zk));

            rel_t = gps(kgps,7) - data.t_start;
            is_holdout = in_windows(rel_t, cfg.holdout_windows_s);
            innov_log(ni,:) = [gps(kgps,7), rel_t, zk', nis, ...
                double(is_holdout), kgps, diag(S(1:6,1:6))'];
            ni = ni + 1;

            if ~is_holdout
                kf.Hk = H_gnss;
                kf.Rk = R_gnss;
                kf = kfupdate(kf, zk, 'M');
                meas_updated = true;

                v_horizontal = hypot(vn_gps(1), vn_gps(2));
                if v_horizontal < cfg.v_enter
                    static_count = static_count + 1;
                    if static_count >= 2
                        zupt_active = true;
                    end
                elseif v_horizontal > cfg.v_exit
                    static_count = 0;
                    zupt_active = false;
                end
            else
                % Holdout GNSS is reference only; it must not drive ZUPT either.
                static_count = 0;
                zupt_active = false;
            end
        end

        omega_b = sum(wvm(:,1:3),1)' / data.nts;
        imu_quiet = norm(omega_b)/glv.deg < cfg.gyro_static_th_dps;
        zupt_counter = zupt_counter + 1;
        if zupt_active && imu_quiet && mod(zupt_counter,cfg.zupt_interval) == 0
            kf.Hk = H_zupt;
            kf.Rk = R_zupt;
            kf = kfupdate(kf, ins.vn, 'M');
            meas_updated = true;
        end

        if meas_updated
            [kf, ins] = kffeedback(kf, ins, 1, 'avpedL');
        end
        ins = inslever(ins);

        nav_log(ki,:) = [ins.att', ins.vnL', ins.posL', t];
        bias_log(ki,:) = [ins.eb'/glv.dph, ins.db'/glv.ug, t];
        p_bias = max(diag(kf.Pxk(10:15,10:15)), 0);
        bias_std_log(ki,:) = [sqrt(p_bias(1:3))'/glv.dph, ...
            sqrt(p_bias(4:6))'/glv.ug, t];
        ki = ki + 1;
    end

    nav_log(ki:end,:) = [];
    bias_log(ki:end,:) = [];
    bias_std_log(ki:end,:) = [];
    innov_log(ni:end,:) = [];

    result.key = model.key;
    result.name = model.name;
    result.model = model;
    result.nav_log = nav_log;
    result.bias_log = bias_log;
    result.bias_std_log = bias_std_log;
    result.innov_log = innov_log;
    result.metrics = evaluate_case(result, data, cfg);

    fprintf(['Holdout: velocity 3D RMSE %.4f m/s, horizontal position RMSE %.3f m, ' ...
        'mean NIS %.3f\n'], result.metrics.vel_rmse_3d, ...
        result.metrics.pos_rmse_horizontal, result.metrics.mean_nis);
end

function metrics = evaluate_case(result, data, cfg)
    ref = data.gnss_ref;
    tref = ref(:,2);
    trel = tref - data.t_start;
    holdout = false(size(tref));
    for i = 1:size(cfg.holdout_windows_s,1)
        holdout = holdout | (trel >= cfg.holdout_windows_s(i,1) & ...
            trel <= cfg.holdout_windows_s(i,2));
    end

    [tnav, ia] = unique(result.nav_log(:,10), 'stable');
    nav = result.nav_log(ia,:);
    est = interp1(tnav, nav(:,1:9), tref, 'linear', NaN);
    valid = holdout & all(isfinite(est),2);

    v_ref = [ref(:,7), ref(:,6), -ref(:,8)];
    vel_err = est(:,4:6) - v_ref;
    pos_ref = [ref(:,3)*pi/180, ref(:,4)*pi/180, ref(:,5)];
    pos_err = geodetic_error_enu(est(:,7:9), pos_ref);

    ve = vel_err(valid,:);
    pe = pos_err(valid,:);
    assert(size(ve,1) >= 10, 'Too few valid GNSS samples in holdout windows.');

    vel_norm = sqrt(sum(ve.^2,2));
    pos_horizontal = hypot(pe(:,1),pe(:,2));
    pos_norm = sqrt(sum(pe.^2,2));

    hold_innov = result.innov_log(:,10) > 0.5;
    nis = result.innov_log(hold_innov,9);

    metrics.valid_mask = valid;
    metrics.time = tref;
    metrics.time_rel = trel;
    metrics.vel_error_all = vel_err;
    metrics.pos_error_all = pos_err;
    metrics.n_holdout = size(ve,1);
    metrics.vel_rmse_axis = sqrt(mean(ve.^2,1));
    metrics.vel_rmse_3d = sqrt(mean(vel_norm.^2));
    metrics.vel_p95_3d = percentile_local(vel_norm,95);
    metrics.pos_rmse_axis = sqrt(mean(pe.^2,1));
    metrics.pos_rmse_horizontal = sqrt(mean(pos_horizontal.^2));
    metrics.pos_rmse_3d = sqrt(mean(pos_norm.^2));
    metrics.pos_p95_horizontal = percentile_local(pos_horizontal,95);
    metrics.mean_nis = mean(nis);
    metrics.nis_upper95_coverage = mean(nis <= 12.5916);
end

function summary = build_summary_table(results)
    n = numel(results);
    Model = strings(n,1);
    HoldoutSamples = zeros(n,1);
    VelRMSE_E_mps = zeros(n,1);
    VelRMSE_N_mps = zeros(n,1);
    VelRMSE_U_mps = zeros(n,1);
    VelRMSE3D_mps = zeros(n,1);
    VelP95_3D_mps = zeros(n,1);
    PosRMSE_E_m = zeros(n,1);
    PosRMSE_N_m = zeros(n,1);
    PosRMSE_U_m = zeros(n,1);
    PosRMSEHorizontal_m = zeros(n,1);
    PosRMSE3D_m = zeros(n,1);
    PosP95_Horizontal_m = zeros(n,1);
    MeanNIS = zeros(n,1);
    NISUpper95Coverage = zeros(n,1);

    for i = 1:n
        m = results(i).metrics;
        Model(i) = string(results(i).name);
        HoldoutSamples(i) = m.n_holdout;
        VelRMSE_E_mps(i) = m.vel_rmse_axis(1);
        VelRMSE_N_mps(i) = m.vel_rmse_axis(2);
        VelRMSE_U_mps(i) = m.vel_rmse_axis(3);
        VelRMSE3D_mps(i) = m.vel_rmse_3d;
        VelP95_3D_mps(i) = m.vel_p95_3d;
        PosRMSE_E_m(i) = m.pos_rmse_axis(1);
        PosRMSE_N_m(i) = m.pos_rmse_axis(2);
        PosRMSE_U_m(i) = m.pos_rmse_axis(3);
        PosRMSEHorizontal_m(i) = m.pos_rmse_horizontal;
        PosRMSE3D_m(i) = m.pos_rmse_3d;
        PosP95_Horizontal_m(i) = m.pos_p95_horizontal;
        MeanNIS(i) = m.mean_nis;
        NISUpper95Coverage(i) = m.nis_upper95_coverage;
    end

    summary = table(Model,HoldoutSamples,VelRMSE_E_mps,VelRMSE_N_mps, ...
        VelRMSE_U_mps,VelRMSE3D_mps,VelP95_3D_mps,PosRMSE_E_m, ...
        PosRMSE_N_m,PosRMSE_U_m,PosRMSEHorizontal_m,PosRMSE3D_m, ...
        PosP95_Horizontal_m,MeanNIS,NISUpper95Coverage);
end

function make_comparison_figures(results, summary, cfg)
    colors = [0.00 0.45 0.74; 0.85 0.33 0.10];

    f1 = figure('Color','w','Position',[100 100 1000 650]);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    ax1 = nexttile; hold(ax1,'on');
    ax2 = nexttile; hold(ax2,'on');
    for i = 1:numel(results)
        m = results(i).metrics;
        tv = m.time_rel;
        v3 = sqrt(sum(m.vel_error_all.^2,2));
        ph = hypot(m.pos_error_all(:,1),m.pos_error_all(:,2));
        plot(ax1,tv,v3,'Color',colors(i,:),'DisplayName',results(i).name);
        plot(ax2,tv,ph,'Color',colors(i,:),'DisplayName',results(i).name);
    end
    shade_windows(ax1,cfg.holdout_windows_s);
    shade_windows(ax2,cfg.holdout_windows_s);
    ylabel(ax1,'Velocity error 3D / m s^{-1}');
    ylabel(ax2,'Horizontal position error / m');
    xlabel(ax2,'Time from navigation start / s');
    legend(ax1,'Location','best');
    grid(ax1,'on'); grid(ax2,'on');
    title(ax1,'GNSS holdout comparison (shaded intervals are scored)');
    exportgraphics(f1,fullfile(cfg.fig_dir,'fig_01_holdout_errors.png'),'Resolution',300);

    f2 = figure('Color','w','Position',[120 120 1050 500]);
    tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
    nexttile;
    bar(categorical(summary.Model),summary.VelRMSE3D_mps);
    ylabel('3D velocity RMSE / m s^{-1}'); grid on;
    nexttile;
    bar(categorical(summary.Model),summary.PosRMSEHorizontal_m);
    ylabel('Horizontal position RMSE / m'); grid on;
    nexttile;
    bar(categorical(summary.Model),summary.MeanNIS);
    hold on; yline(6,'--k','6-D ideal mean');
    ylabel('Mean NIS'); grid on;
    exportgraphics(f2,fullfile(cfg.fig_dir,'fig_02_metric_bars.png'),'Resolution',300);

    f3 = figure('Color','w','Position',[140 80 1100 850]);
    tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
    labels = {'b_{gx} / deg h^{-1}','b_{gy} / deg h^{-1}', ...
              'b_{gz} / deg h^{-1}','b_{ax} / ug','b_{ay} / ug','b_{az} / ug'};
    for a = 1:6
        ax = nexttile; hold(ax,'on');
        for i = 1:numel(results)
            t = results(i).bias_log(:,7) - results(i).bias_log(1,7);
            plot(ax,t,results(i).bias_log(:,a),'Color',colors(i,:), ...
                'DisplayName',results(i).name);
        end
        ylabel(ax,labels{a}); grid(ax,'on');
        if a >= 5, xlabel(ax,'Filter time / s'); end
        if a == 1, legend(ax,'Location','best'); end
    end
    exportgraphics(f3,fullfile(cfg.fig_dir,'fig_03_bias_estimates.png'),'Resolution',300);
end

function tf = in_windows(t, windows)
    tf = false;
    for i = 1:size(windows,1)
        tf = tf || (t >= windows(i,1) && t <= windows(i,2));
    end
end

function enu = geodetic_error_enu(est_pos, ref_pos)
    % Inputs: [lat(rad), lon(rad), h(m)]. Output: [E,N,U] in metres.
    a = 6378137.0;
    f = 1/298.257223563;
    e2 = f*(2-f);
    lat = ref_pos(:,1);
    h = ref_pos(:,3);
    s = sin(lat);
    den = sqrt(1-e2*s.^2);
    RN = a./den;
    RM = a*(1-e2)./den.^3;
    dlat = est_pos(:,1)-ref_pos(:,1);
    dlon = atan2(sin(est_pos(:,2)-ref_pos(:,2)), ...
                  cos(est_pos(:,2)-ref_pos(:,2)));
    enu = [dlon.*(RN+h).*cos(lat), dlat.*(RM+h), est_pos(:,3)-h];
end

function q = percentile_local(x,p)
    x = sort(x(isfinite(x)));
    assert(~isempty(x), 'Cannot compute percentile of empty data.');
    if numel(x) == 1
        q = x;
        return;
    end
    r = 1 + (numel(x)-1)*p/100;
    lo = floor(r);
    hi = ceil(r);
    q = x(lo) + (r-lo)*(x(hi)-x(lo));
end

function shade_windows(ax,windows)
    yl = ylim(ax);
    for i = 1:size(windows,1)
        patch(ax,[windows(i,1) windows(i,2) windows(i,2) windows(i,1)], ...
            [yl(1) yl(1) yl(2) yl(2)],[0.85 0.85 0.85], ...
            'FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
    end
    ylim(ax,yl);
end
