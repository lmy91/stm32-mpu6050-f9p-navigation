%% calib24_static_numbered_tempcomp.m
% 24 位置静态标定（先做温度补偿，再求标定系数）
%
% 与 calib24_static_numbered.m 的区别：
%   本脚本在统计各位置均值**之前**，先对每个样本做温度漂移补偿：
%
%       dT      = T - T0
%       drift(T)= c1*dT + ... + c_o*dT^o             (不补 0 阶 c0, 各轴阶数单独)
%       a_tc    = a_raw - drift(T)
%       g_tc    = g_raw - drift(T)
%
%   - 温补配置**完全由系数矩阵决定**（不再有 cfg.tcEnable）：各轴阶数取自
%     系数文件的 ord，未拟合轴整行为 0（polyval 得 0，等于该轴不做温补）。
%     例：axisOrder = [0 0 3 5 5 0] → 只补 az(3阶)/gx(5阶)/gy(5阶)，
%     ax/ay/gz 保持仅静态标定（面向 GNSS 1s 修正、失锁 <=600 s 工况裁定）。
%   - cfg.outTag 进入输出文件名；留空则按矩阵非零轴自动派生（例 'azgxgy'），
%     避免不同配置的结果互相覆盖。
%
%   - 温补系数在**原始域**拟合（tools/calibration/fit_temp_bias_raw.m），
%     与作用对象同域，避免 Ca-I 标度差异被带入
%   - 不补偿 c0：c0 是基准温度下的常值零偏（加速度计内还含重力投影），
%     属于本标定要求解的未知量，补偿掉会把 ba 与重力一起减掉
%   - 逐样本补偿后再求均值，可同时去掉段内温度漂移，减小位置均值方差
%
% 输出（不覆盖原 calib24_result.mat / calib24_summary.csv）：
%   calib24_result_tempcomp.mat
%   calib24_summary_tempcomp.csv
%   fig_tempcomp_gravity_error.png     温补前后重力模长误差对比
%   fig_tempcomp_temp_drift.png        各位置温度与补偿量
%
% 依赖：先运行 tools/calibration/fit_temp_bias_raw.m 生成
%       data/calib24/temp_coeffs_raw.mat
% ---------------------------------------------------------------
% Usage:
% 1) 24 个方位数据命名 01.csv ... 24.csv 放在 data/calib24/
% 2) 运行 tools/calibration/fit_temp_bias_raw.m 得到原始域温补系数矩阵
% 3) 运行本脚本
% ---------------------------------------------------------------
%%
clear;
clc;
close all;

%% ======================== USER CONFIG =============================

cfg.projRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cfg.dataDir  = fullfile(cfg.projRoot, 'data', 'calib24');

% 原始域温补系数文件（由 tools/calibration/fit_temp_bias_raw.m 生成；未拟合轴整行为 0）
cfg.tempCoeffFile = fullfile(cfg.dataDir, 'temp_coeffs_raw.mat');

% 温补配置完全由系数矩阵决定（只使用 c1..c_o，不含 c0）：
%   各轴阶数取自系数文件的 ord，未拟合轴整行为 0 → 该轴不补偿，无需单独开关。
cfg.outTag        = '';   % 留空 = 按矩阵非零轴自动派生文件名标签（例 'azgxgy'）
cfg.applyTempComp = true;
cfg.useZeroOrder  = false;     % 固定为 false：不补 0 阶

% Nominal gravity used as reference.
lat = 30.5284884000000;
lon = 114.355083100000;
h   = 35.2970000000000;
cfg.g = normal_gravity_wgs84(lat, h);

% Remove transients at the beginning/end of each static record.
cfg.trimStart = 10;            % s
cfg.trimEnd   = 10;            % s
cfg.minUsableDuration = 10;    % s

cfg.maxOrientationErrorDeg = 20;
cfg.maxRawNormError_mg     = 100;   % mg

% Output filenames 在加载系数矩阵之后再确定（cfg.outTag 可能由矩阵派生）。

fprintf('Latitude  = %.10f deg\n', lat);
fprintf('Longitude = %.10f deg\n', lon);
fprintf('Height    = %.3f m\n', h);
fprintf('Normal gravity = %.10f m/s^2\n', cfg.g);

%% ======================= COLUMN NAMES ==============================

col.time = 'time_s';
col.acc  = {'ax_m_s2','ay_m_s2','az_m_s2'};
col.gyr  = {'gx_deg_h','gy_deg_h','gz_deg_h'};
col.temp = 'temp_deg_c';

%% ==================== LOAD TEMP COEFFICIENTS ======================

if ~isfile(cfg.tempCoeffFile)
    error(['Temp coefficient file not found:\n%s\n' ...
           'Run tools/calibration/fit_temp_bias_raw.m first.'], cfg.tempCoeffFile);
end

S = load(cfg.tempCoeffFile);
assert(isfield(S,'domain') && strcmp(S.domain,'raw'), '只能使用原始域温补系数');
assert(all(isfinite(S.coef),'all') && size(S.coef,1)==6, '温补系数必须是有限的六轴矩阵');
assert(isfield(S,'Tmin') && isfield(S,'Tmax'), '请重新拟合并保存有效温区');
TC_all = S.coef;          % nAx x (maxOrder+1)，polyfit 约定 [c5 c4 c3 c2 c1 c0]
T0     = S.Tref;          % 基准温度

% 兼容旧格式（4 列 [c3 c2 c1 c0]）：左侧补 0，统一到 6 列 [c5 c4 c3 c2 c1 c0]
nCol = size(TC_all, 2);
if nCol < 6
    TC_all = [zeros(size(TC_all,1), 6-nCol), TC_all];
    warning('温度系数只有 %d 列，已左补 0 到 6 列布局。', nCol);
end
maxOrderTC = size(TC_all, 2) - 1;

% 各轴阶数：优先取系数文件里的 ord，旧文件则按非零最高阶推断
if isfield(S, 'ord') && numel(S.ord) == size(TC_all,1)
    ordTC = S.ord(:)';
else
    nAxTC = size(TC_all,1);
    ordTC = zeros(1, nAxTC);
    for a = 1:nAxTC
        nz = 0;                                  % 前导（高阶）零系数个数
        while nz < maxOrderTC && TC_all(a, nz+1) == 0
            nz = nz + 1;
        end
        ordTC(a) = maxOrderTC - nz;              % 非零最高阶
    end
    fprintf('系数文件无 ord 字段，已按非零最高阶推断各轴阶数。\n');
end

% 有效温补轴：ord > 0 的轴（未拟合轴整行为 0，polyval 得 0，等于不补偿）
tcActive = ordTC > 0;                          % 1x6 logical, [ax ay az gx gy gz]

% 输出文件标签：留空则按非零轴自动派生（例 [0 0 3 5 5 0] -> 'azgxgy'）
if isempty(cfg.outTag)
    axShort = {'ax','ay','az','gx','gy','gz'};
    cfg.outTag = strjoin(axShort(tcActive), '');
end

% Output filenames (do not overwrite the original calibration results)
if isempty(cfg.outTag)
    cfg.resultMat  = 'calib24_result_tempcomp.mat';
    cfg.summaryCsv = 'calib24_summary_tempcomp.csv';
    cfg.figGravity = 'fig_tempcomp_gravity_error.png';
    cfg.figTemp    = 'fig_tempcomp_temp_drift.png';
else
    cfg.resultMat  = sprintf('calib24_result_tempcomp_%s.mat', cfg.outTag);
    cfg.summaryCsv = sprintf('calib24_summary_tempcomp_%s.csv', cfg.outTag);
    cfg.figGravity = sprintf('fig_tempcomp_%s_gravity_error.png', cfg.outTag);
    cfg.figTemp    = sprintf('fig_tempcomp_%s_temp_drift.png', cfg.outTag);
end

if cfg.useZeroOrder
    warning('useZeroOrder = true 会连常值零偏一起补掉，通常不是期望行为。');
    TCpoly = TC_all;                                        % [c5 c4 c3 c2 c1 c0]
else
    % 注意: polyval 需要完整多项式系数，常数项必须显式给 0，
    %       否则 [c5..c1] 会被当成低 1 次多项式，丢掉一次幂（少补 dT 倍）。
    TCpoly = [TC_all(:, 1:end-1), zeros(size(TC_all,1),1)];  % [c5 c4 c3 c2 c1 0]
end

fprintf('\n温补系数文件: %s\n', cfg.tempCoeffFile);
fprintf('基准温度 T0 = %.4f degC\n', T0);
fprintf('各轴温度模型阶数: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d\n', ordTC);
fprintf('有效温补轴 (由矩阵 ord>0 派生) [ax ay az gx gy gz]: %d %d %d %d %d %d\n', +tcActive);
fprintf('温补多项式系数 (原始域, polyfit 约定 [c%d..c1 c0], 不含 0 阶):\n', maxOrderTC);
disp(TCpoly);
if cfg.applyTempComp
    fprintf('温度补偿: ON (逐样本补偿，不含 0 阶)\n');
else
    fprintf('温度补偿: OFF\n');
end

%% ========================= FILE LIST ===============================

files = arrayfun(@(k) sprintf('%02d.csv', k), (1:24)', 'UniformOutput', false);

positionName = {
    'X+ R0';'X+ R90';'X+ R180';'X+ R270';
    'X- R0';'X- R90';'X- R180';'X- R270';
    'Y+ R0';'Y+ R90';'Y+ R180';'Y+ R270';
    'Y- R0';'Y- R90';'Y- R180';'Y- R270';
    'Z+ R0';'Z+ R90';'Z+ R180';'Z+ R270';
    'Z- R0';'Z- R90';'Z- R180';'Z- R270'};

Npos = 24;
g    = cfg.g;

%% ================= REFERENCE SPECIFIC FORCE ========================

ref = zeros(Npos,3);
ref(1:4,:)   = repmat([ g  0  0],4,1);
ref(5:8,:)   = repmat([-g  0  0],4,1);
ref(9:12,:)  = repmat([ 0  g  0],4,1);
ref(13:16,:) = repmat([ 0 -g  0],4,1);
ref(17:20,:) = repmat([ 0  0  g],4,1);
ref(21:24,:) = repmat([ 0  0 -g],4,1);

%% ======================= PREALLOCATE ===============================

accMeanRaw = nan(Npos,3);     % 温补前位置均值
gyrMeanRaw = nan(Npos,3);
accMean    = nan(Npos,3);     % 温补后位置均值（用于求解）
gyrMean    = nan(Npos,3);
accStdRaw  = nan(Npos,3);
accStd     = nan(Npos,3);
gyrStdRaw  = nan(Npos,3);
gyrStd     = nan(Npos,3);

compAcc    = nan(Npos,3);     % 实际施加的加速度计补偿量（均值）
compGyr    = nan(Npos,3);     % 实际施加的陀螺补偿量（均值）

tempMean = nan(Npos,1);
tempStd  = nan(Npos,1);

Nsamples     = zeros(Npos,1);
duration_s   = nan(Npos,1);
sampleRate_Hz= nan(Npos,1);

orientationErrorDegRaw  = nan(Npos,1);
orientationErrorDeg     = nan(Npos,1);
rawNormError_mgRaw      = nan(Npos,1);
rawNormError_mg         = nan(Npos,1);
orientationOK           = false(Npos,1);

%% ======================= FILE CHECK ================================

missingFiles = {};
for k = 1:Npos
    if ~isfile(fullfile(cfg.dataDir, files{k}))
        missingFiles{end+1,1} = files{k}; %#ok<SAGROW>
    end
end
if ~isempty(missingFiles)
    fprintf('\nMissing calibration files:\n');
    fprintf('  %s\n', missingFiles{:});
    error('24 files are required in: %s', cfg.dataDir);
end

%% ======================== READ DATA ================================

fprintf('\n============================================================\n');
fprintf('   24-position calibration WITH temperature compensation\n');
fprintf('============================================================\n\n');
fprintf(['%2s  %-8s | %6s | %8s | %7s | %10s | %10s | %6s\n'], ...
        '#','pos','N','fs[Hz]','T[C]','|a|raw','|a|comp','dir.err');

for k = 1:Npos

    T = readtable(fullfile(cfg.dataDir, files{k}));

    requiredCols = [{col.time}, col.acc, col.gyr, {col.temp}];
    for c = 1:numel(requiredCols)
        if ~ismember(requiredCols{c}, T.Properties.VariableNames)
            error('Column "%s" not found in %s.', requiredCols{c}, files{k});
        end
    end

    time = T.(col.time);
    acc  = [T.(col.acc{1}), T.(col.acc{2}), T.(col.acc{3})];
    gyr  = [T.(col.gyr{1}), T.(col.gyr{2}), T.(col.gyr{3})];
    temp = T.(col.temp);

    valid = isfinite(time) & all(isfinite(acc),2) & ...
            all(isfinite(gyr),2) & isfinite(temp);

    time = time(valid);  acc = acc(valid,:);
    gyr  = gyr(valid,:); temp = temp(valid);

    if numel(time) < 2
        error('Not enough valid samples in %s.', files{k});
    end

    timeRel = time - time(1);
    duration_s(k) = timeRel(end);

    dt = diff(time);  dt = dt(isfinite(dt) & dt > 0);
    if isempty(dt)
        error('Cannot determine sample interval in %s.', files{k});
    end
    sampleRate_Hz(k) = 1 / mean(dt);

    % ---- Trim beginning/end ----
    idx = timeRel >= cfg.trimStart & timeRel <= (timeRel(end) - cfg.trimEnd);
    usableDuration = duration_s(k) - cfg.trimStart - cfg.trimEnd;
    if usableDuration < cfg.minUsableDuration
        error('%s is too short (%.2f s usable).', files{k}, usableDuration);
    end

    accUse  = acc(idx,:);
    gyrUse  = gyr(idx,:);
    tempUse = temp(idx);
    assert(all(tempUse >= S.Tmin & tempUse <= S.Tmax), ...
           '24 位置数据超出温补拟合温区，请检查或重新拟合');

    Nsamples(k) = size(accUse,1);

    % ================= 逐样本温度补偿（不补 0 阶, 按轴开关）=================
    dT = tempUse - T0;

    driftAcc = zeros(size(accUse));
    driftGyr = zeros(size(gyrUse));
    if cfg.applyTempComp
        for ax = 1:3
            if tcActive(ax)              % 加计 ax/ay/az
                driftAcc(:,ax) = polyval(TCpoly(ax,   :), dT);   % [c5..c1 0]
            end
            if tcActive(ax+3)            % 陀螺 gx/gy/gz
                driftGyr(:,ax) = polyval(TCpoly(ax+3, :), dT);
            end
        end
    end

    if cfg.applyTempComp
        accUseC = accUse - driftAcc;
        gyrUseC = gyrUse - driftGyr;
    else
        accUseC = accUse;
        gyrUseC = gyrUse;
    end

    % ---- 温补前统计 ----
    accMeanRaw(k,:) = mean(accUse,1);
    gyrMeanRaw(k,:) = mean(gyrUse,1);
    accStdRaw(k,:)  = std(accUse,0,1);
    gyrStdRaw(k,:)  = std(gyrUse,0,1);

    % ---- 温补后统计（用于求解）----
    accMean(k,:) = mean(accUseC,1);
    gyrMean(k,:) = mean(gyrUseC,1);
    accStd(k,:)  = std(accUseC,0,1);
    gyrStd(k,:)  = std(gyrUseC,0,1);

    % ---- 实际施加的补偿量（位置均值）----
    compAcc(k,:) = mean(driftAcc,1);
    compGyr(k,:) = mean(driftGyr,1);

    tempMean(k) = mean(tempUse);
    tempStd(k)  = std(tempUse);

    % ---- 方位自检（温补前后各算一次）----
    aRaw = accMeanRaw(k,:);  r = ref(k,:);
    c0r  = max(-1,min(1, dot(aRaw,r)/(norm(aRaw)*norm(r))));
    orientationErrorDegRaw(k) = acosd(c0r);
    rawNormError_mgRaw(k)     = (norm(aRaw)-g)/g*1000;

    a = accMean(k,:);
    c0 = max(-1,min(1, dot(a,r)/(norm(a)*norm(r))));
    orientationErrorDeg(k) = acosd(c0);
    rawNormError_mg(k)     = (norm(a)-g)/g*1000;

    orientationOK(k) = orientationErrorDeg(k) <= cfg.maxOrientationErrorDeg && ...
                       abs(rawNormError_mg(k)) <= cfg.maxRawNormError_mg;

    if orientationOK(k); statusTxt = 'OK'; else; statusTxt = 'CHECK'; end

    fprintf(['%2d  %-8s | %6d | %8.3f | %7.3f | %10.5f | %10.5f | %6.2f  %s\n'], ...
            k, positionName{k}, Nsamples(k), sampleRate_Hz(k), tempMean(k), ...
            norm(accMeanRaw(k,:)), norm(accMean(k,:)), orientationErrorDeg(k), statusTxt);
end

badIdx = find(~orientationOK);
if ~isempty(badIdx)
    fprintf('\nWARNING: %d file(s) failed orientation sanity check:\n', numel(badIdx));
    for ii = 1:numel(badIdx)
        k = badIdx(ii);
        fprintf('  %02d.csv (%s): dir.err %.2f deg, norm error %.2f mg\n', ...
                k, positionName{k}, orientationErrorDeg(k), rawNormError_mg(k));
    end
end

%% ============ 求解标定参数：温补前 / 温补后各一次 =================

[ba0, Ma0, Ca0, accCalMean0, stat0] = solve_calib(accMeanRaw, ref, g);
[ba1, Ma1, Ca1, accCalMean1, stat1] = solve_calib(accMean,    ref, g);

%% ====================== GYRO DIAGNOSTICS ===========================

gyroBiasMeanRaw_deg_h = mean(gyrMeanRaw,1)';
gyroBiasMean_deg_h    = mean(gyrMean,1)';       % 温补后（推荐使用）
gyroPositionStdRaw_deg_h = std(gyrMeanRaw,0,1)';
gyroPositionStd_deg_h    = std(gyrMean,0,1)';
gyroStaticNoiseRaw_deg_h = mean(gyrStdRaw,1)';
gyroStaticNoise_deg_h    = mean(gyrStd,1)';

gyroBiasMean_deg_s    = gyroBiasMean_deg_h / 3600;
gyroPositionStd_deg_s = gyroPositionStd_deg_h / 3600;
gyroStaticNoise_deg_s = gyroStaticNoise_deg_h / 3600;

%% ======================== TEMPERATURE ==============================

Tmin = min(tempMean);  Tmax = max(tempMean);  Tspan = Tmax - Tmin;

%% ======================== PRINT RESULTS ============================

fprintf('\n\n============================================================\n');
fprintf('   24-POSITION CALIBRATION RESULTS (TEMP COMPENSATED)\n');
fprintf('============================================================\n\n');

fprintf('--- 加速度计零偏 ba (温补后) [m/s^2] ---\n'); disp(ba1);
fprintf('--- 加速度计零偏 ba (温补前) [m/s^2] ---\n'); disp(ba0);

fprintf('--- 补偿矩阵 Ca = inv(Ma) (温补后) ---\n'); disp(Ca1);

fprintf('--- 标度因子误差 (温补后) [%%] ---\n');
disp((diag(Ma1) - 1)*100);
fprintf('--- 交叉轴项 (温补后) [%%] ---\n');
disp((Ma1 - diag(diag(Ma1)))*100);

fprintf('--- 重力模长 RMS ---\n');
fprintf('    标定前（原始均值）        : %.8f m/s^2 = %.4f mg\n', ...
        stat0.rmsGRaw, stat0.rmsGRaw/g*1000);
fprintf('    标定后（温补前标定参数）  : %.8f m/s^2 = %.4f mg\n', ...
        stat0.rmsGCal, stat0.rmsGCal/g*1000);
fprintf('    标定后（温补后标定参数）  : %.8f m/s^2 = %.4f mg\n', ...
        stat1.rmsGCal, stat1.rmsGCal/g*1000);
fprintf('    差值（温补收益）          : %+.4f mg\n\n', ...
        (stat1.rmsGCal - stat0.rmsGCal)/g*1000);

fprintf('--- 三轴残差 RMS [m/s^2] ---\n');
fprintf('    温补前: %.8f %.8f %.8f\n', stat0.rmsXYZ);
fprintf('    温补后: %.8f %.8f %.8f\n', stat1.rmsXYZ);
fprintf('    总体  : %.8f -> %.8f\n\n', stat0.rmsTotal, stat1.rmsTotal);

fprintf('--- 陀螺静态零偏 [deg/h] ---\n');
fprintf('    温补前: %12.3f %12.3f %12.3f\n', gyroBiasMeanRaw_deg_h);
fprintf('    温补后: %12.3f %12.3f %12.3f\n\n', gyroBiasMean_deg_h);

fprintf('--- 陀螺位置间离散 STD [deg/h] ---\n');
fprintf('    温补前: %12.3f %12.3f %12.3f\n', gyroPositionStdRaw_deg_h);
fprintf('    温补后: %12.3f %12.3f %12.3f\n\n', gyroPositionStd_deg_h);

fprintf('--- 温度 ---\n');
fprintf('    Tmin = %.3f C, Tmax = %.3f C, dT = %.3f C\n', Tmin, Tmax, Tspan);
fprintf('    实际施加温补量 (位置均值):\n');
fprintf('      加计 [mg]   : '); fprintf('%8.3f', compAcc/g*1000); fprintf('\n');
fprintf('      陀螺 [deg/h]: '); fprintf('%8.2f', compGyr); fprintf('\n\n');

fprintf('--- 质量判据（温补后）---\n');
rmsMg1 = stat1.rmsGCal/g*1000;
if rmsMg1 < 5
    fprintf('    ACC calibration quality: EXCELLENT (<5 mg)\n');
elseif rmsMg1 < 10
    fprintf('    ACC calibration quality: GOOD / ACCEPTABLE (5-10 mg)\n');
elseif rmsMg1 < 20
    fprintf('    ACC calibration quality: MARGINAL (10-20 mg)\n');
else
    fprintf('    ACC calibration quality: POOR (>20 mg)\n');
end
fprintf('    Gravity norm RMS = %.3f mg\n', rmsMg1);

%% ======================== SUMMARY TABLE ============================

Position    = (1:24)';
File        = string(files);
Orientation = string(positionName);

summaryTable = table( ...
    Position, File, Orientation, Nsamples, duration_s, sampleRate_Hz, ...
    tempMean, tempStd, ...
    orientationErrorDegRaw, orientationErrorDeg, orientationOK, ...
    compAcc(:,1)/g*1000, compAcc(:,2)/g*1000, compAcc(:,3)/g*1000, ...
    compGyr(:,1), compGyr(:,2), compGyr(:,3), ...
    accMeanRaw(:,1), accMeanRaw(:,2), accMeanRaw(:,3), ...
    accMean(:,1), accMean(:,2), accMean(:,3), ...
    accCalMean1(:,1), accCalMean1(:,2), accCalMean1(:,3), ...
    stat1.gNormCal, stat1.gErrorCal/g*1000, ...
    gyrMeanRaw(:,1), gyrMeanRaw(:,2), gyrMeanRaw(:,3), ...
    gyrMean(:,1), gyrMean(:,2), gyrMean(:,3), ...
    'VariableNames', { ...
    'Position','File','Orientation','SamplesUsed','FileDuration_s','SampleRate_Hz', ...
    'Temperature_C','TemperatureStd_C', ...
    'OrientationErrorRaw_deg','OrientationErrorTC_deg','OrientationOK', ...
    'CompAx_mg','CompAy_mg','CompAz_mg', ...
    'CompGx_deg_h','CompGy_deg_h','CompGz_deg_h', ...
    'AxRawMean_m_s2','AyRawMean_m_s2','AzRawMean_m_s2', ...
    'AxTC_m_s2','AyTC_m_s2','AzTC_m_s2', ...
    'AxCal_m_s2','AyCal_m_s2','AzCal_m_s2', ...
    'CalNorm_m_s2','CalNormError_mg', ...
    'GxRawMean_deg_h','GyRawMean_deg_h','GzRawMean_deg_h', ...
    'GxTC_deg_h','GyTC_deg_h','GzTC_deg_h'});

disp(summaryTable);

%% =========================== SAVE =================================

result = struct();
result.ba = ba1;  result.Ma = Ma1;  result.Ca = Ca1;         % 推荐使用（温补后）
result.ba_noTC = ba0; result.Ma_noTC = Ma0; result.Ca_noTC = Ca0;
result.biasMg = ba1 / g * 1000;
result.scaleErrorPercent = diag(Ma1-1)*100;
result.crossAxisPercent = (Ma1 - diag(diag(Ma1))) * 100;

result.accMeanRaw = accMeanRaw;   result.accMean = accMean;
result.accCalMean = accCalMean1;  result.accStd = accStd;
result.reference = ref;           result.residual = stat1.residual;

result.rmsXYZ = stat1.rmsXYZ;     result.rmsTotal = stat1.rmsTotal;
result.rmsXYZ_noTC = stat0.rmsXYZ; result.rmsTotal_noTC = stat0.rmsTotal;

result.gravityRMSRaw    = stat0.rmsGRaw;
result.gravityRMSTemperatureOnly = stat1.rmsGRaw;
result.gravityRMSCal    = stat1.rmsGCal;         % 温补后
result.gravityRMSCal_noTC = stat0.rmsGCal;       % 温补前
result.gravityMaxError  = stat1.maxGCal;

result.gyroBiasMean_deg_h = gyroBiasMean_deg_h;
result.gyroBiasMean_deg_s = gyroBiasMean_deg_s;
result.gyroBiasMeanRaw_deg_h  = gyroBiasMeanRaw_deg_h;
result.gyroPositionStd_deg_h    = gyroPositionStd_deg_h;
result.gyroPositionStdRaw_deg_h = gyroPositionStdRaw_deg_h;
result.gyroStaticNoise_deg_h    = gyroStaticNoise_deg_h;
result.gyroStaticNoiseRaw_deg_h = gyroStaticNoiseRaw_deg_h;

result.compAcc = compAcc;  result.compGyr = compGyr;
result.tempMean = tempMean; result.tempStd = tempStd;
result.tempMin = Tmin; result.tempMax = Tmax; result.tempSpan = Tspan;

result.tempCoeffFile = cfg.tempCoeffFile;
result.tempCoeff = TCpoly;      % nAx x 6 [c5..c1 0]，c0 为 0（不补 0 阶）
result.tempCoeffFull = TC_all;  % nAx x 6 [c5..c0]，原始域系数矩阵（未拟合轴整行为 0）
result.tempOrder = ordTC;       % 各轴阶数 [ax ay az gx gy gz]，0 = 未拟合
result.tcActive = tcActive;     % 有效温补轴（由 ord>0 派生）[ax ay az gx gy gz]
result.tcEnable = double(tcActive);  % 兼容旧字段名，语义与 tcActive 相同
result.outTag = cfg.outTag;
result.Tref = T0;
result.applyTempComp = cfg.applyTempComp;
result.useZeroOrder = cfg.useZeroOrder;
result.tempFitMin = S.Tmin; result.tempFitMax = S.Tmax;
result.tempDomain = S.domain;
result.tempSourceFile = S.sourceFile;
result.processingOrder = 'raw physical units -> temperature drift -> static calibration';

result.sampleRate_Hz = sampleRate_Hz;
result.orientationErrorDeg = orientationErrorDeg;
result.orientationErrorDegRaw = orientationErrorDegRaw;
result.orientationOK = orientationOK;
result.config = cfg;

save(fullfile(cfg.dataDir, cfg.resultMat), 'result');
writetable(summaryTable, fullfile(cfg.dataDir, cfg.summaryCsv));

%% =========================== FIGURES ===============================

% ---- 图1: 温补前后重力模长误差 ----
fig1 = figure('Name','Gravity norm error: with/without temp comp', ...
              'Position',[80 80 1000 520]);
plot(1:24, stat0.gNormRaw_gError, 'o-', 'LineWidth', 1.2, 'Color', [0.5 0.5 0.5]); hold on;
plot(1:24, stat0.gErrorCal/g*1000, 's-', 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]);
plot(1:24, stat1.gErrorCal/g*1000, '^-', 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);
yline(0,'k--');
yline( 5,'--', 'Color',[0.3 0.7 0.3]);
yline(-5,'--', 'Color',[0.3 0.7 0.3]);
xlabel('Position file number'); ylabel('Gravity norm error [mg]');
legend({'标定前 (raw)','标定后 (无温补)','标定后 (温补后)'}, 'Location','best');
grid on; title('24 位置重力模长误差：温补前后对比');
xlim([0.5 24.5]);
saveas(fig1, fullfile(cfg.dataDir, cfg.figGravity));

% ---- 图2: 各位置温度与补偿量 ----
fig2 = figure('Name','Temperature and applied compensation', ...
              'Position',[80 80 1100 800]);

subplot(3,1,1);
plot(1:24, tempMean, 'o-', 'LineWidth', 1.3, 'Color', [0.85 0.33 0.10]); hold on;
yline(T0, 'k--');
xlabel('Position file number'); ylabel('Temperature [degC]');
title(sprintf('各位置温度 (T0 = %.3f degC, 跨度 %.3f degC)', T0, Tspan));
grid on; xlim([0.5 24.5]);

subplot(3,1,2);
plot(1:24, compAcc(:,1)/g*1000, 'o-', 'LineWidth', 1.2); hold on;
plot(1:24, compAcc(:,2)/g*1000, 's-', 'LineWidth', 1.2);
plot(1:24, compAcc(:,3)/g*1000, '^-', 'LineWidth', 1.2);
xlabel('Position file number'); ylabel('Accel compensation [mg]');
legend({'X','Y','Z'}, 'Location','best');
grid on; title('加速度计实际施加温补量 (不含 0 阶)'); xlim([0.5 24.5]);

subplot(3,1,3);
plot(1:24, compGyr(:,1), 'o-', 'LineWidth', 1.2); hold on;
plot(1:24, compGyr(:,2), 's-', 'LineWidth', 1.2);
plot(1:24, compGyr(:,3), '^-', 'LineWidth', 1.2);
xlabel('Position file number'); ylabel('Gyro compensation [deg/h]');
legend({'Gx','Gy','Gz'}, 'Location','best');
grid on; title('陀螺实际施加温补量 (不含 0 阶)'); xlim([0.5 24.5]);

saveas(fig2, fullfile(cfg.dataDir, cfg.figTemp));

%% ============================= END ================================

fprintf('\nCalibration (temp compensated) completed.\n');
fprintf('Saved in: %s\n', cfg.dataDir);
fprintf('  %s\n  %s\n  %s\n  %s\n', cfg.resultMat, cfg.summaryCsv, ...
        cfg.figGravity, cfg.figTemp);
fprintf('\n使用:\n');
fprintf('    a_cal = Ca * (a_raw - drift(T) - ba)\n');
fprintf('    drift(T)= sum(c_m*dT^m, m=1..各轴阶数), dT = T - %.4f\n\n', T0);

%% ======================= LOCAL FUNCTIONS ==========================

function [ba, Ma, Ca, accCalMean, stat] = solve_calib(accMean, ref, g)
%SOLVE_CALIB  24 位置加速度计确定性标定求解
Npos = size(accMean,1);
Xp = mean(accMean(1:4,:),1)';   Xm = mean(accMean(5:8,:),1)';
Yp = mean(accMean(9:12,:),1)';  Ym = mean(accMean(13:16,:),1)';
Zp = mean(accMean(17:20,:),1)'; Zm = mean(accMean(21:24,:),1)';

bX = (Xp + Xm)/2;  bY = (Yp + Ym)/2;  bZ = (Zp + Zm)/2;
ba = (bX + bY + bZ)/3;

Mx = (Xp - Xm)/(2*g);  My = (Yp - Ym)/(2*g);  Mz = (Zp - Zm)/(2*g);
Ma = [Mx My Mz];
if cond(Ma) > 100
    warning('Ma is poorly conditioned: cond = %.3f', cond(Ma));
end
    Ca = Ma \ eye(3);

accCalMean = zeros(Npos,3);
for k = 1:Npos
    accCalMean(k,:) = (Ca * (accMean(k,:)' - ba))';
end

stat.residual = accCalMean - ref;
stat.rmsXYZ = sqrt(mean(stat.residual.^2,1));
stat.rmsTotal = sqrt(mean(stat.residual(:).^2));

stat.gNormRaw = vecnorm(accMean,2,2);
stat.rmsGRaw = sqrt(mean((stat.gNormRaw - g).^2));

stat.gNormCal = vecnorm(accCalMean,2,2);
stat.gErrorCal = stat.gNormCal - g;      % m/s^2
stat.gNormRaw_gError = (stat.gNormRaw - g)/g*1000;
stat.rmsGCal = sqrt(mean(stat.gErrorCal.^2));
stat.maxGCal = max(abs(stat.gErrorCal));
end

function gamma_h = normal_gravity_wgs84(lat_deg, h)
%NORMAL_GRAVITY_WGS84  WGS-84 normal gravity at latitude and height.
a  = 6378137.0;
f  = 1 / 298.257223563;
b  = a * (1 - f);
GM = 3.986004418e14;
omega = 7.292115e-5;
e2 = f * (2 - f);
gamma_e = 9.7803253359;
k = 0.00193185265241;
phi = deg2rad(lat_deg);
sin2phi = sin(phi)^2;
gamma_0 = gamma_e * (1 + k*sin2phi) / sqrt(1 - e2*sin2phi);
m = omega^2 * a^2 * b / GM;
gamma_h = gamma_0 * (1 - 2/a*(1 + f + m - 2*f*sin2phi)*h + 3*h^2/a^2);
end
