%% calib24_static_numbered.m
% 24-position static calibration for low-cost MEMS IMU
%
% Expected input files:
%   calib24/01.csv
%   calib24/02.csv
%   ...
%   calib24/24.csv
%
% Your logger CSV columns:
%   sample, gps_week, gps_tow_us, time_valid, timer_us, time_s, dt_s,
%   ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw,
%   ax_m_s2, ay_m_s2, az_m_s2, temp_deg_c,
%   gx_deg_h, gy_deg_h, gz_deg_h
%
% Calibration model:
%   a_m = Ma * a_true + ba + noise
%   a_cal = Ca * (a_m - ba),  Ca = inv(Ma)
%
% Position definition:
%   01-04 : X+   (0/90/180/270 deg about gravity axis)
%   05-08 : X-
%   09-12 : Y+
%   13-16 : Y-
%   17-20 : Z+
%   21-24 : Z-
%
% IMPORTANT:
%   The gyro columns are deg/h in your logger. This script keeps gyro
%   diagnostics in deg/h and also reports deg/s for convenience.
%
%   Static 24-position data are used here for accelerometer deterministic
%   calibration and gyro static-bias diagnostics. They do NOT provide a
%   reliable gyro scale-factor/non-orthogonality calibration without known
%   angular-rate excitation.
%
% ---------------------------------------------------------------
% Usage:
% 1) Create a folder named "calib24" in the current MATLAB directory.
% 2) After each orientation test, rename imu.csv to 01.csv ... 24.csv.
% 3) Put all 24 files into calib24/.
% 4) Run this script.
% ---------------------------------------------------------------
%%
imu = readtable('calib24\04.csv');
%%
clear;
clc;
close all;

%% ======================== USER CONFIG =============================

cfg.dataDir = fullfile(pwd, 'calib24');

% Nominal gravity used as reference.
% If you later want higher accuracy, replace this with local gravity.
lat = 30.5284884000000;
lon = 114.355083100000;    % 正常重力公式本身不需要 lon
h   = 35.2970000000000;

cfg.g = normal_gravity_wgs84(lat, h);

fprintf('Latitude  = %.10f deg\n', lat);
fprintf('Longitude = %.10f deg\n', lon);
fprintf('Height    = %.3f m\n', h);
fprintf('Normal gravity = %.10f m/s^2\n', cfg.g);


% Remove transients at the beginning/end of each static record.
cfg.trimStart = 10;                    % s
cfg.trimEnd   = 10;                    % s

% Minimum usable data after trimming.
cfg.minUsableDuration = 10;            % s

% Orientation sanity-check threshold.
% If mean acceleration differs from the expected direction by >20 deg,
% the script warns that the numbered file may correspond to the wrong face.
cfg.maxOrientationErrorDeg = 20;

% Gravity-norm sanity-check threshold before calibration.
cfg.maxRawNormError_mg = 100;          % mg

% Output filenames
cfg.resultMat = 'calib24_result.mat';
cfg.summaryCsv = 'calib24_summary.csv';

%% ======================= COLUMN NAMES ==============================

col.time = 'time_s';

col.acc = {'ax_m_s2','ay_m_s2','az_m_s2'};
col.gyr = {'gx_deg_h','gy_deg_h','gz_deg_h'};
col.temp = 'temp_deg_c';

%% ========================= FILE LIST ===============================

files = arrayfun(@(k) sprintf('%02d.csv', k), ...
                 (1:24)', 'UniformOutput', false);

positionName = {
    'X+ R0'
    'X+ R90'
    'X+ R180'
    'X+ R270'
    'X- R0'
    'X- R90'
    'X- R180'
    'X- R270'
    'Y+ R0'
    'Y+ R90'
    'Y+ R180'
    'Y+ R270'
    'Y- R0'
    'Y- R90'
    'Y- R180'
    'Y- R270'
    'Z+ R0'
    'Z+ R90'
    'Z+ R180'
    'Z+ R270'
    'Z- R0'
    'Z- R90'
    'Z- R180'
    'Z- R270'
    };

Npos = 24;
g = cfg.g;

%% ================= REFERENCE SPECIFIC FORCE ========================

ref = zeros(Npos,3);

ref(1:4,:)   = repmat([ g  0  0],4,1);   % X+
ref(5:8,:)   = repmat([-g  0  0],4,1);   % X-
ref(9:12,:)  = repmat([ 0  g  0],4,1);   % Y+
ref(13:16,:) = repmat([ 0 -g  0],4,1);   % Y-
ref(17:20,:) = repmat([ 0  0  g],4,1);   % Z+
ref(21:24,:) = repmat([ 0  0 -g],4,1);   % Z-

%% ======================= PREALLOCATE ===============================

accMean = nan(Npos,3);
accStd  = nan(Npos,3);

gyrMean_deg_h = nan(Npos,3);
gyrStd_deg_h  = nan(Npos,3);

tempMean = nan(Npos,1);
tempStd  = nan(Npos,1);

Nsamples = zeros(Npos,1);
duration_s = nan(Npos,1);
meanDt_s = nan(Npos,1);
sampleRate_Hz = nan(Npos,1);

orientationErrorDeg = nan(Npos,1);
rawNormError_mg = nan(Npos,1);
orientationOK = false(Npos,1);

%% ======================= FILE CHECK ================================

missingFiles = {};

for k = 1:Npos
    filename = fullfile(cfg.dataDir, files{k});
    if ~isfile(filename)
        missingFiles{end+1,1} = files{k}; %#ok<SAGROW>
    end
end

if ~isempty(missingFiles)
    fprintf('\nMissing calibration files:\n');
    fprintf('  %s\n', missingFiles{:});
    error(['24 files are required. Put 01.csv ... 24.csv in:\n' ...
           '%s'], cfg.dataDir);
end

%% ======================== READ DATA ================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('       Reading numbered 24-position calibration files\n');
fprintf('============================================================\n\n');

for k = 1:Npos

    filename = fullfile(cfg.dataDir, files{k});
    T = readtable(filename);

    % ---- Check required columns ----
    requiredCols = [{col.time}, col.acc, col.gyr, {col.temp}];

    for c = 1:numel(requiredCols)
        if ~ismember(requiredCols{c}, T.Properties.VariableNames)
            error('Column "%s" not found in %s.', ...
                  requiredCols{c}, files{k});
        end
    end

    % ---- Extract logger outputs ----
    time = T.(col.time);
    acc = [ ...
        T.(col.acc{1}), ...
        T.(col.acc{2}), ...
        T.(col.acc{3}) ];

    gyr_deg_h = [ ...
        T.(col.gyr{1}), ...
        T.(col.gyr{2}), ...
        T.(col.gyr{3}) ];

    temp = T.(col.temp);

    % ---- Valid rows ----
    valid = isfinite(time) & ...
            all(isfinite(acc),2) & ...
            all(isfinite(gyr_deg_h),2) & ...
            isfinite(temp);

    time = time(valid);
    acc = acc(valid,:);
    gyr_deg_h = gyr_deg_h(valid,:);
    temp = temp(valid);

    if numel(time) < 2
        error('Not enough valid samples in %s.', files{k});
    end

    % The logger time_s can continue from device boot, so make it relative
    % to the beginning of THIS file before trimming.
    timeRel = time - time(1);

    duration_s(k) = timeRel(end);

    dt = diff(time);
    dt = dt(isfinite(dt) & dt > 0);

    if isempty(dt)
        error('Cannot determine sample interval in %s.', files{k});
    end

    meanDt_s(k) = mean(dt);
    sampleRate_Hz(k) = 1 / meanDt_s(k);

    % ---- Trim beginning/end ----
    idx = timeRel >= cfg.trimStart & ...
          timeRel <= (timeRel(end) - cfg.trimEnd);

    usableDuration = duration_s(k) - cfg.trimStart - cfg.trimEnd;

    if usableDuration < cfg.minUsableDuration
        error(['%s is too short. Duration = %.2f s; after trimming only ' ...
               '%.2f s remain.'], ...
               files{k}, duration_s(k), usableDuration);
    end

    accUse = acc(idx,:);
    gyrUse_deg_h = gyr_deg_h(idx,:);
    tempUse = temp(idx);

    Nsamples(k) = size(accUse,1);

    % ---- Statistics ----
    accMean(k,:) = mean(accUse,1);
    accStd(k,:)  = std(accUse,0,1);

    gyrMean_deg_h(k,:) = mean(gyrUse_deg_h,1);
    gyrStd_deg_h(k,:)  = std(gyrUse_deg_h,0,1);

    tempMean(k) = mean(tempUse);
    tempStd(k)  = std(tempUse);

    % ---- Orientation sanity check ----
    a = accMean(k,:);
    r = ref(k,:);

    cosTheta = dot(a,r) / (norm(a)*norm(r));
    cosTheta = max(-1,min(1,cosTheta));
    orientationErrorDeg(k) = acosd(cosTheta);

    rawNormError_mg(k) = (norm(a)-g)/g*1000;

    orientationOK(k) = ...
        orientationErrorDeg(k) <= cfg.maxOrientationErrorDeg && ...
        abs(rawNormError_mg(k)) <= cfg.maxRawNormError_mg;

    if orientationOK(k)
        statusTxt = 'OK';
    else
        statusTxt = 'CHECK';
    end

    fprintf(['%02d  %-8s | N=%6d | fs=%8.3f Hz | T=%6.2f C | ' ...
             '|a|=%9.5f | dir.err=%6.2f deg | %s\n'], ...
             k, positionName{k}, Nsamples(k), sampleRate_Hz(k), ...
             tempMean(k), norm(accMean(k,:)), ...
             orientationErrorDeg(k), statusTxt);
end

%% ==================== PRE-CALIBRATION CHECK ========================

badIdx = find(~orientationOK);

if ~isempty(badIdx)
    fprintf('\n');
    fprintf('WARNING: Some files failed the orientation sanity check:\n');
    for ii = 1:numel(badIdx)
        k = badIdx(ii);
        fprintf('  %02d.csv (%s): direction error %.2f deg, norm error %.2f mg\n', ...
                k, positionName{k}, ...
                orientationErrorDeg(k), rawNormError_mg(k));
    end
    fprintf(['Please verify that these numbered files correspond to the ' ...
             'intended faces before trusting the calibration.\n\n']);
end

%% ===================== BASIC GROUP MEANS ===========================

Xp = mean(accMean(1:4,:),1)';
Xm = mean(accMean(5:8,:),1)';

Yp = mean(accMean(9:12,:),1)';
Ym = mean(accMean(13:16,:),1)';

Zp = mean(accMean(17:20,:),1)';
Zm = mean(accMean(21:24,:),1)';

%% ================== ACCELEROMETER BIAS =============================

% Each +/- pair gives an independent estimate of the same bias vector.
bX = (Xp + Xm) / 2;
bY = (Yp + Ym) / 2;
bZ = (Zp + Zm) / 2;

% Average the three pair-derived estimates.
ba = (bX + bY + bZ) / 3;

%% ================== MEASUREMENT MATRIX Ma ==========================

% Columns correspond to true X/Y/Z specific-force inputs.
Mx = (Xp - Xm) / (2*g);
My = (Yp - Ym) / (2*g);
Mz = (Zp - Zm) / (2*g);

Ma = [Mx My Mz];

condMa = cond(Ma);

if condMa > 100
    warning('Ma is poorly conditioned: cond(Ma) = %.3f', condMa);
end

%% ================= CORRECTION MATRIX Ca ============================

Ca = inv(Ma);

%% ===================== APPLY CALIBRATION ===========================

accCalMean = zeros(size(accMean));

for k = 1:Npos
    accCalMean(k,:) = (Ca * (accMean(k,:)' - ba))';
end

%% ======================== RESIDUALS ================================

residual = accCalMean - ref;

rmsXYZ = sqrt(mean(residual.^2,1));
rmsTotal = sqrt(mean(residual(:).^2));

gNormRaw = vecnorm(accMean,2,2);
gErrorRaw = gNormRaw - g;

gNormCal = vecnorm(accCalMean,2,2);
gErrorCal = gNormCal - g;

rmsGRaw = sqrt(mean(gErrorRaw.^2));
rmsGCal = sqrt(mean(gErrorCal.^2));
maxGCal = max(abs(gErrorCal));

%% ======================== SCALE INFO ===============================

scaleDiag = diag(Ma);
scaleErrorPercent = (scaleDiag - 1) * 100;

crossAxis = Ma - diag(diag(Ma));
crossAxisPercent = crossAxis * 100;

biasMg = ba / g * 1000;  %mg

%% ====================== GYRO DIAGNOSTICS ===========================

% Your logger gyro output is deg/h.
gyroBiasMean_deg_h = mean(gyrMean_deg_h,1)';
gyroPositionStd_deg_h = std(gyrMean_deg_h,0,1)';
gyroStaticNoise_deg_h = mean(gyrStd_deg_h,1)';

% Convenience conversion to deg/s.
gyroBiasMean_deg_s = gyroBiasMean_deg_h / 3600;
gyroPositionStd_deg_s = gyroPositionStd_deg_h / 3600;
gyroStaticNoise_deg_s = gyroStaticNoise_deg_h / 3600;

%% ======================== TEMPERATURE ==============================

Tmin = min(tempMean);
Tmax = max(tempMean);
Tspan = Tmax - Tmin;

%% ======================== PRINT RESULTS ============================

fprintf('\n\n');
fprintf('============================================================\n');
fprintf('             24-POSITION CALIBRATION RESULTS\n');
fprintf('============================================================\n\n');

fprintf('Accelerometer bias ba [m/s^2]:\n');
disp(ba);

fprintf('Accelerometer bias [mg]:\n');
disp(biasMg);

fprintf('Measurement matrix Ma:\n');
disp(Ma);

fprintf('Correction matrix Ca = inv(Ma):\n');
disp(Ca);

fprintf('Scale-factor error [%%]:\n');
disp(scaleErrorPercent);

fprintf('Cross-axis terms [%%]:\n');
disp(crossAxisPercent);

fprintf('Condition number of Ma:\n');
fprintf('    %.6f\n\n', condMa);

fprintf('Calibration residual RMS XYZ [m/s^2]:\n');
disp(rmsXYZ);

fprintf('Overall component RMS [m/s^2]:\n');
fprintf('    %.8f\n\n', rmsTotal);

fprintf('Gravity norm RMS BEFORE calibration:\n');
fprintf('    %.8f m/s^2 = %.4f mg\n\n', ...
        rmsGRaw, rmsGRaw/g*1000);

fprintf('Gravity norm RMS AFTER calibration:\n');
fprintf('    %.8f m/s^2 = %.4f mg\n\n', ...
        rmsGCal, rmsGCal/g*1000);

fprintf('Maximum gravity norm error AFTER calibration:\n');
fprintf('    %.8f m/s^2 = %.4f mg\n\n', ...
        maxGCal, maxGCal/g*1000);

fprintf('Mean gyro static bias [deg/h]:\n');
disp(gyroBiasMean_deg_h);

fprintf('Mean gyro static bias [deg/s]:\n');
disp(gyroBiasMean_deg_s);

fprintf('Position-to-position gyro mean STD [deg/h]:\n');
disp(gyroPositionStd_deg_h);

fprintf('Average within-position gyro STD [deg/h]:\n');
disp(gyroStaticNoise_deg_h);

fprintf('Temperature range:\n');
fprintf('    Tmin = %.3f degC\n', Tmin);
fprintf('    Tmax = %.3f degC\n', Tmax);
fprintf('    dT   = %.3f degC\n\n', Tspan);

fprintf('Observed sample rate across files:\n');
fprintf('    min = %.4f Hz\n', min(sampleRate_Hz));
fprintf('    max = %.4f Hz\n', max(sampleRate_Hz));
fprintf('    avg = %.4f Hz\n\n', mean(sampleRate_Hz));

%% ======================== QUALITY CHECK ============================

fprintf('============================================================\n');
fprintf('                    QUALITY CHECK\n');
fprintf('============================================================\n');

rmsMg = rmsGCal / g * 1000;

if rmsMg < 5
    fprintf('ACC calibration quality: EXCELLENT\n');
elseif rmsMg < 10
    fprintf('ACC calibration quality: GOOD / ACCEPTABLE\n');
elseif rmsMg < 20
    fprintf('ACC calibration quality: MARGINAL\n');
    fprintf('Check table level, vibration, housing faces and temperature.\n');
else
    fprintf('ACC calibration quality: POOR\n');
    fprintf('Recommended to inspect the data and repeat the experiment.\n');
end

fprintf('Gravity norm RMS = %.3f mg\n', rmsMg);

if Tspan > 1
    fprintf('WARNING: temperature changed %.2f degC across the 24 records.\n', ...
            Tspan);
else
    fprintf('Temperature stability across records: OK (span %.2f degC)\n', ...
            Tspan);
end

if isempty(badIdx)
    fprintf('All 24 numbered files passed the orientation sanity check.\n');
else
    fprintf('%d file(s) failed the orientation sanity check.\n', ...
            numel(badIdx));
end

%% ======================== SUMMARY TABLE ============================

Position = (1:24)';
File = string(files);
Orientation = string(positionName);

summaryTable = table( ...
    Position, ...
    File, ...
    Orientation, ...
    Nsamples, ...
    duration_s, ...
    sampleRate_Hz, ...
    orientationErrorDeg, ...
    orientationOK, ...
    accMean(:,1), ...
    accMean(:,2), ...
    accMean(:,3), ...
    gNormRaw, ...
    rawNormError_mg, ...
    accCalMean(:,1), ...
    accCalMean(:,2), ...
    accCalMean(:,3), ...
    gNormCal, ...
    gErrorCal/g*1000, ...
    gyrMean_deg_h(:,1), ...
    gyrMean_deg_h(:,2), ...
    gyrMean_deg_h(:,3), ...
    tempMean, ...
    tempStd, ...
    'VariableNames',{ ...
    'Position', ...
    'File', ...
    'Orientation', ...
    'SamplesUsed', ...
    'FileDuration_s', ...
    'SampleRate_Hz', ...
    'OrientationError_deg', ...
    'OrientationOK', ...
    'AxRaw_m_s2', ...
    'AyRaw_m_s2', ...
    'AzRaw_m_s2', ...
    'RawNorm_m_s2', ...
    'RawNormError_mg', ...
    'AxCal_m_s2', ...
    'AyCal_m_s2', ...
    'AzCal_m_s2', ...
    'CalNorm_m_s2', ...
    'CalNormError_mg', ...
    'GxMean_deg_h', ...
    'GyMean_deg_h', ...
    'GzMean_deg_h', ...
    'Temperature_C', ...
    'TemperatureStd_C'} );

disp(summaryTable);

%% =========================== SAVE =================================

result.ba = ba;
result.Ma = Ma;
result.Ca = Ca;

result.biasMg = biasMg;
result.scaleErrorPercent = scaleErrorPercent;
result.crossAxisPercent = crossAxisPercent;

result.accMean = accMean;
result.accStd = accStd;
result.accCalMean = accCalMean;

result.reference = ref;
result.residual = residual;

result.rmsXYZ = rmsXYZ;
result.rmsTotal = rmsTotal;

result.gravityRMSRaw = rmsGRaw;
result.gravityRMSCal = rmsGCal;
result.gravityMaxError = maxGCal;

result.gyroBiasMean_deg_h = gyroBiasMean_deg_h;
result.gyroBiasMean_deg_s = gyroBiasMean_deg_s;
result.gyroPositionStd_deg_h = gyroPositionStd_deg_h;
result.gyroStaticNoise_deg_h = gyroStaticNoise_deg_h;

result.tempMin = Tmin;
result.tempMax = Tmax;
result.tempSpan = Tspan;

result.sampleRate_Hz = sampleRate_Hz;
result.orientationErrorDeg = orientationErrorDeg;
result.orientationOK = orientationOK;

result.config = cfg;

save(fullfile(cfg.dataDir, cfg.resultMat), 'result');

writetable(summaryTable, ...
    fullfile(cfg.dataDir, cfg.summaryCsv));

%% =========================== FIGURES ===============================

figure('Name','Gravity Norm Error');

plot(1:24, (gNormRaw-g)/g*1000, 'o-', 'LineWidth',1.2);
hold on;
plot(1:24, (gNormCal-g)/g*1000, 's-', 'LineWidth',1.2);
yline(0,'--');

xlabel('Position file number');
ylabel('Gravity norm error [mg]');
legend('Before calibration','After calibration','Location','best');
grid on;
title('24-position gravity magnitude error');


figure('Name','Accelerometer Calibration Residual');

plot(1:24, residual(:,1)/g*1000, 'o-', 'LineWidth',1.2);
hold on;
plot(1:24, residual(:,2)/g*1000, 's-', 'LineWidth',1.2);
plot(1:24, residual(:,3)/g*1000, '^-', 'LineWidth',1.2);
yline(0,'--');

xlabel('Position file number');
ylabel('Residual [mg]');
legend('X residual','Y residual','Z residual','Location','best');
grid on;
title('Accelerometer calibration residual');


figure('Name','Gyroscope Static Means');

plot(1:24, gyrMean_deg_h(:,1), 'o-', 'LineWidth',1.2);
hold on;
plot(1:24, gyrMean_deg_h(:,2), 's-', 'LineWidth',1.2);
plot(1:24, gyrMean_deg_h(:,3), '^-', 'LineWidth',1.2);

xlabel('Position file number');
ylabel('Gyro mean [deg/h]');
legend('Gx','Gy','Gz','Location','best');
grid on;
title('Gyroscope mean at each static position');


figure('Name','Orientation Sanity Check');

bar(1:24, orientationErrorDeg);
hold on;
yline(cfg.maxOrientationErrorDeg,'--');

xlabel('Position file number');
ylabel('Direction error [deg]');
grid on;
title('Expected-vs-measured gravity direction check');

%% ============================= END ================================

fprintf('\n');
fprintf('Calibration completed.\n');
fprintf('Saved in: %s\n', cfg.dataDir);
fprintf('  %s\n', cfg.resultMat);
fprintf('  %s\n', cfg.summaryCsv);
fprintf('\n');
fprintf('Use accelerometer correction as:\n');
fprintf('    a_cal = Ca * (a_raw - ba)\n\n');

function gamma_h = normal_gravity_wgs84(lat_deg, h)
% NORMAL_GRAVITY_WGS84
% Calculate WGS-84 normal gravity at latitude and ellipsoidal height.
%
% Inputs:
%   lat_deg : geodetic latitude [deg]
%   h       : ellipsoidal height [m]
%
% Output:
%   gamma_h : normal gravity [m/s^2]

%% WGS-84 constants
a  = 6378137.0;                % semi-major axis [m]
f  = 1 / 298.257223563;        % flattening
b  = a * (1 - f);              % semi-minor axis [m]

GM = 3.986004418e14;           % Earth's gravitational constant [m^3/s^2]
omega = 7.292115e-5;           % Earth rotation rate [rad/s]

% First eccentricity squared
e2 = f * (2 - f);

% WGS-84 normal gravity constants
gamma_e = 9.7803253359;        % equatorial gravity [m/s^2]
k = 0.00193185265241;

%% Latitude
phi = deg2rad(lat_deg);

sin2phi = sin(phi)^2;

%% Normal gravity on the ellipsoid surface
% Somigliana formula
gamma_0 = gamma_e * ...
    (1 + k * sin2phi) / ...
    sqrt(1 - e2 * sin2phi);

%% Height correction
% WGS-84 second-order normal gravity height correction

m = omega^2 * a^2 * b / GM;

gamma_h = gamma_0 * ...
    (1 ...
    - 2/a * (1 + f + m - 2*f*sin2phi) * h ...
    + 3*h^2/a^2);

end
