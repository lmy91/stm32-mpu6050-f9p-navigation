%% gm_autocorrelation_analysis.m
% 21 h 静态数据：温度补偿 + 24 位置标定后，用自相关函数辨识六轴一阶 GM 参数。
%
% 模型：
%   R_b(tau) = sigma_GM^2 * exp(-tau / T_c)
%
% 处理链（与 docs/温补标定SOP.md 一致）：
%   原始物理量 -> 逐样本温补 -> 24 位置确定性标定 -> 分块均值 -> ACF/GM 拟合
%
% 说明：
%   1. 使用非零延迟协方差拟合，避免把 lag=0 的白噪声尖峰计入 GM 方差。
%      最终 sigma_GM 使用块平均的解析传递函数反算回原始连续 GM 过程。
%   2. 默认只去均值，不去线性趋势；线性去趋势会压低长相关时间。如确认仍有
%      非随机线性漂移，可把 cfg.detrendMode 改为 'linear' 并比较两次结果。
%   3. 加速度计结果在传感器标定坐标系，陀螺结果单位为 deg/h。
%   4. 长时数据采用 datastore 分块读取，不会一次载入整个 1.75 GB CSV。
%
% 运行（仓库根目录）：
%   run('tools/noise_analysis/gm_autocorrelation_analysis.m')
%
% 输出目录：
%   data/decoded/<session>/gm_autocorrelation/
%     gm_parameters.csv
%     gm_block_series.csv
%     gm_results.mat
%     gm_autocorrelation_fit.png / .fig
%     gm_symmetric_autocovariance.png / .fig
%     gm_block_series.png / .fig

clear; clc; close all;

%% ======================== USER CONFIG =============================
cfg.projRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cfg.sessionName = '20260926005735';
cfg.imuFile = fullfile(cfg.projRoot, 'data', 'decoded', ...
    cfg.sessionName, 'imu.csv');

cfg.tempCoeffFile = fullfile(cfg.projRoot, 'data', 'calib24', ...
    'temp_coeffs_raw.mat');
cfg.calibFile = fullfile(cfg.projRoot, 'data', 'calib24', ...
    'calib24_result_tempcomp_azgxgy.mat');
cfg.outDir = fullfile(cfg.projRoot, 'data', 'decoded', ...
    cfg.sessionName, 'gm_autocorrelation');

cfg.averageSec = 10;           % 先做块均值，抑制单点白噪声 [s]
cfg.maxLagSec = 6*3600;        % ACF 最长延迟 [s]
cfg.minFitRho = 0.05;          % 拟合终点：ACF 首次降到此值附近
cfg.minFitPoints = 20;         % 至少使用的非零延迟点数
cfg.maxLagFraction = 0.25;     % 最大延迟不得超过总块数的该比例
cfg.detrendMode = 'constant';  % 'constant'（推荐）或 'linear'
cfg.readSize = 500000;         % 每次读取的 CSV 行数
cfg.minBlockCoverage = 0.80;   % 保留至少达到标称样本数此比例的时间块
cfg.saveFig = true;

axisNames = {'ax','ay','az','gx','gy','gz'};
nativeUnits = {'m/s^2','m/s^2','m/s^2','deg/h','deg/h','deg/h'};

assert(isfile(cfg.imuFile), '找不到 21 h IMU 数据：%s', cfg.imuFile);
assert(isfile(cfg.tempCoeffFile), '找不到温补系数：%s', cfg.tempCoeffFile);
assert(isfile(cfg.calibFile), '找不到标定结果：%s', cfg.calibFile);
assert(cfg.averageSec > 0 && cfg.maxLagSec > cfg.averageSec, ...
    'averageSec/maxLagSec 配置无效');
assert(any(strcmp(cfg.detrendMode, {'constant','linear'})), ...
    'cfg.detrendMode 只能为 constant 或 linear');
if ~isfolder(cfg.outDir); mkdir(cfg.outDir); end

%% ================= LOAD AND VERIFY PARAMETERS =====================
TC = load(cfg.tempCoeffFile);
assert(isfield(TC,'domain') && strcmp(TC.domain,'raw'), ...
    '温补系数必须属于原始物理量域 (domain=raw)');
assert(isfield(TC,'coef') && size(TC.coef,1)==6, ...
    'temp_coeffs_raw.mat 必须包含 6 行 coef');
assert(isfield(TC,'Tref') && isfield(TC,'Tmin') && isfield(TC,'Tmax'), ...
    '温补文件缺少 Tref/Tmin/Tmax，请重新运行温补拟合');

coef = TC.coef;
if size(coef,2) < 6
    coef = [zeros(6, 6-size(coef,2)), coef];
end
TCpoly = [coef(:,1:end-1), zeros(6,1)]; % [c_n ... c1 0]，不补 c0
if isfield(TC,'ord') && numel(TC.ord)==6
    tempOrder = TC.ord(:)';
else
    tempOrder = infer_orders(coef);
end
tcActive = tempOrder > 0;

CS = load(cfg.calibFile);
assert(isfield(CS,'result'), '标定 MAT 文件缺少 result 结构体');
calib = CS.result;
assert(isfield(calib,'ba') && isequal(size(calib.ba),[3 1]), ...
    '标定结果缺少 3x1 result.ba');
assert(isfield(calib,'Ca') && isequal(size(calib.Ca),[3 3]), ...
    '标定结果缺少 3x3 result.Ca');
assert(isfield(calib,'gyroBiasMean_deg_h') && ...
    numel(calib.gyroBiasMean_deg_h)==3, ...
    '标定结果缺少 result.gyroBiasMean_deg_h');

% 温补参数与 24 位置标定结果必须来自同一配置。
assert(isfield(calib,'tempCoeff') && isequal(size(calib.tempCoeff),size(TCpoly)), ...
    '标定结果缺少与当前温补矩阵同尺寸的 result.tempCoeff');
assert(max(abs(calib.tempCoeff(:)-TCpoly(:))) < 1e-10, ...
    '当前温补系数与 24 位置标定绑定的温补系数不一致，禁止混用');
assert(~isfield(calib,'Tref') || abs(calib.Tref-TC.Tref) < 1e-8, ...
    '当前温补 Tref 与标定结果 Tref 不一致');
assert(~isfield(calib,'tcActive') || ...
    isequal(logical(calib.tcActive(:)'), logical(tcActive)), ...
    '当前温补有效轴与标定结果不一致');

fprintf('============================================================\n');
fprintf('  21 h 温补 + 标定后六轴 GM 自相关辨识\n');
fprintf('============================================================\n');
fprintf('数据文件 : %s\n', cfg.imuFile);
fprintf('温补文件 : %s\n', cfg.tempCoeffFile);
fprintf('标定文件 : %s\n', cfg.calibFile);
fprintf('温补阶数 [ax ay az gx gy gz] = %s\n', mat2str(tempOrder));
fprintf('块均值 = %.1f s, 最大 ACF 延迟 = %.2f h, 去趋势 = %s\n\n', ...
    cfg.averageSec, cfg.maxLagSec/3600, cfg.detrendMode);

%% =========== STREAM RAW CSV, APPLY TC/CALIB, FORM BLOCKS =========
neededVars = {'time_s','ax_m_s2','ay_m_s2','az_m_s2', ...
    'temp_deg_c','gx_deg_h','gy_deg_h','gz_deg_h'};
ds = tabularTextDatastore(cfg.imuFile, 'Delimiter', ',', ...
    'ReadVariableNames', true, 'TextType', 'string');
missingVars = setdiff(neededVars, ds.VariableNames);
assert(isempty(missingVars), 'imu.csv 缺少字段：%s', strjoin(missingVars, ', '));
ds.SelectedVariableNames = neededVars;
ds.ReadSize = cfg.readSize;

blockSum = zeros(0,6);
blockCount = zeros(0,1);
tFirst = NaN; tLast = NaN; prevTime = NaN;
dtSum = 0; dtCount = 0; totalRows = 0; validRows = 0;
tempSeenMin = inf; tempSeenMax = -inf;
chunkNo = 0;

while hasdata(ds)
    D = read(ds);
    chunkNo = chunkNo + 1;
    totalRows = totalRows + height(D);

    t = double(D.time_s);
    temp = double(D.temp_deg_c);
    accRaw = [double(D.ax_m_s2), double(D.ay_m_s2), double(D.az_m_s2)];
    gyrRaw = [double(D.gx_deg_h), double(D.gy_deg_h), double(D.gz_deg_h)];
    valid = isfinite(t) & isfinite(temp) & ...
        all(isfinite(accRaw),2) & all(isfinite(gyrRaw),2);
    if ~all(valid)
        warning('第 %d 数据块跳过 %d 行非有限数据', chunkNo, sum(~valid));
        t=t(valid); temp=temp(valid); accRaw=accRaw(valid,:); gyrRaw=gyrRaw(valid,:);
    end
    if isempty(t); continue; end
    validRows = validRows + numel(t);

    if isnan(tFirst); tFirst = t(1); end
    if ~isnan(prevTime)
        assert(t(1) > prevTime, 'time_s 在数据块边界处不单调递增');
        dtSum = dtSum + (t(1)-prevTime); dtCount = dtCount + 1;
    end
    dt = diff(t);
    assert(all(dt>0), 'time_s 在第 %d 数据块内不单调递增', chunkNo);
    dtSum = dtSum + sum(dt); dtCount = dtCount + numel(dt);
    prevTime = t(end); tLast = t(end);

    tempSeenMin = min(tempSeenMin, min(temp));
    tempSeenMax = max(tempSeenMax, max(temp));
    if any(temp < TC.Tmin-1e-9 | temp > TC.Tmax+1e-9)
        error(['21 h 数据温度超出拟合温区 [%.4f, %.4f] degC；' ...
               '禁止外推温补。当前数据块范围 [%.4f, %.4f] degC。'], ...
               TC.Tmin, TC.Tmax, min(temp), max(temp));
    end

    dT = temp - TC.Tref;
    drift = zeros(numel(temp),6);
    for a = 1:6
        if tcActive(a)
            drift(:,a) = polyval(TCpoly(a,:), dT);
        end
    end
    accTC = accRaw - drift(:,1:3);
    gyrTC = gyrRaw - drift(:,4:6);

    % a_cal = Ca * (a_TC-ba); g_cal = g_TC-gb。
    accCal = (calib.Ca * (accTC - calib.ba(:)').').';
    gyrCal = gyrTC - calib.gyroBiasMean_deg_h(:)';
    Y = [accCal, gyrCal];

    bin = floor((t-tFirst)/cfg.averageSec) + 1;
    maxBin = max(bin);
    if maxBin > size(blockSum,1)
        blockSum(maxBin,6) = 0;
        blockCount(maxBin,1) = 0;
    end
    for a = 1:6
        blockSum(:,a) = blockSum(:,a) + ...
            accumarray(bin, Y(:,a), [size(blockSum,1),1], @sum, 0);
    end
    blockCount = blockCount + ...
        accumarray(bin, 1, [numel(blockCount),1], @sum, 0);

    if chunkNo==1 || mod(chunkNo,5)==0
        fprintf('已读取 %.2f 百万行，当前时长 %.2f h\n', ...
            totalRows/1e6, (tLast-tFirst)/3600);
    end
end

assert(validRows > 0 && isfinite(tFirst) && isfinite(tLast), '没有可用 IMU 数据');
fsMean = dtCount / dtSum;
nominalPerBlock = cfg.averageSec * fsMean;
keep = blockCount >= cfg.minBlockCoverage*nominalPerBlock;
if any(~keep & blockCount>0)
    warning('丢弃 %d 个覆盖率不足 %.0f%% 的块', ...
        sum(~keep & blockCount>0), 100*cfg.minBlockCoverage);
end
blockMean = blockSum(keep,:) ./ blockCount(keep);
allBlockCenter = ((1:numel(blockCount))'-0.5)*cfg.averageSec;
blockTime = allBlockCenter(keep);

% ACF 要求等间隔连续序列；若中间有缺块，避免跨缺口拟合。
keptIndex = find(keep);
if any(diff(keptIndex)~=1)
    runs = contiguous_runs(keptIndex);
    [~,longest] = max(runs(:,2)-runs(:,1)+1);
    useIndex = runs(longest,1):runs(longest,2);
    warning('数据含时间缺口；ACF 仅使用最长连续段（%d 个 %.1f s 块）', ...
        numel(useIndex), cfg.averageSec);
    useMask = ismember(keptIndex, useIndex);
    acfData = blockMean(useMask,:);
    acfTime = blockTime(useMask);
else
    acfData = blockMean;
    acfTime = blockTime;
end
assert(size(acfData,1) >= 4*cfg.minFitPoints, ...
    '有效连续块太少，无法辨识 GM 参数');

fprintf('\n读取完成：%d 行，%.3f h，平均采样率 %.5f Hz\n', ...
    validRows, (tLast-tFirst)/3600, fsMean);
fprintf('温度范围：[%.4f, %.4f] degC；ACF 连续块数：%d\n\n', ...
    tempSeenMin, tempSeenMax, size(acfData,1));

%% ======================= ACF AND GM FIT ============================
n = size(acfData,1);
maxLagBlocks = min([floor(cfg.maxLagSec/cfg.averageSec), ...
    floor(cfg.maxLagFraction*n), n-2]);
assert(maxLagBlocks >= cfg.minFitPoints, '最大延迟配置不足以进行拟合');
lagSec = (0:maxLagBlocks)'*cfg.averageSec;

acf = nan(maxLagBlocks+1,6);
autocovariance = nan(maxLagBlocks+1,6);
acfFit = nan(maxLagBlocks+1,6);
sigmaBlockFitNative = nan(6,1); tauC = nan(6,1); empiricalEfold = nan(6,1);
fitEndSec = nan(6,1); fitPoints = zeros(6,1); fitR2 = nan(6,1);
blockStdNative = nan(6,1); lagOneRho = nan(6,1);
gmVarianceFraction = nan(6,1); residualWhiteStdNative = nan(6,1);
fitStatus = strings(6,1);

for a = 1:6
    x = acfData(:,a);
    if strcmp(cfg.detrendMode,'linear')
        x = detrend(x,1);
    else
        x = x - mean(x);
    end
    blockStdNative(a) = std(x,0);
    C = autocov_fft_unbiased(x, maxLagBlocks);
    autocovariance(:,a) = C;
    acf(:,a) = C/C(1);
    lagOneRho(a) = acf(2,a);

    j = find(acf(:,a) <= exp(-1), 1, 'first');
    if ~isempty(j) && j>1
        empiricalEfold(a) = interp_crossing(lagSec(j-1:j), ...
            acf(j-1:j,a), exp(-1));
    end

    [sigmaBlockFitNative(a), tauC(a), fitEndSec(a), fitPoints(a), ...
        fitR2(a), acfFit(:,a), gmVarianceFraction(a), fitStatus(a)] = ...
        fit_gm_covariance(lagSec, C, cfg.minFitRho, cfg.minFitPoints);
    residualWhiteStdNative(a) = sqrt(max(C(1)*(1-gmVarianceFraction(a)),0));
end

% 非重叠块均值的非零延迟协方差：
% Cbar(m*Delta)=sigma^2*[sinh(x/2)/(x/2)]^2*exp(-m*Delta/Tc), x=Delta/Tc。
% 因此把非零延迟拟合的截距反算为原始连续 GM 过程的稳态 sigma。
xAverage = cfg.averageSec./tauC;
lagAverageFactor = (sinh(xAverage/2)./(xAverage/2)).^2;
lagAverageFactor(abs(xAverage)<sqrt(eps)) = 1;
averagingSigmaCorrection = 1./sqrt(lagAverageFactor);
sigmaNative = sigmaBlockFitNative.*averagingSigmaCorrection;

% 工程单位：加计用 mg，陀螺保持 deg/h。
engineeringUnit = [repmat("mg",3,1); repmat("deg/h",3,1)];
gmStdEngineering = sigmaNative;
blockStdEngineering = blockStdNative;
residualWhiteStdEngineering = residualWhiteStdNative;
gmStdEngineering(1:3) = sigmaNative(1:3)/9.80665*1000;
blockStdEngineering(1:3) = blockStdNative(1:3)/9.80665*1000;
residualWhiteStdEngineering(1:3) = ...
    residualWhiteStdNative(1:3)/9.80665*1000;

sensor = [repmat("accel",3,1); repmat("gyro",3,1)];
axis = ["X";"Y";"Z";"X";"Y";"Z"];
nativeUnit = string(nativeUnits(:));
observedCorrelationCycles = (acfTime(end)-acfTime(1))./tauC;
decayRate_1_s = 1./tauC;
drivingNoiseDensityNative = sigmaNative.*sqrt(2./tauC);
reliable = fitR2 >= 0.80 & observedCorrelationCycles >= 10 & ...
    fitPoints >= cfg.minFitPoints & fitStatus=="OK";

gmTable = table(sensor, axis, sigmaNative, nativeUnit, ...
    gmStdEngineering, engineeringUnit, tauC, tauC/3600, ...
    empiricalEfold, blockStdNative, blockStdEngineering, lagOneRho, ...
    gmVarianceFraction, residualWhiteStdNative, ...
    residualWhiteStdEngineering, observedCorrelationCycles, ...
    averagingSigmaCorrection, decayRate_1_s, drivingNoiseDensityNative, ...
    fitEndSec, fitPoints, fitR2, reliable, fitStatus, ...
    'VariableNames', {'Sensor','Axis','GMStdNative','NativeUnit', ...
    'GMStdEngineering','EngineeringUnit','CorrelationTime_s', ...
    'CorrelationTime_h','EmpiricalEfold_s','BlockStdNative', ...
    'BlockStdEngineering','LagOneCorrelation', ...
    'GMVarianceFraction','ResidualWhiteStdNative', ...
    'ResidualWhiteStdEngineering','ObservedCorrelationCycles', ...
    'AveragingSigmaCorrection','DecayRate_1_s', ...
    'DrivingNoiseDensityNativePerSqrtS', ...
    'FitEnd_s','FitPoints','FitR2','Reliable','FitStatus'});

fprintf('========== 一阶 GM 自相关拟合结果 ==========\n');
disp(gmTable(:,{'Sensor','Axis','GMStdEngineering','EngineeringUnit', ...
    'CorrelationTime_s','CorrelationTime_h','FitR2','Reliable','FitStatus'}));

%% ========================= TABLE OUTPUTS ===========================
outParamCsv = fullfile(cfg.outDir, 'gm_parameters.csv');
outSeriesCsv = fullfile(cfg.outDir, 'gm_block_series.csv');
outMat = fullfile(cfg.outDir, 'gm_results.mat');
writetable(gmTable, outParamCsv);

blockTable = array2table([blockTime, blockCount(keep), blockMean], ...
    'VariableNames', {'TimeFromStart_s','SamplesInBlock', ...
    'ax_cal_m_s2','ay_cal_m_s2','az_cal_m_s2', ...
    'gx_cal_deg_h','gy_cal_deg_h','gz_cal_deg_h'});
writetable(blockTable, outSeriesCsv);

metadata = struct();
metadata.sourceFile = cfg.imuFile;
metadata.sourceRows = validRows;
metadata.duration_s = tLast-tFirst;
metadata.meanSampleRate_Hz = fsMean;
metadata.temperatureRange_C = [tempSeenMin tempSeenMax];
metadata.tempOrder = tempOrder;
metadata.tcActive = tcActive;
metadata.processingOrder = ...
    'raw physical units -> temperature compensation -> 24-position calibration -> block mean -> ACF';
metadata.model = 'R(tau)=sigma_GM^2*exp(-tau/T_c)';
metadata.note = ['lag=0 excluded from fit; sigma_GM is the fitted covariance ' ...
    'intercept corrected analytically for block averaging and does not include ' ...
    'the zero-lag white-noise spike'];
save(outMat, 'cfg','metadata','gmTable','blockTable','lagSec', ...
    'autocovariance','acf','acfFit');

%% ============================= PLOTS ===============================
colors = lines(6);
fig1 = figure('Name','Six-axis GM autocorrelation fit', ...
    'Color','w','Position',[60 40 1380 800]);
for a = 1:6
    subplot(2,3,a); hold on;
    plot(lagSec(2:end), acf(2:end,a), '-', ...
        'Color',colors(a,:), 'LineWidth',1.2);
    if any(isfinite(acfFit(:,a)))
        fitMask = lagSec>0 & lagSec<=fitEndSec(a);
        plot(lagSec(fitMask), acfFit(fitMask,a), 'k--', 'LineWidth',1.5);
        xline(tauC(a), ':', 'Color',[0.25 0.25 0.25]);
    end
    yline(exp(-1), ':', 'Color',[0.55 0.55 0.55]);
    yline(0, '-', 'Color',[0.75 0.75 0.75]);
    grid on; box on; ylim([-0.25 1.05]);
    xlim([cfg.averageSec max(lagSec(end),cfg.averageSec*2)]);
    xlabel('延迟 \tau (s)'); ylabel('归一化自相关');
    title(sprintf('%s: \\sigma_{GM}=%.4g %s, T_c=%.1f s, R^2=%.3f', ...
        axisNames{a}, gmStdEngineering(a), engineeringUnit(a), ...
        tauC(a), fitR2(a)));
    if a==1
        legend({'经验 ACF','一阶 GM 拟合','T_c','e^{-1}','零线'}, ...
            'Location','best');
    end
end
sgtitle(sprintf('温补 + 标定后六轴自相关与一阶 GM 拟合（%.0f s 块均值）', ...
    cfg.averageSec));
exportgraphics(fig1, fullfile(cfg.outDir,'gm_autocorrelation_fit.png'), ...
    'Resolution',180);
if cfg.saveFig; savefig(fig1, fullfile(cfg.outDir,'gm_autocorrelation_fit.fig')); end

% 与常见教材示意图一致的双边自协方差：r(+tau)=r(-tau)。
figSym = figure('Name','Symmetric GM autocovariance', ...
    'Color','w','Position',[70 45 1380 800]);
layoutSym = tiledlayout(figSym,2,3,'TileSpacing','compact','Padding','compact');
for a = 1:6
    nexttile(layoutSym,a); hold on;
    scale2 = 1;
    if a<=3; scale2=(1000/9.80665)^2; end
    lagSym = [-flipud(lagSec(2:end)); lagSec];
    covSym = [flipud(autocovariance(2:end,a)); autocovariance(:,a)]*scale2;
    gmCurve = gmStdEngineering(a)^2*exp(-abs(lagSym)/tauC(a));
    plot(lagSym,covSym,'-','Color',[0.68 0.68 0.68],'LineWidth',0.9);
    plot(lagSym,gmCurve,'b-','LineWidth',1.8);
    yline(0,'k-','LineWidth',0.8);
    xline(0,'k-','LineWidth',0.8);
    sigma2 = gmStdEngineering(a)^2;
    eLevel = exp(-1)*sigma2;
    plot([-tauC(a),tauC(a)],[eLevel,eLevel],'k--','LineWidth',1.0);
    plot([-tauC(a),-tauC(a)],[0,eLevel],'k--','LineWidth',1.0);
    plot([ tauC(a), tauC(a)],[0,eLevel],'k--','LineWidth',1.0);
    plot(0,sigma2,'bo','MarkerFaceColor','b','MarkerSize',4);
    text(0,sigma2,sprintf('  \\sigma^2=%.4g',sigma2), ...
        'VerticalAlignment','bottom','FontSize',8);
    text(tauC(a),eLevel,sprintf('  0.3679\\sigma^2'), ...
        'VerticalAlignment','bottom','FontSize',8);
    grid on; box on;
    xlim([-lagSec(end),lagSec(end)]);
    xlabel('延迟 \tau (s)');
    ylabel(sprintf('r(\\tau) [(%s)^2]',engineeringUnit(a)));
    title(sprintf('%s: \\sigma_{GM}=%.4g %s, T_c=%.1f s', ...
        axisNames{a},gmStdEngineering(a),engineeringUnit(a),tauC(a)));
    if a==1
        legend({'经验双边自协方差','一阶 GM: \sigma^2e^{-|\tau|/T_c}', ...
            '零线','\tau=0','0.3679\sigma^2 与 \pmT_c'}, ...
            'Location','best');
    end
end
title(layoutSym,sprintf(['温补 + 标定后六轴双边自协方差（%.0f s 块均值；' ...
    '\\sigma 已反算为原始连续 GM 过程）'],cfg.averageSec));
exportgraphics(figSym,fullfile(cfg.outDir,'gm_symmetric_autocovariance.png'), ...
    'Resolution',180);
if cfg.saveFig
    savefig(figSym,fullfile(cfg.outDir,'gm_symmetric_autocovariance.fig'));
end

fig2 = figure('Name','Calibrated block-mean series', ...
    'Color','w','Position',[80 60 1380 800]);
tHour = blockTime/3600;
for a = 1:6
    subplot(2,3,a);
    y = blockMean(:,a)-mean(blockMean(:,a));
    if a<=3
        y = y/9.80665*1000;
        unitText = 'mg';
    else
        unitText = 'deg/h';
    end
    plot(tHour,y,'Color',colors(a,:),'LineWidth',0.9);
    grid on; box on;
    xlabel('时间 (h)'); ylabel(sprintf('去均值残差 (%s)',unitText));
    title(sprintf('%s：温补+标定后 %.0f s 块均值',axisNames{a},cfg.averageSec));
end
sgtitle('用于 GM 自相关辨识的六轴块均值序列（请检查平稳性与异常段）');
exportgraphics(fig2, fullfile(cfg.outDir,'gm_block_series.png'), ...
    'Resolution',180);
if cfg.saveFig; savefig(fig2, fullfile(cfg.outDir,'gm_block_series.fig')); end

fprintf('\n输出完成：\n');
fprintf('  参数表   %s\n', outParamCsv);
fprintf('  块均值表 %s\n', outSeriesCsv);
fprintf('  MAT 结果 %s\n', outMat);
fprintf('  图        %s\n', fullfile(cfg.outDir,'gm_autocorrelation_fit.png'));
fprintf('            %s\n', fullfile(cfg.outDir,'gm_symmetric_autocovariance.png'));
fprintf('            %s\n', fullfile(cfg.outDir,'gm_block_series.png'));
fprintf(['\nReliable=false 表示该轴不宜直接按一阶 GM 参数使用；请结合时序图、' ...
    '拟合 R^2 和独立静态数据复核。\n']);

%% ========================== LOCAL FUNCTIONS ========================
function ord = infer_orders(coef)
maxOrder = size(coef,2)-1;
ord = zeros(1,size(coef,1));
for a = 1:size(coef,1)
    first = find(abs(coef(a,1:end-1))>0,1,'first');
    if ~isempty(first); ord(a)=maxOrder-first+1; end
end
end

function runs = contiguous_runs(index)
% 返回原始块编号中的连续段 [start,end]。
cut = [true; diff(index(:))~=1; true];
pos = find(cut);
runs = [index(pos(1:end-1)), index(pos(2:end)-1)];
end

function C = autocov_fft_unbiased(x,maxLag)
% 无工具箱依赖的 FFT 无偏自协方差。
x = x(:);
n = numel(x);
nfft = 2^nextpow2(2*n-1);
F = fft(x,nfft);
raw = real(ifft(F.*conj(F)));
C = raw(1:maxLag+1) ./ (n-(0:maxLag))';
end

function tCross = interp_crossing(t,rho,target)
if rho(2)==rho(1)
    tCross = mean(t);
else
    tCross = t(1)+(target-rho(1))*(t(2)-t(1))/(rho(2)-rho(1));
end
end

function [sigma,tau,fitEnd,nFit,R2,rhoFit,q,status] = ...
    fit_gm_covariance(lagSec,C,minFitRho,minFitPoints)
% 用非零延迟正协方差拟合 q*exp(-t/Tc)，并约束 0<q<=1。
% q=A/C(0) 是块均值总方差中的 GM 方差比例，可防止拟合的 GM 方差
% 大于实际总方差；剩余 1-q 解释为块均值中尚存的白噪声方差。
rho = C/C(1);
last = find(rho(2:end)<=minFitRho | ~isfinite(rho(2:end)),1,'first');
if isempty(last)
    endIdx = numel(rho);
else
    endIdx = last+1;
end
endIdx = max(endIdx,min(numel(rho),minFitPoints+1));
idx = (2:endIdx)';
idx = idx(isfinite(C(idx)) & C(idx)>0 & isfinite(rho(idx)));

rhoFit = nan(size(rho)); sigma=NaN; tau=NaN; fitEnd=NaN; q=NaN;
nFit=numel(idx); R2=NaN; status="拟合点不足";
if nFit<minFitPoints; return; end

t = lagSec(idx);
y = rho(idx);
z = log(C(idx));
X = [ones(nFit,1),t];
beta0 = X\z;
if ~all(isfinite(beta0)) || beta0(2)>=0
    status="非衰减 ACF"; return;
end

q0 = min(max(exp(beta0(1))/C(1),1e-4),1-1e-6);
tau0 = -1/beta0(2);
p0 = [log(q0/(1-q0)), log(tau0)];
w = sqrt(max(y,0.05));
objective = @(p) sum(w.*(y-logistic(p(1))*exp(-t/exp(p(2)))).^2);
opt = optimset('Display','off','MaxIter',2000,'MaxFunEvals',4000, ...
    'TolX',1e-10,'TolFun',1e-12);
p = fminsearch(objective,p0,opt);
q = logistic(p(1));
tau = exp(p(2));
if ~isfinite(q) || ~isfinite(tau) || tau<=0
    status="约束拟合失败"; q=NaN; tau=NaN; return;
end

sigma = sqrt(q*C(1));
fitEnd = max(t);
rhoFit = q*exp(-lagSec/tau);

yhat = q*exp(-t/tau);
ssRes = sum((y-yhat).^2);
ssTot = sum((y-mean(y)).^2);
R2 = 1-ssRes/max(ssTot,eps);
status="OK";
end

function q = logistic(x)
% 数值稳定的 logistic 映射，将无界参数映射到 (0,1)。
if x>=0
    q = 1/(1+exp(-x));
else
    ex = exp(x); q = ex/(1+ex);
end
end
