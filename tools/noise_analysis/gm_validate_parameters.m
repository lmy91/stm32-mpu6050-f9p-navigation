function results = gm_validate_parameters(cfg)
%GM_VALIDATE_PARAMETERS Audit the existing GM model without replacing it.
%   addpath('tools'); setup_tools; results = gm_validate_parameters;
%   results = gm_validate_parameters(struct('figureVisible','on'));
%   gm_validate_parameters(struct('replotOnly',true)); % saved results only
%
% Independent records must have externally verified, fixed-pose windows:
%   r.file = 'absolute/path/to/imu.csv'; r.label = 'independent static';
%   r.windows_s = [start_time_s end_time_s]; % original CSV time_s domain
%   r.verifiedStatic = true; r.staticEvidence = 'how this was verified';
%   results = gm_validate_parameters(struct('independentRecords',r));
% Do not mark a moving vehicle or a collection of different poses as static.
% Short windows can test short-term noise, but cannot validate hour-scale Tc.
% Outputs stay in gm_validation/; gm_parameters.csv is never overwritten.

if nargin < 1; cfg = struct(); end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
session = fullfile(root,'data','decoded','20260926005735');
defaults = struct('referenceMat',fullfile(session,'gm_autocorrelation','gm_results.mat'), ...
    'tempCoeffFile',fullfile(root,'data','calib24','temp_coeffs_raw.mat'), ...
    'calibFile',fullfile(root,'data','calib24','calib24_result_tempcomp_azgxgy.mat'), ...
    'allanParamFile',fullfile(session,'allan_compare_tc_configs','allan_three_configs_params.csv'), ...
    'allanCurveFile',fullfile(session,'allan_compare_tc_configs','allan_deviation.csv'), ...
    'allanFixedTauFile',fullfile(session,'allan_compare_tc_configs','allan_adev_at_tau.csv'), ...
    'outDir',fullfile(session,'gm_validation'), 'taus_s',[30 60 120 300 600], ...
    'minCorrelationTimes',10,'minAllanClusters',10,'sigmaRatioBounds',[0.67 1.5], ...
    'tauRatioBounds',[0.5 2],'allanRatioBounds',[0.8 1.25], ...
    'readSize',500000,'figureVisible','off','saveFig',false,'independentAverageSec',1, ...
    'independentRecords',default_independent_records(root),'replotOnly',false, ...
    'cachedResultFile',fullfile(session,'gm_validation','gm_validation.mat'));
cfg = with_defaults(cfg,defaults);
names = ["ax","ay","az","gx","gy","gz"];
units = ["mg","mg","mg","deg/h","deg/h","deg/h"];
if cfg.replotOnly
    assert(isfile(cfg.cachedResultFile),'Saved validation result missing: %s',cfg.cachedResultFile);
    cached=load(cfg.cachedResultFile,'results');results=cached.results;
    results.cfg.outDir=cfg.outDir;results.cfg.figureVisible=cfg.figureVisible;results.cfg.saveFig=cfg.saveFig;
    if ~isfolder(cfg.outDir);mkdir(cfg.outDir);end
    make_plots(results,results.summary.ReferenceSigma,results.summary.ReferenceTc_s, ...
        names,units,results.series(1).blockSec);
    fprintf('Replotted saved validation results; no source CSV was reread.\n');return;
end
assert(isfile(cfg.referenceMat),'Reference GM MAT not found: %s',cfg.referenceMat);
S = load(cfg.referenceMat);
assert(all(isfield(S,{'cfg','metadata','gmTable','blockTable'})), ...
    'Reference MAT must contain cfg, metadata, gmTable, and blockTable.');
delta = S.cfg.averageSec;
assert(all(cfg.taus_s > 0) && all(abs(cfg.taus_s/delta-round(cfg.taus_s/delta))<1e-8), ...
    'Validation taus must be positive integer multiples of the stored block length.');
assert(cfg.minCorrelationTimes>0 && cfg.minAllanClusters>=3,'Invalid validation thresholds.');
assert(cfg.independentAverageSec>0 && cfg.independentAverageSec<=1 && ...
    all(abs(cfg.taus_s/cfg.independentAverageSec-round(cfg.taus_s/cfg.independentAverageSec))<1e-8), ...
    'Independent Allan blocks must be at most 1 s and divide the requested taus.');
assert(~strcmpi(cfg.outDir,fileparts(cfg.referenceMat)), ...
    'Use a separate output directory; original GM outputs must be preserved.');
if ~isfolder(cfg.outDir); mkdir(cfg.outDir); end

[TC,C,provenance] = check_provenance(cfg,S);
sanityTable=synthetic_sanity();
assert(all(sanityTable.Passed),'Synthetic GM/Allan sanity check failed.');
refSigma = S.gmTable.GMStdEngineering(:);
refTau = S.gmTable.CorrelationTime_s(:);
assert(height(S.gmTable)==6 && isequal(lower(string(S.gmTable.Sensor(:))), ...
    ["accel";"accel";"accel";"gyro";"gyro";"gyro"]) && ...
    isequal(upper(string(S.gmTable.Axis(:))),["X";"Y";"Z";"X";"Y";"Z"]), ...
    'Reference parameters must use the original sensor-axis order ax ay az gx gy gz.');
white = load_calibrated_white(cfg.allanParamFile,names);
series = struct('name',"training",'kind',"Training", ...
    'time',S.blockTable.TimeFromStart_s,'values',to_engineering(S.blockTable{:,3:8}), ...
    'file',string(S.metadata.sourceFile),'status',"TrainingResiduals",'blockSec',delta);
assert(all(diff(series.time)>0) && all(isfinite(series.values),'all'),'Invalid stored blocks.');
trainingIndex = longest_run(series.time,delta);
series.time = series.time(trainingIndex);
series.values = series.values(trainingIndex,:);

segmentRows = struct([]); modelRows = struct([]); recordRows = struct([]);
n = numel(series.time);
segments = {"full_constant",1:n,false; "full_linear",1:n,true};
for parts = [2 4]
    for k = 1:parts
        ii = floor((k-1)*n/parts)+1:floor(k*n/parts);
        segments(end+1,:) = {sprintf('%d_part_%d',parts,k),ii,false}; %#ok<AGROW>
    end
end
for k = 1:size(segments,1)
    ii = segments{k,2};
    rows = analyze_segment(series.time(ii),series.values(ii,:), ...
        string(segments{k,1}),"TrainingSensitivity",segments{k,3}, ...
        delta,S.cfg,refSigma,refTau,cfg,names,units);
    segmentRows = append_rows(segmentRows,rows);
end
observed = load_observed_allan(cfg,names,series.values,delta);
modelRows = append_rows(modelRows,compare_allan("training", "Training", ...
    series.values,delta,observed,refSigma,refTau,white,cfg,names,units));
recordRows = append_rows(recordRows,record_row("training",series.file, ...
    "TrainingResiduals",n*delta,S.metadata.temperatureRange_C, ...
    "Existing calibrated blocks; same record used for temperature fitting."));

for k = 1:numel(cfg.independentRecords)
    r = cfg.independentRecords(k);
    try
        fprintf('Reading verified independent record: %s\n',string(r.file));
        [windows,info] = read_independent(r,TC,C,cfg,cfg.independentAverageSec,string(S.metadata.sourceFile));
    catch err
        label = string(getfield_default(r,'label',sprintf('independent_%d',k)));
        file = string(getfield_default(r,'file',''));
        recordRows = append_rows(recordRows,record_row(label,file,"ReadOrConfigurationError", ...
            NaN,[NaN NaN],string(err.message)));
        warning('GM validation: independent record %s skipped: %s',label,err.message);
        continue;
    end
    recordRows = append_rows(recordRows,info);
    for j = 1:numel(windows)
        w = windows(j);
        series(end+1) = w; %#ok<AGROW>
        rows = analyze_segment(w.time,w.values,w.name,"Independent",false, ...
            w.blockSec,S.cfg,refSigma,refTau,cfg,names,units);
        segmentRows = append_rows(segmentRows,rows);
        modelRows = append_rows(modelRows,compare_allan(w.name,"Independent", ...
            w.values,w.blockSec,[],refSigma,refTau,white,cfg,names,units));
    end
end

segmentTable = struct2table(segmentRows);
allanTable = struct2table(modelRows);
recordTable = struct2table(recordRows);
summaryRows = struct([]);
for a = 1:6
    sr = segmentTable.Axis==names(a) & segmentTable.RecordKind=="TrainingSensitivity" & ...
        segmentTable.Segment~="full_constant";
    usable = sr & segmentTable.FitStatus=="OK";
    sensitive = any(~segmentTable.WithinSensitivityBounds(sr));
    ar = allanTable.Axis==names(a) & allanTable.RecordKind=="Training";
    mismatch = any(~allanTable.WithinAllanBounds(ar) & allanTable.EvidenceStatus(ar)=="Usable");
    independent = segmentTable.Axis==names(a) & segmentTable.RecordKind=="Independent";
    independentLong = independent & segmentTable.IndependentTcStatus=="LongRecordConsistent";
    independentAllan=allanTable.Axis==names(a)&allanTable.RecordKind=="Independent";
    usableIndependentAllan=independentAllan&allanTable.EvidenceStatus=="Usable";
    independentAllanMismatch=any(~allanTable.WithinAllanBounds(usableIndependentAllan));
    if ~any(independentAllan); independentAllanStatus="MissingIndependentAllanRecord";
    elseif ~any(usableIndependentAllan); independentAllanStatus="InsufficientDisjointAllanPairs";
    elseif independentAllanMismatch; independentAllanStatus="EngineeringBoundsMismatch";
    else; independentAllanStatus="WithinEngineeringBounds_NotStatisticalValidation";
    end
    if ~any(independent); independentStatus="MissingVerifiedIndependentStaticRecord";
    elseif ~any(segmentTable.Span_s(independent)./refTau(a)>=cfg.minCorrelationTimes)
        independentStatus="InsufficientIndependentDurationForTc";
    elseif any(independentLong); independentStatus="IndependentCandidateConsistent";
    else; independentStatus="IndependentTcFitNotConsistent";
    end
    status = "Provisional";
    if mismatch; status=status+"_AllanMismatch"; end
    if sensitive; status=status+"_SegmentOrTrendSensitive"; end
    if independentAllanMismatch;status=status+"_IndependentAllanMismatch";end
    status=status+"_"+independentStatus;
    row = struct('Axis',names(a),'Unit',units(a),'ReferenceSigma',refSigma(a), ...
        'ReferenceTc_s',refTau(a),'TrainingDurationOverTc',n*delta/refTau(a), ...
        'MinSensitivitySigmaRatio',finite_min(segmentTable.SigmaRatio(usable)), ...
        'MaxSensitivitySigmaRatio',finite_max(segmentTable.SigmaRatio(usable)), ...
        'MinSensitivityTcRatio',finite_min(segmentTable.TcRatio(usable)), ...
        'MaxSensitivityTcRatio',finite_max(segmentTable.TcRatio(usable)), ...
        'AllanMismatch',mismatch,'SegmentOrTrendSensitive',sensitive, ...
        'IndependentAllanStatus',independentAllanStatus,'IndependentAllanMismatch',independentAllanMismatch, ...
        'UsableIndependentAllanRows',nnz(usableIndependentAllan), ...
        'UsableIndependentAllanRecords',numel(unique(allanTable.Record(usableIndependentAllan))), ...
        'UsableIndependentAllanTaus',numel(unique(allanTable.Tau_s(usableIndependentAllan))), ...
        'IndependentTcStatus',independentStatus,'Status',status, ...
        'ProvenanceStatus',provenance.status);
    summaryRows = append_rows(summaryRows,row);
end
summaryTable = struct2table(summaryRows);
writetable(segmentTable,fullfile(cfg.outDir,'gm_validation_segments.csv'));
writetable(allanTable,fullfile(cfg.outDir,'gm_validation_allan.csv'));
writetable(recordTable,fullfile(cfg.outDir,'gm_validation_records.csv'));
writetable(summaryTable,fullfile(cfg.outDir,'gm_validation_summary.csv'));
writetable(sanityTable,fullfile(cfg.outDir,'gm_validation_sanity.csv'));
results = struct('cfg',cfg,'provenance',provenance,'summary',summaryTable, ...
    'segments',segmentTable,'allan',allanTable,'records',recordTable, ...
    'whiteNoiseInEngineeringUnitsPerSqrtS',white,'series',series,'sanity',sanityTable);
save(fullfile(cfg.outDir,'gm_validation.mat'),'results','TC','C');
make_plots(results,refSigma,refTau,names,units,delta);
fprintf('\nGM validation saved to %s\n',cfg.outDir);
disp(summaryTable(:,{'Axis','AllanMismatch','IndependentAllanStatus','SegmentOrTrendSensitive','IndependentTcStatus'}));
fprintf(['All statuses remain provisional. Thresholds are engineering screening bounds, ' ...
    'not confidence intervals. Disjoint pair counts are not effective degrees of freedom. ' ...
    'Training partitions are not independent validation; fixed-pose captures are not bias truth or power-cycle repeatability.\n']);
end

function records = default_independent_records(root)
% These fixed-pose windows were separately audited against sample continuity,
% temperature bounds, within-block noise, direction stability and GNSS speed.
% Separate captures do not prove a power cycle or bias repeatability.
base=fullfile(root,'data','decoded');
evidence="Audited fixed-pose candidate: no sample/time gaps, no saturation or moving blocks; gravity direction variation below 0.028 deg in the selected long window and GNSS speed below 0.10 m/s. Small fixture tilt cannot be separated from sensor drift using IMU alone. Separate capture from training; power-cycle independence and bias truth are not established.";
records(1)=struct('file',fullfile(base,'20260921140624','imu.csv'), ...
    'label','independent_20260921140624','windows_s',[1800 29000], ...
    'verifiedStatic',true,'staticEvidence',evidence);
records(2)=struct('file',fullfile(base,'20260921121352','imu.csv'), ...
    'label','independent_20260921121352','windows_s',[60 4400], ...
    'verifiedStatic',true,'staticEvidence',evidence);
end

function cfg = with_defaults(cfg,defaults)
fields = fieldnames(defaults);
for k=1:numel(fields)
    if ~isfield(cfg,fields{k}); cfg.(fields{k})=defaults.(fields{k}); end
end
end

function [TC,C,p] = check_provenance(cfg,S)
assert(isfile(cfg.tempCoeffFile)&&isfile(cfg.calibFile),'Temperature/calibration file missing.');
TC=load(cfg.tempCoeffFile); CS=load(cfg.calibFile); C=CS.result;
assert(strcmp(TC.domain,'raw') && size(TC.coef,1)==6,'Temperature coefficients must be raw-domain six-axis coefficients.');
poly=[TC.coef(:,1:end-1),zeros(6,1)];
assert(isequal(size(C.tempCoeff),size(poly)) && max(abs(C.tempCoeff-poly),[],'all')<1e-10, ...
    'Temperature coefficients differ from those bound to the calibration.');
assert(abs(C.Tref-TC.Tref)<1e-8 && isequal(logical(C.tcActive(:)),logical(TC.ord(:)>0)), ...
    'Temperature reference or active-axis configuration differs from calibration.');
assert(isequal(size(C.Ca),[3 3]) && numel(C.ba)==3 && numel(C.gyroBiasMean_deg_h)==3, ...
    'Missing calibration matrix/bias.');
assert(isequal(S.metadata.tempOrder(:),TC.ord(:)), ...
    'Reference GM temperature orders differ from current parameters.');
assert(same_path(S.cfg.tempCoeffFile,cfg.tempCoeffFile) && same_path(S.cfg.calibFile,cfg.calibFile), ...
    'Reference GM parameter-file paths differ from current files.');
p=struct('status',"CurrentBindingConsistent_HistoricalSnapshotMissing", ...
    'referenceFile',cfg.referenceMat,'currentTempFile',cfg.tempCoeffFile, ...
    'currentCalibFile',cfg.calibFile,'temperatureTrainingFile',string(TC.sourceFile), ...
    'gmTrainingFile',string(S.metadata.sourceFile), ...
    'temperatureAndGmSameRecord',same_path(TC.sourceFile,S.metadata.sourceFile), ...
    'temperatureRange_C',[TC.Tmin TC.Tmax], ...
    'note',"Original GM MAT contains parameter paths/orders but no coefficient snapshot or hash; current binding can be checked, historical identity cannot be proved.");
% This output saves current TC/C snapshots for future reproducible validation.
end

function white = load_calibrated_white(file,names)
assert(isfile(file),'Ca-calibrated Allan parameter table missing: %s',file);
T=readtable(file,'TextType','string');
assert(all(ismember({'stage','axis','arw_vrw','arw_vrw_unit'},T.Properties.VariableNames)), ...
    'Allan parameters need stage/axis/arw_vrw/arw_vrw_unit columns.');
white=nan(6,1);
for a=1:6
    i=strcmpi(T.stage,'raw+TC+calib') & strcmpi(T.axis,names(a));
    assert(nnz(i)==1,'Exactly one Ca-calibrated white-noise row is required per axis.');
    if a<=3
        assert(strcmp(T.arw_vrw_unit(i),'m/s/sqrt(h)'),'Unexpected accelerometer VRW unit.');
        white(a)=T.arw_vrw(i)/60*1000/9.80665;
    else
        assert(strcmp(T.arw_vrw_unit(i),'deg/sqrt(h)'),'Unexpected gyro ARW unit.');
        white(a)=T.arw_vrw(i)*60;
    end
end
assert(all(isfinite(white)&white>0),'Invalid calibrated white-noise parameters.');
end

function observed = load_observed_allan(cfg,names,Y,delta)
values=nan(numel(cfg.taus_s),6);
sources=strings(numel(cfg.taus_s),6);
if isfile(cfg.allanFixedTauFile)
    F=readtable(cfg.allanFixedTauFile,'VariableNamingRule','preserve','TextType','string');
    for a=1:6
        i=strcmpi(F.axis,names(a));assert(nnz(i)==1,'Fixed-tau table missing calibrated axis.');
        for k=1:numel(cfg.taus_s)
            col=sprintf('raw+TC+calib_adev_%ds',cfg.taus_s(k));
            if ismember(col,F.Properties.VariableNames)
                values(k,a)=F{i,col};sources(k,a)="TrainingCalibratedFixedTauTable";
            end
        end
    end
end
if isfile(cfg.allanCurveFile)
    T=readtable(cfg.allanCurveFile,'VariableNamingRule','preserve');
    for a=1:6
        col=char("raw+TC+calib_"+names(a)+"_adev_rad_s_or_m_s2");
        assert(ismember(col,T.Properties.VariableNames),'Calibrated Allan curve missing axis %s.',names(a));
        v=T.(col); if a<=3; v=v*1000/9.80665; else; v=v*180/pi*3600; end
        good=isfinite(T.tau_s)&T.tau_s>0&isfinite(v)&v>0;
        missing=~isfinite(values(:,a));
        values(missing,a)=exp(interp1(log(T.tau_s(good)),log(v(good)),log(cfg.taus_s(missing)),'linear',NaN));
        sources(missing,a)="TrainingCalibratedFullRateCurve_LogInterpolated";
    end
elseif ~isfile(cfg.allanFixedTauFile)
    warning('No calibrated full-rate Allan curve found; using stored-block estimates.');
end
for a=1:6
    for k=1:numel(cfg.taus_s)
        if ~isfinite(values(k,a))
            values(k,a)=overlapping_allan(Y(:,a),round(cfg.taus_s(k)/delta));
            sources(k,a)="TrainingStored10sBlockAllan_FullRateReferenceMissing";
        end
    end
end
observed=struct('values',values,'sources',sources);
end

function rows = analyze_segment(t,Y,label,kind,linear,delta,fitCfg,refSigma,refTau,cfg,names,units)
rows=struct([]); n=numel(t); span=n*delta;
for a=1:6
    original=Y(:,a); original=original(:); x=original-mean(original);
    trend=[ones(n,1),t(:)-t(1)]\original;
    if linear; x=detrend(original,1); end
    [sigma,tau,r2,q,nFit,fitEnd,maxLag,status]=fit_blocks(x,delta,fitCfg);
    sigmaRatio=sigma/refSigma(a); tauRatio=tau/refTau(a);
    within=status=="OK" && between(sigmaRatio,cfg.sigmaRatioBounds) && between(tauRatio,cfg.tauRatioBounds);
    if kind~="Independent"; independence="TrainingSensitivityOnly";
    elseif span/refTau(a)<cfg.minCorrelationTimes; independence="InsufficientDurationForReferenceTc";
    elseif status~="OK"; independence="FitFailed";
    elseif span/tau<cfg.minCorrelationTimes; independence="InsufficientDurationForFittedTc";
    elseif within && r2>=0.8; independence="LongRecordConsistent";
    else; independence="LongRecordModelMismatch";
    end
    row=struct('RecordKind',kind,'Segment',label,'Axis',names(a),'Unit',units(a), ...
        'Span_s',span,'Blocks',n,'Mean',mean(original),'BlockStd',std(original), ...
        'LinearTrendPerHour',trend(2)*3600,'DetrendMode',string(ternary(linear,'linear','constant')), ...
        'FittedSigma',sigma,'FittedTc_s',tau,'SigmaRatio',sigmaRatio,'TcRatio',tauRatio, ...
        'FitR2',r2,'InterceptRatioToLagZero',q,'InterceptAtConstraint',q>=1-1e-6, ...
        'FitPoints',nFit,'FitEnd_s',fitEnd,'MaxObservedLag_s',maxLag, ...
        'SpanOverReferenceTc',span/refTau(a),'SpanOverFittedTc',span/tau, ...
        'WithinSensitivityBounds',within,'IndependentTcStatus',independence,'FitStatus',status);
    rows=append_rows(rows,row);
end
end

function [sigma,tau,R2,q,nFit,fitEnd,maxLag,status] = fit_blocks(x,delta,cfg)
% Same fitting definition as gm_autocorrelation_analysis.m, for comparability.
sigma=NaN;tau=NaN;R2=NaN;q=NaN;nFit=0;fitEnd=NaN;maxLag=NaN;status="InsufficientBlocks";
n=numel(x); if n<4*cfg.minFitPoints; return; end
m=min([floor(cfg.maxLagSec/delta),floor(cfg.maxLagFraction*n),n-2]);
maxLag=m*delta; if m<cfg.minFitPoints; return; end
F=fft(x,2^nextpow2(2*n-1)); raw=real(ifft(F.*conj(F)));
C=raw(1:m+1)./(n-(0:m))';
if ~isfinite(C(1))||C(1)<=0; status="NonpositiveVariance";return;end
rho=C/C(1); lag=(0:m)'*delta;
last=find(rho(2:end)<=cfg.minFitRho|~isfinite(rho(2:end)),1);
if isempty(last); last=m; end
idx=(2:max(last+1,min(m+1,cfg.minFitPoints+1)))';
idx=idx(isfinite(C(idx))&C(idx)>0);nFit=numel(idx);
if nFit<cfg.minFitPoints;return;end
t=lag(idx);y=rho(idx);beta=[ones(nFit,1),t]\log(C(idx));
if ~all(isfinite(beta))||beta(2)>=0;status="NondecayingACF";return;end
q0=min(max(exp(beta(1))/C(1),1e-4),1-1e-6);
p0=[log(q0/(1-q0)),log(-1/beta(2))];w=sqrt(max(y,0.05));
objective=@(p)sum(w.*((y-logistic(p(1))*exp(-t/exp(p(2)))).^2));
op=optimset('Display','off','MaxIter',2000,'MaxFunEvals',4000,'TolX',1e-10,'TolFun',1e-12);
[p,~,exitflag]=fminsearch(objective,p0,op);
q=logistic(p(1));tau=exp(p(2));
if exitflag<=0||~isfinite(tau)||tau<=0;status="FitFailed";sigma=NaN;return;end
h=delta/tau; if abs(h)<sqrt(eps);f=1;else;f=(sinh(h/2)/(h/2))^2;end
sigma=sqrt(q*C(1)/f);fitEnd=max(t);
R2=1-sum((y-q*exp(-t/tau)).^2)/max(sum((y-mean(y)).^2),eps);
status="OK";
end

function rows = compare_allan(label,kind,Y,delta,observed,sigma,tau,white,cfg,names,units)
rows=struct([]);n=size(Y,1);
for a=1:6
    for k=1:numel(cfg.taus_s)
        ta=cfg.taus_s(k);m=round(ta/delta);
        [blockAdev,pairs]=overlapping_allan(Y(:,a),m);
        clusters=floor(n/m); adjacentPairs=max(clusters-1,0);disjointPairs=floor(n/(2*m));
        if isempty(observed);empirical=blockAdev;source="IndependentCalibrated1sBlockAllan";
        else;empirical=observed.values(k,a);source=observed.sources(k,a);end
        gm=sigma(a)*sqrt(gm_allan_factor(ta/tau(a)));
        wn=white(a)/sqrt(ta);prediction=hypot(gm,wn);ratio=prediction/empirical;
        if disjointPairs<cfg.minAllanClusters
            status="InsufficientDisjointAllanPairs";
        elseif ~isfinite(empirical)||empirical<=0
            status="InvalidEmpiricalAllan";
        else
            status="Usable";
        end
        row=struct('Record',label,'RecordKind',kind,'Axis',names(a),'Unit',units(a), ...
            'Tau_s',ta,'ModelGMAllan',gm,'ModelWhiteAllan',wn,'ModelTotalAllan',prediction, ...
            'EmpiricalAllan',empirical,'StoredBlockAllan',blockAdev,'PredictionToObserved',ratio, ...
            'OverlappingPairs',pairs,'NonoverlapAdjacentPairs',adjacentPairs, ...
            'DisjointAllanPairs',disjointPairs,'InputBlockSec',delta, ...
            'StoredBlockToPrimaryObserved',blockAdev/empirical, ...
            'WithinAllanBounds',status=="Usable"&&between(ratio,cfg.allanRatioBounds), ...
            'EvidenceStatus',status,'EmpiricalSource',source);
        rows=append_rows(rows,row);
    end
end
end

function f = gm_allan_factor(u)
% Integral of sigma^2 exp(-|lag|/Tc); Taylor expansion avoids cancellation.
if u<1e-3
    f=2*u/3-u^2/2+7*u^3/30-u^4/12+31*u^5/1260-u^6/160;
else
    f=(2*u+4*expm1(-u)-expm1(-2*u))/u^2;
end
f=max(f,0);
end

function [adev,pairs] = overlapping_allan(x,m)
n=numel(x);pairs=max(n-2*m+1,0);adev=NaN;
if pairs<1;return;end
cs=[0;cumsum(x(:)-mean(x))];means=(cs(m+1:end)-cs(1:end-m))/m;
d=means(m+1:end)-means(1:end-m);adev=sqrt(mean(d.^2)/2);
end

function [windows,info] = read_independent(r,TC,C,cfg,delta,trainingFile)
windows=struct([]);info=struct([]);
file=string(getfield_default(r,'file',''));label=string(getfield_default(r,'label','independent'));
assert(strlength(file)>0&&isfile(file),'Independent CSV missing.');
assert(~same_path(file,trainingFile) && ~same_path(file,TC.sourceFile), ...
    'The GM/temperature training record cannot be independent validation.');
assert(getfield_default(r,'verifiedStatic',false) && isfield(r,'windows_s') && ...
    size(r.windows_s,2)==2 && ~isempty(r.windows_s), ...
    'Independent windows require verifiedStatic=true and explicit fixed-pose windows_s.');
assert(isfield(r,'staticEvidence')&&strlength(string(r.staticEvidence))>0, ...
    'Record how each fixed-pose static window was verified in staticEvidence.');
range=r.windows_s;
assert(all(isfinite(range),'all')&&all(range(:,2)>range(:,1)), 'Invalid static time windows.');
assert(all(diff(range(:,1))>=0) && all(range(2:end,1)>=range(1:end-1,2)), ...
    'Static windows must be ordered and nonoverlapping.');
nr=size(range,1); sums=cell(nr,1);counts=cell(nr,1);tmin=inf(nr,1);tmax=-inf(nr,1);
for j=1:nr
    nb=ceil((range(j,2)-range(j,1))/delta);
    sums{j}=zeros(nb,6);counts{j}=zeros(nb,1);
end
vars={'time_s','ax_m_s2','ay_m_s2','az_m_s2','temp_deg_c','gx_deg_h','gy_deg_h','gz_deg_h'};
ds=tabularTextDatastore(char(file),'Delimiter',',','ReadVariableNames',true,'TextType','string');
assert(all(ismember(vars,ds.VariableNames)),'Independent CSV missing physical/time/temperature columns.');
optional=intersect({'sample','time_valid','raw_sat','ax_raw','ay_raw','az_raw','gx_raw','gy_raw','gz_raw'}, ...
    ds.VariableNames,'stable');
ds.SelectedVariableNames=[vars optional];ds.ReadSize=cfg.readSize;poly=[TC.coef(:,1:end-1),zeros(6,1)];
dtSum=0;dtCount=0;prev=NaN;
prevWinTime=nan(nr,1);prevWinSample=nan(nr,1);
chunkNo=0;
while hasdata(ds)
    D=read(ds);t=double(D.time_s);
    chunkNo=chunkNo+1;
    assert(all(isfinite(t))&&all(diff(t)>0),'Independent CSV time must be finite and increasing.');
    if isfinite(prev);assert(t(1)>prev,'Nonmonotonic chunk boundary.');dtSum=dtSum+t(1)-prev;dtCount=dtCount+1;end
    dtSum=dtSum+sum(diff(t));dtCount=dtCount+numel(t)-1;prev=t(end);
    nominalDt=median(diff(t));
    for j=1:nr
        use=t>=range(j,1)&t<range(j,2);if ~any(use);continue;end
        selectedTime=t(use);gaps=diff(selectedTime);
        if isfinite(prevWinTime(j));gaps=[selectedTime(1)-prevWinTime(j);gaps];end %#ok<AGROW>
        assert(all(gaps<=1.5*nominalDt),'Sample time gap exceeds 1.5 nominal intervals inside selected window.');
        prevWinTime(j)=selectedTime(end);
        if ismember('sample',optional)
            sample=double(D.sample(use));steps=diff(sample);
            if isfinite(prevWinSample(j));steps=[sample(1)-prevWinSample(j);steps];end %#ok<AGROW>
            assert(all(isfinite(sample))&&all(steps==1),'Sample IDs are discontinuous inside selected static window.');
            prevWinSample(j)=sample(end);
        end
        if ismember('time_valid',optional)
            assert(all(double(D.time_valid(use))==1),'Invalid time flag inside selected static window.');
        end
        if ismember('raw_sat',optional)
            assert(all(double(D.raw_sat(use))==0),'Raw saturation flag inside selected static window.');
        end
        for rawName={'ax_raw','ay_raw','az_raw','gx_raw','gy_raw','gz_raw'}
            if ismember(rawName{1},optional)
                rawValues=double(D.(rawName{1})(use));
                assert(all(isfinite(rawValues))&&all(abs(rawValues)<32760), ...
                    'Raw IMU samples approach saturation inside selected static window.');
            end
        end
        temp=double(D.temp_deg_c(use));raw=[double(D.ax_m_s2(use)),double(D.ay_m_s2(use)), ...
            double(D.az_m_s2(use)),double(D.gx_deg_h(use)),double(D.gy_deg_h(use)),double(D.gz_deg_h(use))];
        assert(all(isfinite(temp))&&all(isfinite(raw),'all'),'Nonfinite samples in selected static window.');
        tmin(j)=min(tmin(j),min(temp));tmax(j)=max(tmax(j),max(temp));
        assert(all(temp>=TC.Tmin-1e-9 & temp<=TC.Tmax+1e-9), ...
            'Independent window is outside the fitted temperature range; extrapolation is forbidden.');
        drift=zeros(size(raw));
        for a=1:6;if TC.ord(a)>0;drift(:,a)=polyval(poly(a,:),temp-TC.Tref);end;end
        Y=raw-drift;Y(:,1:3)=(C.Ca*(Y(:,1:3)-C.ba(:)').').';
        Y(:,4:6)=Y(:,4:6)-C.gyroBiasMean_deg_h(:)';
        bin=floor((t(use)-range(j,1))/delta)+1;nb=numel(counts{j});
        for a=1:6;sums{j}(:,a)=sums{j}(:,a)+accumarray(bin,Y(:,a),[nb,1],@sum,0);end
        counts{j}=counts{j}+accumarray(bin,1,[nb,1],@sum,0);
    end
    if chunkNo==1||mod(chunkNo,5)==0
        fprintf('  %s: scanned %.2f h of CSV time\n',label,t(end)/3600);
    end
    if t(end)>=max(range(:,2));break;end
end
assert(dtCount>0&&dtSum>0,'No usable time intervals.');fs=dtCount/dtSum;
for j=1:nr
    keep=counts{j}>=S_min_coverage(cfg)*delta*fs;
    tt=range(j,1)+((1:numel(keep))'-.5)*delta;
    ii=find(keep);
    if isempty(ii)
        info=append_rows(info,record_row(label+"_window"+j,file,"InsufficientBlocks",0, ...
            [tmin(j) tmax(j)],"No sufficiently populated blocks in verified static window."));continue;
    end
    tt=tt(ii);Y=sums{j}(ii,:)./counts{j}(ii);run=longest_run(tt,delta);tt=tt(run);Y=Y(run,:);
    windowName=label+"_window"+j;
    w=struct('name',windowName,'kind',"Independent",'time',tt,'values',to_engineering(Y), ...
        'file',file,'status',"VerifiedFixedPoseWindow",'blockSec',delta);
    windows=append_rows(windows,w);
    detail=string(r.staticEvidence)+"; longest continuous adequately populated block run used.";
    info=append_rows(info,record_row(windowName,file,"VerifiedFixedPoseWindow",numel(tt)*delta, ...
        [tmin(j) tmax(j)],detail));
end
end

function T = synthetic_sanity()
% Deterministic checks: cancellation, constant input, linear drift, white
% averaging and stationary OU Allan integral. Preserve caller RNG state.
old=rng;cleanup=onCleanup(@()rng(old));
rng(73129,'twister');rows=struct([]);
u=1e-12;f=gm_allan_factor(u);target=2*u/3;
rows=append_rows(rows,struct('Check',"GM_small_u_limit",'Value',f, ...
    'Expected',target,'RelativeError',abs(f/target-1),'Passed',abs(f/target-1)<1e-8));
f=gm_allan_factor(.001);series=2*.001/3-.001^2/2+7*.001^3/30-.001^4/12;
rows=append_rows(rows,struct('Check',"GM_series_formula_boundary",'Value',f, ...
    'Expected',series,'RelativeError',abs(f/series-1),'Passed',abs(f/series-1)<1e-6));
c=overlapping_allan(ones(1000,1)*7,30);
rows=append_rows(rows,struct('Check',"Constant_has_zero_Allan",'Value',c, ...
    'Expected',0,'RelativeError',abs(c),'Passed',abs(c)<1e-12));
m=30;slope=.003;linear=overlapping_allan(slope*(0:1999)',m);target=slope*m/sqrt(2);
rows=append_rows(rows,struct('Check',"Linear_drift_Allan",'Value',linear, ...
    'Expected',target,'RelativeError',abs(linear/target-1),'Passed',abs(linear/target-1)<1e-8));
white=overlapping_allan(randn(100000,1),m);target=1/sqrt(m);
rows=append_rows(rows,struct('Check',"White_Allan_units",'Value',white, ...
    'Expected',target,'RelativeError',abs(white/target-1),'Passed',abs(white/target-1)<.1));
sigma=2;tau=300;phi=exp(-1/tau);q=sigma^2*(-expm1(-2/tau));
x=filter(sqrt(q),[1 -phi],randn(100000,1),phi*sigma*randn);
measured=overlapping_allan(x,m);target=sigma*sqrt(gm_allan_factor(m/tau));
rows=append_rows(rows,struct('Check',"Stationary_OU_Allan",'Value',measured, ...
    'Expected',target,'RelativeError',abs(measured/target-1),'Passed',abs(measured/target-1)<.15));
T=struct2table(rows);
end

function f = S_min_coverage(cfg)
f=getfield_default(cfg,'minBlockCoverage',0.8);
end

function ii = longest_run(t,delta)
assert(~isempty(t),'Empty series.');
cut=[1;find(abs(diff(t)-delta)>max(1e-6,delta*1e-6))+1;numel(t)+1];
[~,j]=max(diff(cut));ii=cut(j):cut(j+1)-1;
end

function Y = to_engineering(Y)
Y(:,1:3)=Y(:,1:3)*1000/9.80665;
end

function row = record_row(label,file,status,span,temp,note)
row=struct('Record',string(label),'File',string(file),'Status',string(status), ...
    'Span_s',span,'MinTemperature_C',temp(1),'MaxTemperature_C',temp(2),'Evidence',string(note));
end

function make_plots(R,sigma,tau,names,units,delta)
cfg=R.cfg;T=R.allan;
fig=figure('Color','w','Visible',cfg.figureVisible,'Position',[80 80 1320 760]);
tl=tiledlayout(fig,2,3,'TileSpacing','compact');
for a=1:6
    nexttile(tl);hold on;i=T.Axis==names(a)&T.RecordKind=="Training";
    plotted=[T.EmpiricalAllan(i);T.ModelTotalAllan(i)];
    loglog(T.Tau_s(i),T.EmpiricalAllan(i),'ko-','LineWidth',1.4,'DisplayName','Training (TC + calibration)');
    loglog(T.Tau_s(i),T.ModelTotalAllan(i),'b--','LineWidth',1.5,'DisplayName','GM + calibrated white');
    j=T.Axis==names(a)&T.RecordKind=="Independent";
    records=unique(T.Record(j),'stable');
    for k=1:numel(records)
        sel=j&T.Record==records(k)&T.EvidenceStatus=="Usable";
        plotted=[plotted;T.EmpiricalAllan(sel)]; %#ok<AGROW>
        source=find(string({R.series.name})==records(k),1);
        span=numel(R.series(source).time)*R.series(source).blockSec;
        if span>=2*3600;lengthLabel='long';else;lengthLabel='short';end
        displayName=sprintf('Independent %s %.2f h',lengthLabel,span/3600);
        loglog(T.Tau_s(sel),T.EmpiricalAllan(sel),'s-','DisplayName',displayName);
    end
    set(gca,'XScale','log','YScale','log');grid on;box on;
    plotted=plotted(isfinite(plotted)&plotted>0);
    if ~isempty(plotted);ylim([min(plotted)*.85,max(plotted)*1.18]);end
    xlim([min(cfg.taus_s)*.85,max(cfg.taus_s)*1.15]);
    xlabel('Averaging time (s)');ylabel("Allan deviation ("+units(a)+")");title(names(a));
    if a==1
        lgd=legend('Orientation','horizontal','NumColumns',4,'FontSize',9,'Interpreter','none');
        lgd.Layout.Tile='south';
    end
end
title(tl,'GM validation at 30-600 s: training comparison is not independent validation');
exportgraphics(fig,fullfile(cfg.outDir,'gm_validation_allan.png'),'Resolution',180);
if cfg.saveFig;savefig(fig,fullfile(cfg.outDir,'gm_validation_allan.fig'));end
if strcmp(cfg.figureVisible,'off');close(fig);end

fig=figure('Color','w','Visible',cfg.figureVisible,'Position',[100 90 1320 760]);
tl=tiledlayout(fig,2,3,'TileSpacing','compact');P=R.segments;
for a=1:6
    nexttile(tl);hold on;i=P.Axis==names(a)&P.RecordKind=="TrainingSensitivity";
    yyaxis left;plot(P.SigmaRatio(i),'bo-','LineWidth',1.2);ylabel('Sigma / reference sigma');
    yline(1,'k:');
    yyaxis right;plot(P.TcRatio(i),'rs-','LineWidth',1.2);ylabel('Tc / reference Tc');yline(1,'k:');
    xticks(1:nnz(i));xticklabels(strrep(P.Segment(i),'_',' '));xtickangle(35);grid on;box on;
    set(gca,'TickLabelInterpreter','none');
    title(sprintf('%s: reference sigma=%.4g %s, Tc=%.0f s',names(a),sigma(a),units(a),tau(a)));
end
title(tl,'Split-record and trend sensitivity (not independent validation)');
exportgraphics(fig,fullfile(cfg.outDir,'gm_validation_sensitivity.png'),'Resolution',180);
if cfg.saveFig;savefig(fig,fullfile(cfg.outDir,'gm_validation_sensitivity.fig'));end
if strcmp(cfg.figureVisible,'off');close(fig);end

fig=figure('Color','w','Visible',cfg.figureVisible,'Position',[120 100 1320 760]);
tl=tiledlayout(fig,2,3,'TileSpacing','compact');Y=R.series(1).values;t=R.series(1).time;
for a=1:6
    nexttile(tl);hold on;x=Y(:,a)-mean(Y(:,a));plot((t-t(1))/3600,x,'LineWidth',.8);
    p=[ones(numel(t),1),t-t(1)]\x;plot((t-t(1))/3600,[ones(numel(t),1),t-t(1)]*p,'k--','LineWidth',1.5);
    for part=1:3;xline(part*numel(t)*delta/4/3600,'k:');end
    grid on;box on;xlabel('Time (h)');ylabel("Mean-removed residual ("+units(a)+")");title(names(a));
end
title(tl,'Stored training residuals, linear trend, and quarter boundaries');
exportgraphics(fig,fullfile(cfg.outDir,'gm_validation_training_series.png'),'Resolution',180);
if cfg.saveFig;savefig(fig,fullfile(cfg.outDir,'gm_validation_training_series.fig'));end
if strcmp(cfg.figureVisible,'off');close(fig);end
end

function tf = between(x,bounds)
tf=isfinite(x)&&x>=bounds(1)&&x<=bounds(2);
end
function q = logistic(x)
if x>=0;q=1/(1+exp(-x));else;ex=exp(x);q=ex/(1+ex);end
end
function tf = same_path(a,b)
tf=strcmpi(strrep(char(a),'\','/'),strrep(char(b),'\','/'));
end
function out = append_rows(out,rows)
if isempty(out);out=rows;elseif ~isempty(rows);out=[out(:);rows(:)];end
end
function value = getfield_default(s,field,fallback)
if isfield(s,field);value=s.(field);else;value=fallback;end
end
function value = ternary(condition,a,b)
if condition;value=a;else;value=b;end
end
function value = finite_min(x)
x=x(isfinite(x));if isempty(x);value=NaN;else;value=min(x);end
end
function value = finite_max(x)
x=x(isfinite(x));if isempty(x);value=NaN;else;value=max(x);end
end
