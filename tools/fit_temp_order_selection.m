%% 原始 21h 静态数据逐轴选阶；输出供下一阶段自动加载的 MAT。
% 判据：1~5 阶中，相邻阶训练残差 RMS 改善不足 0.1% 时停止。
% BIC 与 600s 块均值只作同数据集参考，不代表样本外验证。
clear; clc; close all;
projRoot = fileparts(fileparts(mfilename('fullpath')));
sourceFile = fullfile(projRoot,'data','decoded','20260926005735','imu.csv');
outDir = fullfile(projRoot,'data','calib24');
if ~exist(outDir,'dir'); mkdir(outDir); end
decim = 50; blockSec = 600; orders = 1:5;
cfg.improveThreshold = 0.1;
axNames = {'ax','ay','az','gx','gy','gz'};
axUnit = {'mg','mg','mg','deg/h','deg/h','deg/h'};
axScale = [1000/9.80665,1000/9.80665,1000/9.80665,1,1,1];
fprintf('读取原始数据: %s\n',sourceFile);
M = readmatrix(sourceFile,'NumHeaderLines',1);
valid = all(isfinite(M(:,[1 6 7 15:21])),2);
assert(all(valid),'原始数据含非有限值，请先审查，不能无声拼接数据');
temp = M(:,18); RAW = M(:,[15:17 19:21]);
Nfull = size(M,1);
sourceTimeRange = [M(1,6),M(end,6)];
assert(all(diff(M(:,6))>0),'时间必须严格递增');
sampleRate = 1/median(diff(M(:,6)));
sourceInfo = dir(sourceFile); sourceBytes = sourceInfo.bytes;
clear M;
fitIdx = 1:decim:Nfull;
Tref = temp(1); Tmin = min(temp); Tmax = max(temp);
dT = temp(fitIdx)-Tref; dTa = temp-Tref; N = numel(dT);
block = round(blockSec*sampleRate); nb = floor(Nfull/block);
fprintf('样本=%d, 拟合=%d, Tref=%.8f, 温区=[%.4f,%.4f]\n',Nfull,N,Tref,Tmin,Tmax);
residStd = nan(6,5); residImp = nan(6,5);
bicAll = nan(6,5); aicAll = nan(6,5);
driftPp = nan(6,5); blockStd = nan(6,5);
for k=1:6
    y = RAW(fitIdx,k);
    for oi=1:numel(orders)
        o = orders(oi);
        X = dT.^(o:-1:0);
        p = X\y; r = y-X*p; rss = r'*r;
        residStd(k,oi) = sqrt(rss/N)*axScale(k);
        bicAll(k,oi) = N*log(rss/N)+(o+1)*log(N);
        aicAll(k,oi) = N*log(rss/N)+2*(o+1);
        critical = roots(polyder(p'));
        critical = real(critical(abs(imag(critical))<1e-10));
        critical = critical(critical>=Tmin-Tref & critical<=Tmax-Tref);
        evalDt = [Tmin-Tref;Tmax-Tref;critical(:)];
        values = polyval(p,evalDt);
        driftPp(k,oi) = (max(values)-min(values))*axScale(k);
        residual = RAW(:,k)-polyval(p,dTa);
        means = mean(reshape(residual(1:nb*block),block,nb),1);
        blockStd(k,oi) = std(means)*axScale(k);
        if oi>1
            residImp(k,oi) = 100*(1-residStd(k,oi)/residStd(k,oi-1));
        end
    end
    fprintf('%s: residual RMS %s\n',axNames{k},mat2str(residStd(k,:),6));
end
bestOrder = zeros(6,1); bestOrderBIC = zeros(6,1); bestOrderBlk = zeros(6,1);
for k=1:6
    bestOrder(k) = orders(end);
    for oi=1:numel(orders)-1
        if residImp(k,oi+1)<cfg.improveThreshold
            bestOrder(k)=orders(oi); break;
        end
    end
    [~,i]=min(bicAll(k,:)); bestOrderBIC(k)=orders(i);
    [~,i]=min(blockStd(k,:)); bestOrderBlk(k)=orders(i);
end
save(fullfile(outDir,'temp_order_selection.mat'),'bestOrder','bestOrderBIC', ...
    'bestOrderBlk','orders','decim','Tref','Tmin','Tmax','Nfull','sourceFile', ...
    'sourceBytes','sourceTimeRange','cfg','axNames');
axisCol = repelem(string(axNames(:)),numel(orders));
orderCol = repmat(orders(:),6,1);
Tcsv = table(axisCol,orderCol,reshape(residStd',[],1),reshape(residImp',[],1), ...
    reshape(bicAll',[],1),reshape(aicAll',[],1),reshape(driftPp',[],1), ...
    reshape(blockStd',[],1),'VariableNames',{'axis','order','resid_std', ...
    'resid_improve_percent','BIC','AIC','drift_peak_to_peak','block_mean_std_after'});
writetable(Tcsv,fullfile(outDir,'temp_order_selection.csv'));
fig=figure('Position',[60 60 1400 760]);
for k=1:6
    subplot(2,3,k);
    yyaxis left;
    plot(orders,residStd(k,:),'o-'); ylabel(['Residual RMS (' axUnit{k} ')']);
    yyaxis right;
    plot(orders,bicAll(k,:),'s--'); ylabel('BIC');
    xlabel('Polynomial order'); grid on;
    title(sprintf('%s: selected order %d',axNames{k},bestOrder(k)));
end
sgtitle('Raw-domain temperature model order selection (in-sample)');
saveas(fig,fullfile(outDir,'fig_temp_order_selection.png'));
fid=fopen(fullfile(outDir,'温度模型阶数选择报告.md'),'w','n','UTF-8');
assert(fid>=0,'无法写入报告');
closer=onCleanup(@()fclose(fid));
fprintf(fid,'# 原始域逐轴温补阶数选择\n\n');
fprintf(fid,'- 数据：%s\n- 总样本：%d；每 %d 点取 1 点拟合。\n',sourceFile,Nfull,decim);
fprintf(fid,'- Tref：%.10f °C；有效温区：%.4f~%.4f °C。\n',Tref,Tmin,Tmax);
fprintf(fid,'- 相邻阶训练残差改善阈值：%.2f%%。\n\n',cfg.improveThreshold);
fprintf(fid,'| 轴 | 选定阶数 | BIC 最优 | 600s 块均值最优 | 残差 RMS | 单位 |\n');
fprintf(fid,'|---|---:|---:|---:|---:|---|\n');
for k=1:6
    fprintf(fid,'| %s | %d | %d | %d | %.7g | %s |\n',axNames{k},bestOrder(k), ...
        bestOrderBIC(k),bestOrderBlk(k),residStd(k,bestOrder(k)),axUnit{k});
end
fprintf(fid,'\n上述指标均来自同一 21h 数据，不能视为独立泛化验证。\n');
fprintf(fid,'一阶仍补偿 c1*dT；禁用某轴须明确置零全部温度项并重新标定。\n');
fprintf(fid,'漂移峰峰值包含区间内部极值；加计 mg 使用标准重力 9.80665 m/s²。\n');
fprintf(fid,'不得无告警外推温区。24 位置陀螺离散还包含地球自转投影和重复定位误差。\n');
fprintf(fid,'下一步 fit_temp_bias_raw.m 读取本次 MAT 选阶结果作参考（实际配置以 cfg.axisOrder 为准）。\n');
clear closer;
fprintf('Selected orders: %s\n',mat2str(bestOrder'));
