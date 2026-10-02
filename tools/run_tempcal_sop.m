function run_tempcal_sop(varargin)
%RUN_TEMPCAL_SOP 一键跑通「MPU6050 温补 + 标定 SOP」的步骤①与②。
%
% 处理顺序（详见 docs/温补标定SOP.md，不得颠倒、不得跳步）：
%   ① fit_temp_order_selection.m          逐轴 1~5 阶选阶
%   ① fit_temp_bias_raw.m           在原始域拟合温补系数矩阵
%   ② calib24_static_numbered_tempcomp.m  按系数矩阵有效轴温补后做 24 位置标定
%
% 用法（在仓库根目录，或用绝对路径调用）：
%   run_tempcal_sop                        % 全流程
%   run_tempcal_sop('skipStep1',true)      % 复用已有系数，只重跑步骤②
%   run_tempcal_sop('skipCalib',true)      % 只重算步骤①
%
% 注意：
%   三个被调用脚本均以 clear/clc/close all 开头，会清空调用者工作区；
%   因此本函数把所有跨步骤状态放在根对象 appdata 中（clear 不会清除 appdata）。
%
%   步骤③（三方案 Allan 对比）是 Python 脚本，须在命令行单独运行——
%   命令见本函数结尾输出与 docs/温补标定SOP.md §4。
%
% 输出：data/calib24/ 下的 temp_order_selection.*、temp_coeffs_raw.*、
%       calib24_result_tempcomp_<outTag>.mat 及其配套 CSV/图。

%% ---------------- 选项 ----------------
p = inputParser;
p.addParameter('skipStep1', false, @(x)islogical(x) || isnumeric(x));
p.addParameter('skipCalib', false, @(x)islogical(x) || isnumeric(x));
p.parse(varargin{:});
o = p.Results;

%% ---------------- 状态写入 appdata（跨 clear 保存）----------------
S.projRoot  = fileparts(fileparts(mfilename('fullpath')));
S.toolsDir  = fullfile(S.projRoot, 'tools');
S.calibDir  = fullfile(S.projRoot, 'data', 'calib24');
S.oldDir    = pwd;
S.skipStep1 = logical(o.skipStep1);
S.skipCalib = logical(o.skipCalib);
S.timer     = tic;
S.step1a    = fullfile(S.toolsDir, 'fit_temp_order_selection.m');
S.step1b    = fullfile(S.toolsDir, 'fit_temp_bias_raw.m');
S.step2     = fullfile(S.toolsDir, 'calib24_static_numbered_tempcomp.m');
setappdata(0, 'tempcal_sop_state', S);

%% ---------------- 启动信息 ----------------
fprintf('============ 温补标定 SOP 一键运行 ============\n');
fprintf('项目根目录 : %s\n', S.projRoot);
fprintf('步骤① 选阶 : %s\n', tern(S.skipStep1, '跳过', '执行'));
fprintf('步骤① 系数 : %s\n', tern(S.skipStep1, '跳过', '执行'));
fprintf('步骤② 标定 : %s\n', tern(S.skipCalib, '跳过', '执行'));

%% ---------------- 步骤 ① ----------------
S = getappdata(0, 'tempcal_sop_state');
if ~S.skipStep1
    assert(isfile(S.step1a) && isfile(S.step1b), '找不到步骤①脚本，请检查 tools 目录');

    fprintf('\n>>> 步骤①a 逐轴选阶\n');
    cd(S.projRoot);
    run(S.step1a);
    S = getappdata(0, 'tempcal_sop_state');   % run 内的 clear 会清工作区，须重新取回
    assert(isfile(fullfile(S.calibDir, 'temp_order_selection.mat')), ...
           '选阶未产出 temp_order_selection.mat');

    fprintf('\n>>> 步骤①b 拟合原始域温补系数\n');
    cd(S.projRoot);
    run(S.step1b);
    S = getappdata(0, 'tempcal_sop_state');
    assert(isfile(fullfile(S.calibDir, 'temp_coeffs_raw.mat')), ...
           '拟合未产出 temp_coeffs_raw.mat');
else
    S = getappdata(0, 'tempcal_sop_state');
    cf = fullfile(S.calibDir, 'temp_coeffs_raw.mat');
    assert(isfile(cf), 'skipStep1=true 但找不到已有 %s，请先跑步骤①', cf);
    fprintf('\n[跳过步骤①] 复用 %s\n', cf);
end

%% ---------------- 步骤 ② ----------------
S = getappdata(0, 'tempcal_sop_state');
if ~S.skipCalib
    assert(isfile(S.step2), '找不到步骤②脚本 calib24_static_numbered_tempcomp.m');
    fprintf('\n>>> 步骤② 按轴温补 + 24 位置静态标定\n');
    cd(S.projRoot);
    run(S.step2);
else
    fprintf('\n[跳过步骤②]\n');
end

%% ---------------- 收尾 ----------------
S = getappdata(0, 'tempcal_sop_state');
cd(S.oldDir);
elapsed = toc(S.timer);
rmappdata(0, 'tempcal_sop_state');

%% ---------------- 汇总 ----------------
fprintf('\n============ 完成，用时 %.1f s ============\n', elapsed);

selFile = fullfile(S.calibDir, 'temp_order_selection.mat');
if isfile(selFile)
    sel = load(selFile, 'bestOrder', 'orders');
    fprintf('选阶脚本参考 [ax ay az gx gy gz] = %s（候选 1~%d）\n', ...
            mat2str(sel.bestOrder(:)'), max(sel.orders));
end
coeffFile = fullfile(S.calibDir, 'temp_coeffs_raw.mat');
if isfile(coeffFile)
    fitted = load(coeffFile, 'ord');
    fprintf('实际应用阶数 [ax ay az gx gy gz] = %s（0 表示关闭）\n', ...
            mat2str(fitted.ord(:)'));
    fprintf('温度系数   : %s\n', coeffFile);
end

fprintf(['\n下一步（步骤③ 三方案 Allan 对比，在仓库根目录用命令行运行）：\n' ...
         '  .\\.venv-temp\\Scripts\\python.exe tools\\allan_compare_tc_configs.py ^\n' ...
         '      data\\decoded\\<session>\\imu.csv ^\n' ...
         '      --coeff data\\calib24\\temp_coeffs_raw.csv ^\n' ...
         '      --calib data\\calib24\\calib24_result_tempcomp_<tag>.mat ^\n' ...
         '      --enable <a,b,c,d,e,f>\n']);
fprintf('（--calib 的 <tag> 必须等于步骤② outTag；--enable 必须等于系数矩阵 ord>0）\n');
end

function s = tern(cond, a, b)
if cond
    s = a;
else
    s = b;
end
end
