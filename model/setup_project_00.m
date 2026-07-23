%% 00_setup_project.m  — tidy, no local functions
clc; close all force;
projDir = fileparts(mfilename('fullpath'));
cd(projDir);
addpath(genpath(projDir));   % includes utils etc.

mdl = 'Simple_powertrain_test1';

% Load model (no force-update to avoid mask init)
if ~bdIsLoaded(mdl), load_system(mdl); end

%% Find the RL Agent block and disable it if no valid agent exists
rlBlk = '';
allBlk = find_system(mdl,'LookUnderMasks','all','FollowLinks','on');
for k = 1:numel(allBlk)
    try
        dp = get_param(allBlk{k},'DialogParameters');
        if isstruct(dp) && isfield(dp,'Agent')
            rlBlk = allBlk{k}; break
        end
    end
end
if ~isempty(rlBlk)
    agentName = get_param(rlBlk,'Agent');   % e.g. 'agentObj'
    shouldComment = true;
    if ~isempty(agentName)
        try
            exists = evalin('base',sprintf('exist(''%s'',''var'')==1',agentName));
            if exists
                ao = evalin('base',agentName);
                if isobject(ao) && isscalar(ao)
                    shouldComment = false;   % looks valid → leave enabled
                end
            end
        catch, shouldComment = true;
        end
    end
    set_param(rlBlk,'Commented', ternary(shouldComment,'on','off'));
end

open_system(mdl);

%% ===== Sim settings (base) =====
Simulation_time = 50;    % keep for your drive-cycle work
Ts              = 0.05;
set_param(mdl,'StopTime',num2str(Simulation_time));

%% ===== Plant params =====
Vehicle_mass    = 1200;
Wheel_radius    = 0.30;
Drag_coeff      = 0.32;
Frontal_area    = 2.2;
Rolling_res     = 0.015;
Gear_ratio      = 9.5;
Gear_efficiency = 0.97;

Motor_voltage        = 400;     % V
Motor_no_load_speed  = 4000;   % rpm
Motor_rated_speed    = 3000;   % rpm
Motor_rated_load     = 1200;   % W

%% Push into MODEL WORKSPACE (persistent with model)
mw = get_param(mdl,'ModelWorkspace');
assignin(mw,'Simulation_time',Simulation_time);
assignin(mw,'Ts',Ts);

assignin(mw,'Vehicle_mass',Vehicle_mass);
assignin(mw,'Wheel_radius',Wheel_radius);
assignin(mw,'Drag_coeff',Drag_coeff);
assignin(mw,'Frontal_area',Frontal_area);
assignin(mw,'Rolling_res',Rolling_res);

assignin(mw,'Gear_ratio',Gear_ratio);
assignin(mw,'Gear_efficiency',Gear_efficiency);

assignin(mw,'Motor_voltage',Motor_voltage);
assignin(mw,'Motor_no_load_speed',Motor_no_load_speed);
assignin(mw,'Motor_rated_speed',Motor_rated_speed);
assignin(mw,'Motor_rated_load',Motor_rated_load);

% Persist depending on data source
ds = mw.DataSource;
switch ds
    case 'Model File'
        save_system(mdl);
    case {'MAT-File','MATLAB File'}
        saveToSource(mw);
    otherwise
        warning('ModelWorkspace DataSource is "%s". Falling back to save_system.', ds);
        save_system(mdl);
end

%% Hygiene
try, set_param([mdl '/Math Function'],'Operator','square'); end
set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout');

fprintf('✅ Setup base params done. RL block kept %s.\n', ...
    iff(~isempty(rlBlk) && strcmp(get_param(rlBlk,'Commented'),'off'), 'ENABLED','DISABLED'));

%% ===== Step-up/down experiment defaults (for quick RL) =====
% (Drive-cycle path kept below, commented out)
StepEpisode_StopTime = 10;    % seconds
Step_Amp             = 10;    % m/s
assignin('base','Amp',Step_Amp);

% Use the short episode for step tests (comment to keep 50 s)
set_param(mdl,'StopTime',num2str(StepEpisode_StopTime));

% Expected block names:
stepUpPath   = [mdl '/StepUp'];     % Step block, t=1, Final= Amp
stepDownPath = [mdl '/StepDown'];   % Step block, t=6, Final=-Amp
satPath      = [mdl '/Sat_vref'];   % Saturation (optional)
vrefTW       = [mdl '/v_ref_ts'];   % To Workspace (Timeseries)

% Configure Step blocks if present
if ~isempty(find_system(mdl,'SearchDepth',1,'Name','StepUp'))
    try, set_param(stepUpPath,  'Time','1','Before','0','After','Amp'); end
else
    warning('StepUp block not found at top level. Create "%s".', stepUpPath);
end
if ~isempty(find_system(mdl,'SearchDepth',1,'Name','StepDown'))
    try, set_param(stepDownPath,'Time','6','Before','0','After','-Amp'); end
else
    warning('StepDown block not found at top level. Create "%s".', stepDownPath);
end

% Optional saturation on v_ref
if ~isempty(find_system(mdl,'SearchDepth',1,'Name','Sat_vref'))
    try, set_param(satPath,'UpperLimit','Amp','LowerLimit','0'); end
end

% Ensure v_ref_ts uses Timeseries
if ~isempty(find_system(mdl,'SearchDepth',1,'Name','v_ref_ts'))
    try, set_param(vrefTW,'SaveFormat','Timeseries'); end
end

disp('✅ Step experiment ready (Amp=10 m/s, StopTime=10 s).');

%% ===== Drive-cycle helper (leave commented until you switch back) =====
% % Example: switch your source back to Drive Cycle input (adjust paths):
% % set_param([mdl '/Multiport Switch'],'DataPortOrder','Zero-based contiguous','Inputs','2');
% % set_param(mdl,'StopTime','50');

%% ---- Small inline helpers (no local function definitions) ----
function out = ternary(cond,a,b)
% “inline” ternary: works inside script without creating a separate local function
if cond, out = a; else, out = b; end
end
function out = iff(cond,a,b), out = ternary(cond,a,b); end