%% N_Setup_project_00.m — robust solver + params + diagnostics
clc; close all force;

mdl = 'Simple_powertrain_test1';
if ~bdIsLoaded(mdl), load_system(mdl); end

%% --------- SOLVER: Fixed-step ODE (works with continuous states) ----------
% NOTE: 'ode3' and 'ode4' are fixed-step ODE solvers. Do NOT use FixedStepDiscrete
% when your plant has continuous states.
try
    set_param(mdl, ...
        'SolverType','Fixed-step', ...
        'Solver','ode3', ...            % Bogacki–Shampine (good balance)
        'FixedStep','0.001', ...        % 1 ms
        'MaxStep','auto', ...
        'AutoInsertRateTranBlk','on');  % help when rates differ
catch ME
    warning('Could not set preferred fixed-step ODE solver (ode3, 1 ms): %s', ME.message);
    try
        set_param(mdl, 'SolverType','Fixed-step', 'Solver','ode4', 'FixedStep','0.001', 'AutoInsertRateTranBlk','on');
    catch ME2
        error('Failed to set a fixed-step ODE solver: %s', ME2.message);
    end
end

%% --------- RL block: enable only if a valid agent is in base ----------
rlBlk = '';
allBlk = find_system(mdl,'LookUnderMasks','all','FollowLinks','on');
for k = 1:numel(allBlk)
    try
        dp = get_param(allBlk{k},'DialogParameters');
        if isstruct(dp) && isfield(dp,'Agent'), rlBlk = allBlk{k}; break; end
    catch, end
end
if ~isempty(rlBlk)
    agName = get_param(rlBlk,'Agent');     % e.g. 'agentObj'
    enableIt = false;
    if ~isempty(agName)
        try
            inBase = evalin('base',sprintf('exist(''%s'',''var'')==1',agName));
            if inBase
                ao = evalin('base',agName);
                enableIt = isobject(ao) && isscalar(ao);
            end
        catch, enableIt = false;
        end
    end
    set_param(rlBlk,'Commented', ternary(~enableIt,'on','off'));
end

%% --------- Stop time ----------
Simulation_time = 40;        % keep consistent with your powertrain runs
set_param(mdl,'StopTime',num2str(Simulation_time));

%% --------- Push plant parameters to MODEL WORKSPACE ----------
mw = get_param(mdl,'ModelWorkspace');

% Vehicle / environment
assignin(mw,'Vehicle_mass',         2000);
assignin(mw,'Vehicle_Wheel_Radius', 0.358);
assignin(mw,'C_Aero_drag',          0.28);
assignin(mw,'Vehicle_frontal_area', 2.56);
assignin(mw,'C_rolling_resitance',  0.01);
assignin(mw,'C_viscous_damping',    0.02);
assignin(mw,'Slope',                0);        % deg
assignin(mw,'Damping_coef',         50);       % N*m*s/rad
assignin(mw,'Gear_ratio',           10);

% Motor (your new block may only use a subset—safe to assign all)
assignin(mw,'Motor_voltage',        450);      % V (assumed bus)
assignin(mw,'Motor_no_load_speed',  16000);    % rpm
assignin(mw,'Motor_rated_speed',    9000);     % rpm
assignin(mw,'Motor_rated_load',     150e3);    % W (150 kW)
assignin(mw,'Motor_rotor_inertia',  0.05);     % kg*m^2
assignin(mw,'Motor_rotor_damping',  0.02);     % N*m*s/rad

% Persist
switch mw.DataSource
    case 'Model File'
        save_system(mdl);
    case {'MAT-File','MATLAB File'}
        saveToSource(mw);
    otherwise
        save_system(mdl);
end

%% --------- QUICK COMPILE & DIAGNOSTICS ----------
% Compile to get compiled dims/sample-times, then stop.
set_param(mdl,'SimulationCommand','update');

% 1) Solver summary
solvType = get_param(mdl,'SolverType');
solver   = get_param(mdl,'Solver');
fxStep   = get_param(mdl,'FixedStep');
mxStep   = get_param(mdl,'MaxStep');

fprintf('\n=== SOLVER ===\nType=%s | Solver=%s | FixedStep=%s | MaxStep=%s\n', ...
    solvType, solver, fxStep, mxStep);

% 2) Sizes / state info (text summary)
try
    szTxt = sldiagnostics(mdl,'Sizes');
    fprintf('%s\n', szTxt);
catch
    fprintf('Sizes diagnostic not available in this release.\n');
end

% 3) Sample-time table (compact)
try
    ST = Simulink.BlockDiagram.getSampleTimes(mdl);
    fprintf('--- Sample Times ---\n');
    for i=1:numel(ST)
        if isfield(ST(i),'PeriodAndOffset')
            po = ST(i).PeriodAndOffset;
            if isnumeric(po), poStr = sprintf('[%g %g]', po); else, poStr = char(po); end
        else
            poStr = 'n/a';
        end
        fprintf('%2d) %-12s  Src: %-50s  Period/Offset: %s\n', ...
            i, ST(i).Type, truncate(ST(i).Source), poStr);
    end
catch
    fprintf('Sample-time table not available in this release.\n');
end

% 4) Check important To-Workspace taps for width (helps catch vector signals)
checkTWWidth(mdl,'Veh_ref');     % your new ref
checkTWWidth(mdl,'v_ref_ts');    % legacy ref name, if present
checkTWWidth(mdl,'v_ts');        % actual speed

% End compile
set_param(mdl,'SimulationCommand','stop');

open_system(mdl);
fprintf('✅ Setup finished. RL block is %s.\n\n', ...
    iff(~isempty(rlBlk) && strcmp(get_param(rlBlk,'Commented'),'off'),'ENABLED','DISABLED'));

%% --------- helpers (inline) ----------
function y = ternary(c,a,b), if c, y=a; else, y=b; end, end
function y = iff(c,a,b), y = ternary(c,a,b); end
function s = truncate(str)
    if strlength(string(str))>50, s = char(extractBefore(string(str),51) + "..."); else, s = char(str); end
end
function checkTWWidth(mdl,varName)
    blk = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
                      'BlockType','ToWorkspace','VariableName',varName);
    if isempty(blk), fprintf('TW "%s": not found.\n',varName); return; end
    try
        dims = get_param(blk{1},'CompiledPortDimensions');
        w = dims.Inport(2);  % dims.Inport = [1 width] for 1-D signals
        fprintf('TW "%s": width=%d (1 is expected)\n', varName, w);
    catch
        fprintf('TW "%s": (could not read compiled dimensions)\n', varName);
    end
end