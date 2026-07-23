%% diag_powertrain_min.m — one-button health check for Simple_powertrain_test1
clc;

mdl = 'Simple_powertrain_test1';
if ~bdIsLoaded(mdl), load_system(mdl); end

% --- Make the sim run (gentle settings that work with Simscape) ---
try
    set_param(mdl, ...
        'SimulationMode','normal', ...
        'SolverType','Variable-step', ...
        'Solver','ode23t', ...      % implicit; fewer derivative complaints
        'MaxStep','auto', ...
        'StopTime','5', ...
        'AutoInsertRateTranBlk','on');
catch
end

% --- Ensure RL block (if present) is enabled & has an agent in base ---
rlBlk = '';
allBlk = find_system(mdl,'LookUnderMasks','all','FollowLinks','on');
for k=1:numel(allBlk)
    try
        dp = get_param(allBlk{k},'DialogParameters');
        if isstruct(dp) && isfield(dp,'Agent')
            rlBlk = allBlk{k}; break;
        end
    catch
    end
end
if ~isempty(rlBlk)
    try, set_param(rlBlk,'Commented','off'); end
    % do NOT force an agent here; we only *observe*
end

% --- Run a short simulation (5 s) ---
try
    set_param(mdl,'SimulationCommand','stop'); pause(0.01);
    simOut = sim(mdl,'StopTime','5','FastRestart','off'); %#ok<NASGU>
catch ME
    fprintf(2,'Simulation error: %s\n', ME.message);
end

% === Helpers to fetch To Workspace data (timeseries or struct-with-time) ===
fetch = @(name) get_ts(name);
has   = @(name) evalin('base',sprintf('exist(''%s'',''var'')==1',name));
tsinfo = @(ts) sprintf('T=[%.3g..%.3g] n=%d min/max=[%.3g %.3g]', ...
            ts.Time(1), ts.Time(end), numel(ts.Time), min(ts.Data), max(ts.Data));

fprintf('\n=========== QUICK DIAGNOSTIC ===========\n');

% ---- 1) Reference & actual speed ----
vref = fetch_first_of({'Veh_ref','v_ref_ts','vref'});
v    = fetch_first_of({'v_ts','Veh_Vel','veh_vel','Vehicle_velocity','Vel'});

if isempty(vref)
    fprintf(2,'[REF]  v_ref not found. Please log a To Workspace as "Veh_ref" or "v_ref_ts".\n');
else
    fprintf('[REF]  %s\n', describe('v_ref', vref, 5));
end

if isempty(v)
    fprintf(2,'[VEL]  vehicle speed not found. Log a To Workspace as "v_ts" (or use your Veh_Vel).\n');
else
    fprintf('[VEL]  %s\n', describe('v', v, 5));
end

% ---- 2) Action path (RL → actionMap → accel/brake) ----
u_ts   = fetch_first_of({'act_u_ts','act_ts','u_ts'});
acc_ts = fetch_first_of({'acc_ts','acc_ts1','acc'});
brk_ts = fetch_first_of({'brk_ts','brk_ts1','brk'});

if isempty(u_ts) && isempty(acc_ts) && isempty(brk_ts)
    fprintf(2,'[ACT]  No action/pedal logging found. Please log ANY of: act_ts / acc_ts / brk_ts.\n');
else
    if ~isempty(u_ts),   fprintf('[ACT]  %s\n', describe('u',   u_ts,   5)); end
    if ~isempty(acc_ts), fprintf('[ACT]  %s\n', describe('acc', acc_ts, 5)); end
    if ~isempty(brk_ts), fprintf('[ACT]  %s\n', describe('brk', brk_ts, 5)); end
end

% ---- 3) Power path (bus voltage/current) ----
vbat = fetch_first_of({'vbat_ts','Bus_V','Vbus','vbat','Motor_power'});
ibat = fetch_first_of({'ibat_ts','Btry_I_in','ibat'});

hasV = ~isempty(vbat); hasI = ~isempty(ibat);
if hasV, fprintf('[PWR]  %s\n', describe('Vbus', vbat, 5)); else, fprintf(2,'[PWR]  No bus voltage logged.\n'); end
if hasI, fprintf('[PWR]  %s\n', describe('Ibus', ibat, 5)); else, fprintf(2,'[PWR]  No bus current logged.\n'); end

% ---- 4) Quick verdicts / hints ----
fprintf('\n----------- VERDICTS -----------\n');
if ~isempty(vref)
    if max(vref.Data) <= 1e-6
        fprintf(2,'• v_ref amplitude ~0 → your reference source is flat. Fix the source (Pulse/Step).\n');
    end
end

if ~isempty(v)
    if max(abs(v.Data)) <= 1e-6
        fprintf(2,'• Vehicle speed stays at 0.\n');
        if hasI && max(abs(ibat.Data)) <= 1e-6
            fprintf(2,'  ↳ Ibus ~0 → the motor/bridge has no electrical power. Check DC source / battery enable.\n');
        end
        if ~isempty(acc_ts) && mean(acc_ts.Data) > 0.8
            fprintf(2,'  ↳ acc ~1 but Ibus ~0 → duty is high but bridge is disabled or supply is 0 V.\n');
        end
        if isempty(acc_ts) && ~isempty(u_ts) && std(u_ts.Data) < 1e-3
            fprintf(2,'  ↳ action u is constant ~0 → agent or switch may be feeding a constant.\n');
        end
    end
end

if hasV && hasI
    E = trapz(common_time(vbat,ibat), resample(vbat,common_time(vbat,ibat)).Data .* resample(ibat,common_time(vbat,ibat)).Data);
    if abs(E) < 1e-6
        fprintf(2,'• Electrical power ~0 over 5s → supply off, or PWM disabled, or Simulink-PS input stalled.\n');
    end
end

% ---- 5) Summarize Stop blocks that might end sim early (FYI) ----
stoppers = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','Stop');
if ~isempty(stoppers)
    fprintf('\n[INFO] Stop Simulation blocks present:\n');
    for i=1:numel(stoppers), fprintf('    %s\n', stoppers{i}); end
end

fprintf('============= END =============\n');

% ==================== local functions ====================
function ts = get_ts(name)
ts = [];
if evalin('base',sprintf('exist(''%s'',''var'')==1',name))
    x = evalin('base',name);
    if isa(x,'timeseries')
        ts = x; return
    elseif isstruct(x) && isfield(x,'time') && isfield(x,'signals')
        ts = timeseries(x.signals.values, x.time); ts.Name = name; return
    end
end
end

function ts = fetch_first_of(names)
ts = [];
for i=1:numel(names)
    ts = get_ts(names{i});
    if ~isempty(ts), return; end
end
end

function s = describe(lbl, ts, expectEnd)
try
    s = sprintf('%-5s : %s', lbl, sprintf('T=[%.3g..%.3g] n=%d min/max=[%.3g %.3g]', ...
        ts.Time(1), ts.Time(end), numel(ts.Time), min(ts.Data), max(ts.Data)));
catch
    s = sprintf('%-5s : (unreadable timeseries)', lbl);
end
end

function T = common_time(ts1, ts2)
tmin = max(min(ts1.Time), min(ts2.Time));
tmax = min(max(ts1.Time), max(ts2.Time));
if tmax<=tmin, T = unique([ts1.Time; ts2.Time]); else, T = linspace(tmin,tmax,200)'; end
end