%% 03_eval_run.m — evaluate (no training) and plot KPIs (battery optional)
clc;

% mdl   = 'Simple_powertrain_test1';
% --- Pre-sim Simscape patch: make the electrical net actually run ---
mdl = 'Simple_powertrain_test1';
if ~bdIsLoaded(mdl), load_system(mdl); end

% 1) Use an implicit, variable-step solver (Simscape-friendly)
try
    set_param(mdl, 'SimulationMode','normal', ...
        'SolverType','Variable-step', 'Solver','ode23t', ...
        'MaxStep','auto', 'AutoInsertRateTranBlk','on');
catch
    set_param(mdl, 'SolverType','Variable-step', 'Solver','ode15s', ...
        'MaxStep','auto', 'AutoInsertRateTranBlk','on');
end

% 2) Turn ON local solver in all Solver Configuration blocks (safe defaults)
sc = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','MaskType','Solver Configuration');
for k = 1:numel(sc)
    try
        set_param(sc{k}, 'UseLocalSolver','on', ...
                        'LocalSolverChoice','Backward Euler', ...
                        'LocalSolverSampleTime','1e-4', ...   % 100 µs
                        'MaxNonlinIter','5');
    catch
        % some releases use different param names – ignore if missing
    end
end

% 3) Make Simulink-PS Converter inputs tolerant (so derivatives not required)
spc = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','MaskType','Simulink-PS Converter');
for k = 1:numel(spc)
    ok = false;
    try, set_param(spc{k},'InputFiltering','first-order'); ok=true; end
    if ~ok, try, set_param(spc{k},'FilterType','first-order'); ok=true; end, end
    if ok
        try, set_param(spc{k},'InputFilterTimeConstant','0.01'); end
        try, set_param(spc{k},'FilterTimeConstant','0.01');     end
    else
        % last resort: tell it we provide zero derivatives if those flags exist
        try, set_param(spc{k},'FirstDerivativeSource','Provided derivative is 0'); end
        try, set_param(spc{k},'SecondDerivativeSource','Provided derivative is 0'); end
    end
end

% 4) Quick DC bus sanity print (if you added vbat_ts / ibat_ts To Workspace)
if evalin('base','exist(''vbat_ts'',''var'')'), vb = evalin('base','vbat_ts');
    if isa(vb,'timeseries'), Vb = vb.Data; else, Vb = vb.signals.values; end
    fprintf('[PreSim] vbat_ts min/max: [%g %g] V\n', min(Vb), max(Vb));
else
    fprintf('[PreSim] vbat_ts not found (ok if you didn’t wire it yet)\n');
end
if evalin('base','exist(''ibat_ts'',''var'')'), ib = evalin('base','ibat_ts');
    if isa(ib,'timeseries'), Ib = ib.Data; else, Ib = ib.signals.values; end
    fprintf('[PreSim] ibat_ts min/max: [%g %g] A\n', min(Ib), max(Ib));
else
    fprintf('[PreSim] ibat_ts not found (ok if you didn’t wire it yet)\n');
end
rlBlk = [mdl '/agentObj'];

% ---- make sure a valid agent exists; keep noise off for inference ----
if evalin('base','exist(''agentObj'',''var'')~=1')
    error('agentObj not found in base. Run 01_build_or_load_agent.m first.');
end
agentObj = evalin('base','agentObj');
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
    assignin('base','agentObj',agentObj);
end

% ---- ensure the RL block uses this agent ----
set_param(mdl,'SimulationCommand','stop');
set_param(mdl,'FastRestart','off');
try, set_param(rlBlk,'Agent','agentObj');  end
try, set_param(rlBlk,'Commented','off');   end

% ---- simulate ----
simOut = sim(mdl,'StopTime',get_param(mdl,'StopTime')); %#ok<NASGU>

% ---------- helpers ----------
mkcol = @(ts) squeezeTS(ts);

% fetch a required signal by any of several names
require = @(cands,hint) mkcol(pickTS(cands,true,hint));
% fetch an optional signal (returns [] if not present)
optional = @(cands) pickTS(cands,false,'');

% ---- Required signals ----
vref = require({"v_ref_ts","Veh_ref","vref"}, 'v_ref_ts/Veh_ref');
v    = require({"v_ts","Veh_Vel","veh_vel","Vehicle_velocity","Vel"}, 'v_ts/Veh_Vel');

% ---- Actions / pedals ----
u_ts   = optional({"act_u_ts","act_ts","u_ts"});
acc_ts = optional({"acc_ts","acc_ts1","acc"});
brk_ts = optional({"brk_ts","brk_ts1","brk"});

if isempty(acc_ts) || isempty(brk_ts)
    if isempty(u_ts)
        error(['No action logging found. Please log either:' newline ...
               '  • act_u_ts, acc_ts, brk_ts   (preferred), or' newline ...
               '  • act_ts (scalar); I will synthesize acc/brk.']);
    end
    u_ts   = mkcol(u_ts);
    acc_ts = timeseries(max(u_ts.Data,0),  u_ts.Time, 'Name','acc');
    brk_ts = timeseries(max(-u_ts.Data,0), u_ts.Time, 'Name','brk');
else
    acc_ts = mkcol(acc_ts);
    brk_ts = mkcol(brk_ts);
    if isempty(u_ts)
        % synthesize u for plotting if only pedals were logged
        T = intersectTimes(acc_ts.Time, brk_ts.Time);
        accR = resample(acc_ts, T);
        brkR = resample(brk_ts, T);
        u_ts = timeseries(accR.Data - brkR.Data, T, 'Name','u');
    else
        u_ts = mkcol(u_ts);
    end
end

% ---- Optional battery signals ----
vbat = optional({"vbat_ts","Bus_V","Vbus","vbat"});
ibat = optional({"ibat_ts","Btry_I_in","ibat"});
soc  = optional({"soc_ts","SOC","soc"});

haveBattery = ~isempty(vbat) && ~isempty(ibat);
if haveBattery
    vbat = mkcol(vbat); ibat = mkcol(ibat);
end
haveSOC = ~isempty(soc); if haveSOC, soc = mkcol(soc); end

% ---- KPIs ----
vrefR = vref.resample(v.Time);
rmse  = sqrt(mean((vrefR.Data - v.Data).^2,'omitnan'));

if haveBattery
    vbatR = vbat.resample(ibat.Time);
    P     = vbatR.Data .* ibat.Data;          % W
    dt    = mean(diff(ibat.Time));            % s
    E_drawn = sum(max( P,0))*dt/3600;         % Wh
    E_regen = sum(max(-P,0))*dt/3600;         % Wh
    regen_pct = 100*E_regen/max(E_drawn,eps);
else
    E_drawn = NaN; E_regen = NaN; regen_pct = NaN;
end

if haveSOC
    minSOC = min(soc.Data); maxSOC = max(soc.Data);
else
    minSOC = NaN; maxSOC = NaN;
end

fprintf('\n=== EVALUATION (inference) ===\n');
fprintf('RMSE(v)  : %.3f\n', rmse);
if haveBattery
    fprintf('Energy   : drawn %.2f Wh, regen %.2f Wh (%.1f%%)\n', E_drawn, E_regen, regen_pct);
else
    fprintf('Energy   : (battery signals not found → skipped)\n');
end
if haveSOC
    fprintf('SOC      : %.3f – %.3f\n\n', minSOC, maxSOC);
else
    fprintf('SOC      : (not logged)\n\n');
end

% ---- plots ----
if haveBattery || haveSOC
    figure('Name','RL Evaluation','Color','w'); tiledlayout(3,1);
else
    figure('Name','RL Evaluation','Color','w'); tiledlayout(2,1);
end

% a) speed tracking
nexttile;
plot(v.Time, v.Data, 'LineWidth',1.2); hold on;
plot(vrefR.Time, vrefR.Data, '--', 'LineWidth',1.2);
grid on; ylabel('Speed'); legend('v','v_{ref}','Location','best');
title(sprintf('Tracking (RMSE=%.3f)',rmse));

% b) action & pedals
nexttile;
yyaxis left;  plot(u_ts.Time,   u_ts.Data,   'LineWidth',1); ylabel('action u');
yyaxis right; plot(acc_ts.Time, acc_ts.Data, 'g'); hold on; plot(brk_ts.Time, brk_ts.Data, 'r');
grid on; ylabel('acc/brk'); legend('u','acc','brk','Location','best');
title('Action and Command Split');

% c) battery & SOC (only if available)
if haveBattery || haveSOC
    nexttile;
    if haveBattery
        yyaxis left;  plot(ibat.Time, ibat.Data, 'LineWidth',1); ylabel('I_{bus} (A)');
    end
    if haveSOC
        yyaxis right; plot(soc.Time,  soc.Data,  'LineWidth',1); ylabel('SOC');
    end
    grid on; xlabel('Time (s)');
    if haveBattery && haveSOC
        legend('I_{bus}','SOC','Location','best');
        title(sprintf('Energy: drawn %.1f Wh, regen %.1f Wh (%.0f%%)',E_drawn,E_regen,regen_pct));
    elseif haveBattery
        legend('I_{bus}','Location','best');
        title(sprintf('Energy: drawn %.1f Wh, regen %.1f Wh (%.0f%%)',E_drawn,E_regen,regen_pct));
    else
        legend('SOC','Location','best');
        title('SOC (battery current/voltage not logged)');
    end
end

%% ---------- local helpers ----------
function ts = pickTS(cands, mustHave, hint)
ts = [];
for i = 1:numel(cands)
    name = cands{i};
    if evalin('base',sprintf('exist(''%s'',''var'')==1',name))
        x = evalin('base',name);
        if isa(x,'timeseries')
            ts = x; return
        elseif isstruct(x) && isfield(x,'time') && isfield(x,'signals')
            ts = timeseries(x.signals.values, x.time); ts.Name = name; return
        end
    end
end
if mustHave
    error('Missing required timeseries. Expected something like: %s', hint);
end
end

function ts = squeezeTS(x)
if ~isa(x,'timeseries'), error('Expected timeseries.'); end
y = squeeze(x.Data); if isvector(y), y = y(:); end
ts = timeseries(y, x.Time); ts.Name = x.Name;
end

function T = intersectTimes(t1,t2)
tmin = max(min(t1), min(t2));
tmax = min(max(t1), max(t2));
if tmax <= tmin, T = unique([t1; t2]); else, T = unique([t1; t2; linspace(tmin,tmax,200)']); end
end