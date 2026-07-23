%% 02_train_quick_step.m — quick DDPG training + robust Simscape setup
clc;

mdl        = 'Simple_powertrain_test1';
rlBlk      = [mdl '/agentObj'];           % RL Agent block
PULSE_PATH = '';                          % optional: exact Pulse Generator path (auto-found if empty)
SWITCH_BLK = [mdl '/src_sel'];            % Manual Switch (agent vs PID)

% Match 3-observation agent file from 01_build
agentFile  = fullfile('agents','agent_ddpg_obs3.mat');

if ~bdIsLoaded(mdl), load_system(mdl); end
if ~exist(agentFile,'file'), error('Run 01_build_or_load_agent.m first (3-observation version).'); end

%% ---------- ONE-TIME ROBUST SETUP (solver + Simulink-PS Converters) ----------
% Use an implicit variable-step solver (good with Simscape)
try
    set_param(mdl, ...
        'SimulationMode','normal', ...
        'SolverType','Variable-step', ...
        'Solver','ode23t', ...
        'MaxStep','auto', ...
        'AutoInsertRateTranBlk','on');
catch
    try
        set_param(mdl, ...
            'SimulationMode','normal', ...
            'SolverType','Variable-step', ...
            'Solver','ode15s', ...
            'MaxStep','auto', ...
            'AutoInsertRateTranBlk','on');
    end
end

% Make Simulink-PS Converters tolerant (simple input filtering)
spc = find_system(mdl, 'LookUnderMasks','all','FollowLinks','on', 'MaskType','Simulink-PS Converter');
for k = 1:numel(spc)
    b = spc{k};
    ok = false;
    try
        set_param(b, 'InputFiltering','first-order');
        set_param(b, 'InputFilterTimeConstant','0.01'); % 10 ms
        ok = true;
    end
    if ~ok
        try
            set_param(b,'FilterType','first-order');
            set_param(b,'FilterTimeConstant','0.01');
        end
    end
end
if ~isempty(spc)
    fprintf('Configured %d Simulink-PS Converter(s) for filtered inputs.\n', numel(spc));
end

%% ---------- Load agent and wire it ----------
S = load(agentFile,'agentObj');
agentObj = S.agentObj;

set_param(mdl,'SimulationCommand','stop','FastRestart','off');
set_param(rlBlk,'Agent','agentObj');
set_param(rlBlk,'Commented','off');
assignin('base','agentObj',agentObj);

% Select agent path on manual switch (if present)
try, set_param(SWITCH_BLK,'sw','1'); catch, end

%% ---------- Episode sizing ----------
StopTime = 12;  set_param(mdl,'StopTime',num2str(StopTime));
Ts       = 0.05;
MaxSteps = ceil(StopTime/Ts);

%% ---------- Exploration for training ----------
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation           = 0.2;
    agentObj.AgentOptions.NoiseOptions.StandardDeviationDecayRate  = 1e-4;
    agentObj.AgentOptions.ActorOptimizerOptions.LearnRate          = 1e-4;
    agentObj.AgentOptions.CriticOptimizerOptions.LearnRate         = 1e-3;
    assignin('base','agentObj',agentObj);
end

%% ---------- SPECS (must match wiring) ----------
% obs = [v_ref; v; u_prev]  (3-by-1), action scalar in [-1,1]
obsInfo = rlNumericSpec([3 1], ...
    'LowerLimit',[-inf; -inf; -inf], ...
    'UpperLimit',[ inf;  inf;  inf], ...
    'Name','obs');

actInfo = rlNumericSpec([1 1], ...
    'LowerLimit',-1, ...
    'UpperLimit', 1, ...
    'Name','act');

%% ---------- Find a Pulse Generator (if any) ----------
if isempty(PULSE_PATH)
    pg = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','PulseGenerator');
    if ~isempty(pg)
        PULSE_PATH = pg{1};
        fprintf('Using pulse block: %s\n', PULSE_PATH);
    else
        warning('No Pulse Generator found. ResetFcn will not randomize v_ref.');
        PULSE_PATH = '';
    end
end

%% ---------- Environment & ResetFcn ----------
% IMPORTANT in the MODEL: observation Mux must feed [v_ref; v; u_prev] (this order),
% and u_prev is a Unit Delay of the RL action (Ts = 0.05 s, IC = 0).
env = rlSimulinkEnv(mdl, rlBlk, obsInfo, actInfo);
env.ResetFcn = @(in) localReset(in, PULSE_PATH);

%% ---------- Training options ----------
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 100, ...
    'MaxStepsPerEpisode', MaxSteps, ...
    'ScoreAveragingWindowLength', 5, ...
    'StopTrainingCriteria','EpisodeCount', ...
    'StopTrainingValue', 100, ...
    'UseParallel', false, ...
    'Verbose', true, ...
    'Plots','training-progress');

fprintf('Training: %d eps × %d steps (%.0f s episodes)\n', ...
    trainOpts.MaxEpisodes, MaxSteps, StopTime);

%% ---------- Train ----------
stats = train(agentObj, env, trainOpts); %#ok<NASGU>

%% ---------- Save & reattach; noise off for eval ----------
if ~exist('agents','dir'), mkdir('agents'); end
save(agentFile,'agentObj','-v7');
assignin('base','agentObj',agentObj);

set_param(mdl,'SimulationCommand','stop','FastRestart','off');
set_param(rlBlk,'Agent','agentObj');
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
    assignin('base','agentObj',agentObj);
end
disp('✅ Quick step training done & agent saved.');

%% ---------------- local reset function ----------------
function in = localReset(in, pulsePath)
% Randomize pulse a bit so the agent sees variety
try
    if ~isempty(pulsePath)
        A   = 5  + 7*rand;   % amplitude
        D   = 40 + 30*rand;  % duty (%)
        T   = 6  + 4*rand;   % period (s)
        Ph  = 1*rand;        % phase delay (s)
        in = setBlockParameter(in, pulsePath, ...
            'Amplitude',  num2str(A), ...
            'PulseWidth', num2str(D), ...
            'Period',     num2str(T), ...
            'PhaseDelay', num2str(Ph));
    end
catch
    % ignore if parameter names differ in your block
end
end