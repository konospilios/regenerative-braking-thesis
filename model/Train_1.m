%% ====== QUICK TRAINING FOR YOUR SIMULINK RL SETUP ======
% Model + RL Agent block
mdl    = 'Simple_powertrain_test1';
rlBlk  = [mdl '/agentObj'];        % path to the RL Agent block

% Make sure the RL block points to the variable we will train
set_param(rlBlk,'Agent','agentObj');   % you already have agentObj in base workspace

% Keep episodes short while we debug training
StopTime = 50;                         % seconds  (use small number while prototyping)
set_param(mdl,'StopTime',num2str(StopTime));

% (Optional) faster iteration
set_param(mdl,'FastRestart','on');     % avoids recompiling model every episode

% === Specs (match what your obsMux/actionMap assume) ===
% Observation = [v_ref; v; SOC]  -> 3x1
obsInfo = rlNumericSpec([3 1], ...
    'LowerLimit',[-inf; -inf; 0], ...
    'UpperLimit',[ inf;  inf; 1], ...
    'Name','obs');

% Action = scalar in [-1,1] (your actionMap splits to acc/brk)
actInfo = rlNumericSpec([1 1], 'LowerLimit',-1, 'UpperLimit',1, 'Name','act');

% === Create Simulink environment wrapped around your RL block ===
env = rlSimulinkEnv(mdl, rlBlk, obsInfo, actInfo);

% (Optional) simple reset function – here just returns empty (no randomization yet)
env.ResetFcn = @() [];

% === Pull your existing agent (actor/critic you already built) ===
agent = evalin('base','agentObj');

% Turn ON exploration noise for training (you had it off for evaluation)
opt = agent.AgentOptions;
if isfield(opt,'NoiseOptions')
    opt.NoiseOptions.StandardDeviation = 0.2;      % small exploratory noise
    opt.NoiseOptions.StandardDeviationDecayRate = 1e-4;
end
agent.AgentOptions = opt;

% === Training options (SHORT runs) ===
Ts = 0.05;                                      % your control sample time
MaxSteps = ceil(StopTime/Ts);                   % one simulation step per Ts

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',            20, ...           % keep it small to start
    'MaxStepsPerEpisode',     MaxSteps, ...
    'ScoreAveragingWindowLength', 5, ...
    'StopTrainingCriteria',   'AverageReward', ...
    'StopTrainingValue',      5e2, ...          % just a placeholder target
    'SaveAgentCriteria',      'EpisodeReward', ...
    'SaveAgentValue',         6e2, ...
    'UseParallel',            false, ...
    'Verbose',                true, ...
    'Plots',                  'training-progress');

% === Train ===
fprintf('Starting training: %d episodes, %d steps/episode (StopTime=%gs)\n', ...
        trainOpts.MaxEpisodes, MaxSteps, StopTime);

[trainedAgent, trainStats] = train(agent, env, trainOpts);

% Put the trained agent back on the block and in base workspace
assignin('base','agentObj',trainedAgent);
set_param(rlBlk,'Agent','agentObj');

fprintf('Training done. You can now run your evaluation script again.\n');