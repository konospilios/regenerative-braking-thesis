%% 02_train_quick_speedonly.m  — short episodes, obs=[v_ref; v]
clc;
mdl   = 'Simple_powertrain_test1';
rlBlk = [mdl '/agentObj'];
if ~bdIsLoaded(mdl), load_system(mdl); end

% Load agent built above
S = load(fullfile('agents','agent_ddpg_speedonly.mat'),'agentObj');
agentObj = S.agentObj; assignin('base','agentObj',agentObj);

% Episode length: keep it SHORT (20 s)
set_param(mdl,'StopTime','20');

% Reduce Simulink logging during training (re-enable for eval)
origLog = get_param(mdl,'SignalLogging');
set_param(mdl,'SignalLogging','off');

% Environment (obs 2x1, act 1x1)
obsInfo = rlNumericSpec([2 1],'LowerLimit',[-inf;-inf],'UpperLimit',[inf;inf],'Name','obs');
actInfo = rlNumericSpec([1 1],'LowerLimit',-1,'UpperLimit',1,'Name','act');
env = rlSimulinkEnv(mdl, rlBlk, obsInfo, actInfo);
env.ResetFcn = @(in) in;  % simple reset

% Turn ON exploration for training
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0.2;
    agentObj.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 5e-5;
    assignin('base','agentObj',agentObj);
end

% Training options — tiny but effective for a smoke test
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 20, ...
    'MaxStepsPerEpisode', 400, ...   % 20s / 0.05s
    'ScoreAveragingWindowLength', 5, ...
    'UseParallel', false, ...
    'Verbose', true, ...
    'Plots','training-progress');

fprintf('Training ~%d steps/episode\n', trainOpts.MaxStepsPerEpisode);
stats = train(agentObj, env, trainOpts); %#ok<NASGU>

% Restore logging for evaluation
set_param(mdl,'SignalLogging',origLog);

% Save & re-attach trained agent; set noise OFF for eval
save(fullfile('agents','agent_ddpg_speedonly.mat'),'agentObj','-v7');
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
end
assignin('base','agentObj',agentObj);
set_param(mdl,'SimulationCommand','stop'); set_param(mdl,'FastRestart','off');
set_param(rlBlk,'Agent','agentObj');
disp('✅ Trained & attached (speed-only). Run 03_eval to compare RMSE.');