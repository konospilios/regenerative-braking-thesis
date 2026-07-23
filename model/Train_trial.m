%% ===== TRAIN EV AGENT (DDPG) =====
mdl      = 'Simple_powertrain_test1';
agentBlk = [mdl '/agentObj'];

% Build the Simulink environment around the RL block.
% (obsInfo/actInfo must be in workspace from the agent-creation script)
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

% Optional: set a model reset function per episode (e.g., randomize SOC)
% env.ResetFcn = @(in) setVariable(in,'SOC0',0.7 + 0.2*rand,'Workspace','base');

% Training options (tune to taste)
trainOpts = rlTrainingOptions( ...
    "MaxEpisodes",                 50, ...        % try 150–300 initially
    "MaxStepsPerEpisode",          40, ...       % 4000*Ts seconds per episode
    "ScoreAveragingWindowLength",  5, ...
    "StopTrainingCriteria",        "AverageReward", ...
    "StopTrainingValue",           -50, ...        % example stop target
    "UseParallel",                 false, ...
    "Verbose",                     true, ...
    "SaveAgentCriteria",           "EpisodeReward", ...
    "SaveAgentValue",              -150, ...       % checkpoint threshold
    "Plots",                       "training-progress");

% Train
trainingStats = train(agentObj, env, trainOpts);