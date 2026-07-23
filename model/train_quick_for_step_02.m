clc;

mdl   = 'Simple_powertrain_test1';
rlBlk = [mdl '/agentObj'];
agentFile = fullfile('agents','agent_ddpg.mat');

if ~bdIsLoaded(mdl), load_system(mdl); end
if ~exist(agentFile,'file'), error('Run 01_build_or_load_agent.m first.'); end
S = load(agentFile,'agentObj'); agentObj = S.agentObj;

% Attach agent and enable
set_param(mdl,'SimulationCommand','stop');
set_param(mdl,'FastRestart','off');
set_param(rlBlk,'Agent','agentObj');
set_param(rlBlk,'Commented','off');
assignin('base','agentObj',agentObj);

% Episode length: keep short
StopTime = 10; set_param(mdl,'StopTime',num2str(StopTime));
Ts = 0.05; MaxSteps = ceil(StopTime/Ts);

% Exploration ON for training
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0.2;
    agentObj.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-4;
    agentObj.AgentOptions.ActorOptimizerOptions.LearnRate  = 1e-4;
    agentObj.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-3;
    assignin('base','agentObj',agentObj);
end

% Specs (must match obs/action wiring)
obsInfo = rlNumericSpec([3 1],'LowerLimit',[-inf;-inf;0],'UpperLimit',[inf;inf;1],'Name','obs');
actInfo = rlNumericSpec([1 1],'LowerLimit',-1,'UpperLimit',1,'Name','act');

% Env
env = rlSimulinkEnv(mdl, rlBlk, obsInfo, actInfo);
env.ResetFcn = @(in) in;   % fixed step profile; no randomization yet

% Train small & fast
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 15, ...
    'MaxStepsPerEpisode', MaxSteps, ...
    'ScoreAveragingWindowLength', 5, ...
    'StopTrainingCriteria','EpisodeCount', ...
    'StopTrainingValue', 15, ...
    'UseParallel', false, ...
    'Verbose', true, ...
    'Plots','training-progress');

fprintf('Training: %d eps x %d steps (10 s episodes)\n',trainOpts.MaxEpisodes,MaxSteps);
stats = train(agentObj, env, trainOpts); %#ok<NASGU>

% Save and reattach; set noise OFF for eval
if ~exist('agents','dir'), mkdir('agents'); end
save(agentFile,'agentObj','-v7');
assignin('base','agentObj',agentObj);
set_param(mdl,'SimulationCommand','stop'); set_param(mdl,'FastRestart','off');
set_param(rlBlk,'Agent','agentObj');
try, agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0; assignin('base','agentObj',agentObj); end
disp('✅ Quick step training done & agent saved.');