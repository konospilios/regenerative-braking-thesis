%% 02_train_quick_step.m  — quick “step” curriculum for faster learning
clc;

mdl   = 'Simple_powertrain_test1';
rlBlk = [mdl '/agentObj'];
agentFile = fullfile('agents','agent_ddpg.mat');

% Load model & agent
if ~bdIsLoaded(mdl), load_system(mdl); end
if ~exist(agentFile,'file'), error('Run 01_build_or_load_agent first.'); end
S = load(agentFile,'agentObj'); agentObj = S.agentObj;

% Make sure the RL block uses the agent and is enabled
set_param(mdl,'SimulationCommand','stop');
set_param(mdl,'FastRestart','off');
set_param(rlBlk,'Agent','agentObj');
set_param(rlBlk,'Commented','off');
assignin('base','agentObj',agentObj);

% Episode timing (short)
StopTime = 10;                    % seconds
set_param(mdl,'StopTime',num2str(StopTime));

% === Ensure reference source is a STEP during training =========
% Put a Constant in your model (or Model Workspace) called 'Train_UseStep'
% and a Multiport/Manual switch upstream of v_ref. When =1 → step source.
assignin('base','Train_UseStep',1);

% Provide step params in base (your step block should use these variables)
% Step goes from 0 to Amp at StepTime
assignin('base','StepTime',1.0);

% === Observation/Action specs (must match your obsMux) =========
obsInfo = rlNumericSpec([3 1], 'LowerLimit',[-inf;-inf;0], 'UpperLimit',[inf;inf;1], 'Name','obs');
actInfo = rlNumericSpec([1 1], 'LowerLimit',-1, 'UpperLimit',1, 'Name','act');

% Wrap environment
env = rlSimulinkEnv(mdl, rlBlk, obsInfo, actInfo);

% ResetFcn: randomize step amplitude each episode (and optionally SOC)
env.ResetFcn = @(in) localReset(in);

% Turn ON exploration for training
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0.2;
    agentObj.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-4;
    % Modest learning rates (optional, can speed convergence)
    agentObj.AgentOptions.ActorOptimizerOptions.LearnRate  = 1e-4;
    agentObj.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-3;
    assignin('base','agentObj',agentObj);
end

% Steps per episode
Ts = 0.05; try mw=get_param(mdl,'ModelWorkspace'); Ts=getVariable(mw,'Ts'); end %#ok<NASGU>
MaxSteps = ceil(StopTime/0.05);    % use 0.05 if Ts not found

% Small run (fast)
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 15, ...
    'MaxStepsPerEpisode', MaxSteps, ...
    'ScoreAveragingWindowLength', 5, ...
    'StopTrainingCriteria','EpisodeCount', ...
    'StopTrainingValue', 15, ...
    'UseParallel', false, ...
    'Verbose', true, ...
    'Plots','training-progress');

fprintf('Training (step curriculum): %d eps × %d steps, StopTime=%gs\n', ...
    trainOpts.MaxEpisodes, MaxSteps, StopTime);

stats = train(agentObj, env, trainOpts); %#ok<NASGU>

% Save & reattach
if ~exist('agents','dir'), mkdir('agents'); end
save(agentFile,'agentObj','-v7');
assignin('base','agentObj',agentObj);
set_param(mdl,'SimulationCommand','stop');
set_param(mdl,'FastRestart','off');
set_param(rlBlk,'Agent','agentObj');

% After training, disable exploration for evaluation
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
    assignin('base','agentObj',agentObj);
end
disp('✅ Quick step training done & agent saved.');

% --------- local reset: random step each episode ----------------
function in = localReset(in)
    % random target between 6 and 12 m/s
    Amp = 6 + 6*rand;
    in = setVariable(in,'StepAmp',Amp,'Workspace','base');
    % keep using step source for training
    in = setVariable(in,'Train_UseStep',1,'Workspace','base');
end