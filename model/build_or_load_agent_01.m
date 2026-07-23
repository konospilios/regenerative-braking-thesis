%% 01_build_or_load_agent.m
% Build or load a valid DDPG agent and attach it to the RL block.

mdl   = 'Simple_powertrain_test1';
rlBlk = [mdl '/agentObj'];   % path to your RL Agent block

% Ensure model is loaded (00 opened it, but this is safe)
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

% Safety: if base workspace has an invalid 'agentObj', clear it
if evalin('base', 'exist(''agentObj'',''var'')==1')
    ao = evalin('base','agentObj');
    % If it's not an object at all (e.g., struct), clear it
    if ~isobject(ao)
        evalin('base','clear agentObj');
        disp('Cleared invalid agentObj from base workspace.');
    end
end

% Ensure an agents/ folder exists
if ~exist('agents','dir'); mkdir('agents'); end
agentFile = fullfile('agents','agent_ddpg.mat');

% Try to load an existing agent
agentLoaded = false;
if exist(agentFile,'file')
    S = load(agentFile,'agentObj');
    if isfield(S,'agentObj') && isobject(S.agentObj) && isscalar(S.agentObj)
        agentObj = S.agentObj;
        agentLoaded = true;
        fprintf('Loaded agent from %s (%s)\n', agentFile, class(agentObj));
    else
        disp('Saved agentObj was not a scalar object. Will rebuild.');
    end
end

% Build a new agent if not loaded
if ~agentLoaded
    % Specs: obs = [v_ref; v; SOC], act = scalar in [-1,1]
    obsInfo = rlNumericSpec([3 1], ...
        'LowerLimit', [-inf; -inf; 0], ...
        'UpperLimit', [ inf;  inf; 1], ...
        'Name','obs');
    actInfo = rlNumericSpec([1 1], ...
        'LowerLimit', -1, ...
        'UpperLimit',  1, ...
        'Name','act');

    % ----- Actor (obs -> action in [-1,1]) -----
    lgA = layerGraph();
    lgA = addLayers(lgA, featureInputLayer(3,'Name','obs'));
    lgA = addLayers(lgA, fullyConnectedLayer(64,'Name','a_fc1'));
    lgA = addLayers(lgA, reluLayer('Name','a_relu1'));
    lgA = addLayers(lgA, fullyConnectedLayer(64,'Name','a_fc2'));
    lgA = addLayers(lgA, reluLayer('Name','a_relu2'));
    lgA = addLayers(lgA, fullyConnectedLayer(1,'Name','a_fc3'));
    lgA = addLayers(lgA, tanhLayer('Name','a_tanh'));
    lgA = connectLayers(lgA,'obs','a_fc1');
    lgA = connectLayers(lgA,'a_fc1','a_relu1');
    lgA = connectLayers(lgA,'a_relu1','a_fc2');
    lgA = connectLayers(lgA,'a_fc2','a_relu2');
    lgA = connectLayers(lgA,'a_relu2','a_fc3');
    lgA = connectLayers(lgA,'a_fc3','a_tanh');

    actorNet = dlnetwork(lgA);
    % In R2024a, the actor uses the last layer as the action layer by default.
    actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo, ...
        'ObservationInputNames','obs');

    % ----- Critic ([obs; act] -> Q) -----
    lgC = layerGraph();
    lgC = addLayers(lgC, featureInputLayer(3,'Name','c_obs'));
    lgC = addLayers(lgC, fullyConnectedLayer(64,'Name','c_obs_fc'));
    lgC = addLayers(lgC, reluLayer('Name','c_obs_relu'));

    lgC = addLayers(lgC, featureInputLayer(1,'Name','c_act'));
    lgC = addLayers(lgC, fullyConnectedLayer(64,'Name','c_act_fc'));

    lgC = addLayers(lgC, concatenationLayer(1,2,'Name','concat'));
    lgC = addLayers(lgC, reluLayer('Name','c_relu1'));
    lgC = addLayers(lgC, fullyConnectedLayer(64,'Name','c_fc2'));
    lgC = addLayers(lgC, reluLayer('Name','c_relu2'));
    lgC = addLayers(lgC, fullyConnectedLayer(1,'Name','c_q'));

    lgC = connectLayers(lgC,'c_obs','c_obs_fc');
    lgC = connectLayers(lgC,'c_obs_fc','c_obs_relu');
    lgC = connectLayers(lgC,'c_obs_relu','concat/in1');
    lgC = connectLayers(lgC,'c_act','c_act_fc');
    lgC = connectLayers(lgC,'c_act_fc','concat/in2');
    lgC = connectLayers(lgC,'concat','c_relu1');
    lgC = connectLayers(lgC,'c_relu1','c_fc2');
    lgC = connectLayers(lgC,'c_fc2','c_relu2');
    lgC = connectLayers(lgC,'c_relu2','c_q');

    criticNet = dlnetwork(lgC);
    critic    = rlQValueFunction(criticNet, obsInfo, actInfo, ...
        'ObservationInputNames','c_obs', 'ActionInputNames','c_act');

    % ----- Agent options -----
    opts = rlDDPGAgentOptions( ...
        'SampleTime', 0.05, ...
        'TargetSmoothFactor', 1e-3, ...
        'MiniBatchSize', 256, ...
        'ExperienceBufferLength', 1e6);

    % Exploration for training (we’ll turn off for eval elsewhere)
    opts.NoiseOptions.StandardDeviation = 0.2;
    opts.NoiseOptions.StandardDeviationDecayRate = 1e-5;

    agentObj = rlDDPGAgent(actor, critic, opts);

    % Save for persistence
    save(agentFile,'agentObj','-v7');
    fprintf('Built new DDPG agent and saved to %s\n', agentFile);
end

% --- Try attaching to the block; if Simulink accepts it, it's valid ---
assignin('base','agentObj',agentObj);

% Make the block accept the new Agent value
set_param(mdl,'SimulationCommand','stop');
set_param(mdl,'FastRestart','off');

set_param(rlBlk,'Agent','agentObj');   % assign
set_param(rlBlk,'Commented','off');    % enable

% Default to evaluation noise OFF (training will re-enable)
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
    assignin('base','agentObj',agentObj);
end

fprintf('✅ agentObj (%s) assigned to RL block and enabled.\n', class(agentObj));

% Default to evaluation noise OFF (training script will re-enable)
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
    assignin('base','agentObj',agentObj);
end

fprintf('✅ agentObj (%s) assigned to RL block and enabled.\n', class(agentObj));