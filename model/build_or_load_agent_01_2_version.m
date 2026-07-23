%% 01_build_or_load_agent.m  — speed-only (obs = [v_ref; v])
mdl   = 'Simple_powertrain_test1';
rlBlk = [mdl '/agentObj'];
if ~bdIsLoaded(mdl), load_system(mdl); end
if ~exist('agents','dir'), mkdir('agents'); end
agentFile = fullfile('agents','agent_ddpg_speedonly.mat');

OBS_DIM = 2;  % [v_ref; v]   (keep SOC in model but NOT in obs yet)
ACT_DIM = 1;  % scalar action mapped to acc/brk
Ts      = 0.05;

% Try to load an existing agent
agentLoaded = false;
if exist(agentFile,'file')
    S = load(agentFile,'agentObj');
    if isfield(S,'agentObj') && isobject(S.agentObj) && isscalar(S.agentObj)
        agentObj = S.agentObj;
        agentLoaded = true;
        fprintf('Loaded agent from %s (%s)\n', agentFile, class(agentObj));
    end
end

if ~agentLoaded
    % ----- Specs -----
    obsInfo = rlNumericSpec([OBS_DIM 1], 'LowerLimit',-inf(OBS_DIM,1), ...
                                          'UpperLimit', inf(OBS_DIM,1), ...
                                          'Name','obs');
    actInfo = rlNumericSpec([ACT_DIM 1], 'LowerLimit',-1, 'UpperLimit',1, 'Name','act');

    % ----- Tiny actor (fast!) -----
    lgA = layerGraph();
    lgA = addLayers(lgA, featureInputLayer(OBS_DIM,'Name','obs'));
    lgA = addLayers(lgA, fullyConnectedLayer(32,'Name','a_fc1'));
    lgA = addLayers(lgA, reluLayer('Name','a_relu1'));
    lgA = addLayers(lgA, fullyConnectedLayer(32,'Name','a_fc2'));
    lgA = addLayers(lgA, reluLayer('Name','a_relu2'));
    lgA = addLayers(lgA, fullyConnectedLayer(ACT_DIM,'Name','a_fc3'));
    lgA = addLayers(lgA, tanhLayer('Name','a_tanh'));
    lgA = connectLayers(lgA,'obs','a_fc1');
    lgA = connectLayers(lgA,'a_fc1','a_relu1');
    lgA = connectLayers(lgA,'a_relu1','a_fc2');
    lgA = connectLayers(lgA,'a_fc2','a_relu2');
    lgA = connectLayers(lgA,'a_relu2','a_fc3');
    lgA = connectLayers(lgA,'a_fc3','a_tanh');
    actor = rlContinuousDeterministicActor(dlnetwork(lgA), obsInfo, actInfo, ...
                                           'ObservationInputNames','obs');

    % ----- Tiny critic -----
    lgC = layerGraph();
    lgC = addLayers(lgC, featureInputLayer(OBS_DIM,'Name','c_obs'));
    lgC = addLayers(lgC, fullyConnectedLayer(32,'Name','c_obs_fc'));
    lgC = addLayers(lgC, reluLayer('Name','c_obs_relu'));
    lgC = addLayers(lgC, featureInputLayer(ACT_DIM,'Name','c_act'));
    lgC = addLayers(lgC, fullyConnectedLayer(32,'Name','c_act_fc'));
    lgC = addLayers(lgC, concatenationLayer(1,2,'Name','concat'));
    lgC = addLayers(lgC, reluLayer('Name','c_relu1'));
    lgC = addLayers(lgC, fullyConnectedLayer(32,'Name','c_fc2'));
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
    critic = rlQValueFunction(dlnetwork(lgC), obsInfo, actInfo, ...
                              'ObservationInputNames','c_obs','ActionInputNames','c_act');

    % ----- Agent options -----
    opts = rlDDPGAgentOptions('SampleTime',Ts, ...
        'TargetSmoothFactor',1e-3, 'MiniBatchSize',128, 'ExperienceBufferLength',2e5);
    opts.NoiseOptions.StandardDeviation = 0.2;
    opts.NoiseOptions.StandardDeviationDecayRate = 5e-5;

    agentObj = rlDDPGAgent(actor,critic,opts);
    save(agentFile,'agentObj','-v7');
    fprintf('Built new speed-only agent and saved to %s\n', agentFile);
end

% Attach to block and freeze noise for evaluation by default
set_param(mdl,'SimulationCommand','stop'); set_param(mdl,'FastRestart','off');
set_param(rlBlk,'Agent','agentObj'); set_param(rlBlk,'Commented','off');
try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0; %#ok<STRNU>
end
assignin('base','agentObj',agentObj);
fprintf('✅ agentObj (%s) ready (obs=[v_ref; v]).\n', class(agentObj));