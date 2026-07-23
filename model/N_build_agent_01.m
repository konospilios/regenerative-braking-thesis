%% 01_build_or_load_agent.m — DDPG with 3-obs input ([v_ref; v; u_prev]) and 1-act output (u)
mdl   = 'Simple_powertrain_test1';
rlBlk = [mdl '/agentObj'];
if ~bdIsLoaded(mdl), load_system(mdl); end
if ~exist('agents','dir'), mkdir agents; end

% ----- Fixed specs: obs = [v_ref; v; u_prev] (3×1), act = scalar in [-1,1]
obsDim  = 3;
obsLL   = -inf*ones(obsDim,1);   % robust across MATLAB versions
obsUL   =  inf*ones(obsDim,1);
obsInfo = rlNumericSpec([obsDim 1], 'LowerLimit',obsLL, 'UpperLimit',obsUL, 'Name','obs');
actInfo = rlNumericSpec([1 1],      'LowerLimit',-1,    'UpperLimit', 1,    'Name','act');

% Use a new filename so we don't accidentally load an old 2-obs agent
agentFile = fullfile('agents','agent_ddpg_obs3.mat');

% ----- Load compatible agent if available
rebuild = true;
if exist(agentFile,'file')
    S = load(agentFile,'agentObj');
    if isfield(S,'agentObj') && isscalar(S.agentObj) && isobject(S.agentObj)
        try
            ao = S.agentObj;
            okObs = isequal(size(ao.ObservationInfo(1).LowerLimit), [obsDim 1]);
            okAct = isequal(size(ao.ActionInfo(1).LowerLimit),      [1 1]);
            if okObs && okAct
                agentObj = ao;  rebuild = false;
                fprintf('Loaded compatible agent from %s\n',agentFile);
            end
        catch
        end
    end
end

% ----- Build new agent if needed
if rebuild
    % Actor: 3 -> 1 (tanh)
    lgA = layerGraph();
    lgA = addLayers(lgA, featureInputLayer(obsDim,'Name','obs'));
    lgA = addLayers(lgA, fullyConnectedLayer(64,'Name','a_fc1'));   lgA = addLayers(lgA, reluLayer('Name','a_relu1'));
    lgA = addLayers(lgA, fullyConnectedLayer(64,'Name','a_fc2'));   lgA = addLayers(lgA, reluLayer('Name','a_relu2'));
    lgA = addLayers(lgA, fullyConnectedLayer(1,'Name','a_fc3'));    lgA = addLayers(lgA, tanhLayer('Name','a_tanh'));
    lgA = connectLayers(lgA,'obs','a_fc1'); 
    lgA = connectLayers(lgA,'a_fc1','a_relu1');
    lgA = connectLayers(lgA,'a_relu1','a_fc2'); 
    lgA = connectLayers(lgA,'a_fc2','a_relu2');
    lgA = connectLayers(lgA,'a_relu2','a_fc3'); 
    lgA = connectLayers(lgA,'a_fc3','a_tanh');
    actorNet = dlnetwork(lgA);
    actor    = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo, ...
                    'ObservationInputNames','obs');

    % Critic: [obs(3); act(1)] -> Q
    lgC = layerGraph();
    lgC = addLayers(lgC, featureInputLayer(obsDim,'Name','c_obs'));
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
                    'ObservationInputNames','c_obs','ActionInputNames','c_act');

    opts = rlDDPGAgentOptions( ...
        'SampleTime', 0.05, ...
        'TargetSmoothFactor', 1e-3, ...
        'MiniBatchSize', 256, ...
        'ExperienceBufferLength', 1e6);
    opts.NoiseOptions.StandardDeviation          = 0.2;
    opts.NoiseOptions.StandardDeviationDecayRate = 1e-5;

    agentObj = rlDDPGAgent(actor, critic, opts);
    save(agentFile,'agentObj','-v7');
    fprintf('Built NEW agent (3-obs, 1-act) → %s\n',agentFile);
end

% ----- Attach, verify port sizes, and set noise OFF for eval
assignin('base','agentObj',agentObj);
set_param(mdl,'SimulationCommand','stop','FastRestart','off');
set_param(rlBlk,'Agent','agentObj','Commented','off');

% Quick compile to check dimensions at the block interface
try
    feval(mdl,[],[],[],'compile');
    % Expect obs port width = 3, act port width = 1
    portW = get_param(rlBlk,'CompiledPortWidths');
    inW  = portW.Inport(1);
    outW = portW.Outport(1);
    feval(mdl,[],[],[],'term');
    if inW~=3
        error('RL observation width must be 3 ([v_ref; v; u_prev]) but got %d. Fix the Mux feeding agentObj.', inW);
    end
    if outW~=1
        error('RL "action" must be scalar (width=1) but got %d. Fix downstream wiring.', outW);
    end
catch ME
    try, feval(mdl,[],[],[],'term'); end %#ok<TRYNC>
    rethrow(ME);
end

try
    agentObj.AgentOptions.NoiseOptions.StandardDeviation = 0;
    assignin('base','agentObj',agentObj);
end
disp('✅ agentObj ready and assigned (obs=3: [v_ref; v; u_prev], act=1).')