%% prep_log_signals.m — add To-Workspace taps for key signals
mdl = 'Simple_powertrain_test1';
open_system(mdl);

% Helper to add a To Workspace and connect it
function addTW(blockPath, outportIdx, varname)
    twPath = [bdroot(blockPath) '/' varname '_TW'];
    if isempty(find_system(bdroot(blockPath),'SearchDepth',1,'Name',[varname '_TW']))
        add_block('simulink/Sinks/To Workspace', twPath, ...
            'VariableName',varname, 'SaveFormat','Timeseries', ...
            'MaxDataPoints','1e9', 'Position',[0 0 0 0]);
        % try place near source block
        try
            p = get_param(blockPath,'Position'); off = [120 0 180 30];
            set_param(twPath,'Position',p+off);
        end
    end
    try
        add_line(bdroot(blockPath), ...
            [get_param(blockPath,'Name') '/' num2str(outportIdx)], ...
            [get_param(twPath,'Name') '/1'], 'autorouting','on');
    catch
        % already connected or autoroute failed silently—ignore
    end
end

% 1) v_ref: the line that feeds Driver/Vel_ref
drv = [mdl '/Driver'];
phD = get_param(drv,'PortHandles');
% Create a small Tap by inserting a Signal Conversion so we can connect cleanly
try
    % If not already inserted, place a Signal Conversion before the Driver Vel_ref port
    scPath = [mdl '/VelRef_TAP'];
    if isempty(find_system(mdl,'SearchDepth',1,'Name','VelRef_TAP'))
        add_block('simulink/Signal Attributes/Signal Conversion', scPath, ...
            'Position', [phD.Inport(1).getPosition- [60 10 20 -10]]);
        % Find the source of Vel_ref and rewire through the tap
        src = get_param(phD.Inport(1),'Line');
        src = get(src,'SrcPortHandle');
        delete_line(mdl, get_param(src,'Parent'), get_param(src,'PortNumber'), 'Driver', 1);
        add_line(mdl, get_param(src,'Parent'), get_param(src,'PortNumber'), 'VelRef_TAP', 1,'autorouting','on');
        add_line(mdl, 'VelRef_TAP/1', 'Driver/1','autorouting','on');
    end
    addTW(scPath,1,'v_ref_ts');
catch, warning('Could not tap v_ref; please add a To Workspace on the Vel_ref line named v_ref_ts.'); end

% 2) v: vehicle speed at Sensors/Out (the black "Vel double" wire)
sens = [mdl '/Sensors'];
try
    addTW(sens,1,'v_ts');  % Sensors has 1 Outport feeding the scope
catch, warning('Could not tap Sensors/Out; add To Workspace on that line named v_ts.'); end

% 3) RL action u (branch BEFORE actionMap)
rlBlk = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Name','agentObj');
if isempty(rlBlk)
    % fallback: find any block with Agent parameter
    allBlk = find_system(mdl,'LookUnderMasks','all','FollowLinks','on');
    rlBlk = '';
    for k=1:numel(allBlk)
        try
            dp = get_param(allBlk{k},'DialogParameters');
            if isstruct(dp) && isfield(dp,'Agent'), rlBlk = allBlk{k}; break; end
        catch, end
    end
else
    rlBlk = rlBlk{1};
end
if ~isempty(rlBlk)
    try, addTW(rlBlk,1,'act_ts'); catch, end
else
    warning('RL block not found; skip act_ts.');
end

% 4) actionMap outputs → acc_ts, brk_ts
try, addTW([mdl '/actionMap'],1,'acc_ts'); catch, end
try, addTW([mdl '/actionMap'],2,'brk_ts'); catch, end

% 5) Battery outputs → ibat_ts, vbat_ts, soc_ts
% Adjust the port indices/names below if your Battery block differs.
batt = [mdl '/Battery'];
try, addTW(batt,1,'ibat_ts'); catch, warning('Tap ibat_ts failed'); end
try, addTW(batt,2,'vbat_ts'); catch, warning('Tap vbat_ts failed'); end
% If SOC is on a different outport, change "3" to the correct index:
try, addTW(batt,3,'soc_ts'); catch, warning('Tap soc_ts failed'); end

disp('✅ prep_log_signals done. Run your evaluation script now.');