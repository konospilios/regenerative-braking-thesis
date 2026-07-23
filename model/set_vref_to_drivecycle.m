function set_vref_to_drivecycle(mdl)
% Force the v_ref Multiport Switch to select the drive-cycle input (port 2)

if nargin==0, mdl = 'Simple_powertrain_test1'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

% Find the Multiport Switch that feeds the signal named 'v_ref'
sw = '';
swc = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
                   'BlockType','MultiPortSwitch');
for k = 1:numel(swc)
    ph = get_param(swc{k},'PortHandles');
    try
        ln = get_param(ph.Outport,'Line');
        sigName = get_param(ln,'Name');
        if ischar(sigName) && strcmp(sigName,'v_ref')
            sw = swc{k}; break
        end
    end
end
if isempty(sw)
    % fallback: take the first Multiport Switch (most models have only one)
    sw = swc{1};
    warning('Could not find a Multiport Switch that names its output "v_ref". Using: %s', sw);
end

% Make sure the switch is in one-based contiguous mode
set_param(sw,'DataPortOrder','One-based contiguous');

% Get the CONTROL input source block (port index 1 on the switch)
pc = get_param(sw,'PortConnectivity');
ctrlSrcBlk = pc(1).SrcBlock;   % control input source (your constant "1")

% Set control value to 2  -> select data port #2 (drive-cycle)
if isempty(ctrlSrcBlk)
    error('Control input of %s is not driven. Please connect a Constant to select the data port.', sw);
end
set_param(ctrlSrcBlk,'Value','2');

fprintf('✅ Set control of "%s" to 2 (drive-cycle input).\n', get_param(sw,'Name'));

% Quick 5-s sanity run to confirm v_ref isn’t flat
st = get_param(mdl,'StopTime'); set_param(mdl,'StopTime','5');
simOut = sim(mdl,'StopTime','5','FastRestart','off'); %#ok<NASGU>
set_param(mdl,'StopTime',st);

% Try to fetch v_ref_ts from base (requires your To Workspace block)
if evalin('base','exist(''v_ref_ts'',''var'')==1')
    vref = evalin('base','v_ref_ts');
    fprintf('v_ref_ts: t=[%.2f..%.2f]  min=%.3g  max=%.3g\n', ...
        vref.Time(1), vref.Time(end), min(vref.Data), max(vref.Data));
else
    fprintf('Note: no "v_ref_ts" in base. Make sure your To Workspace block is named v_ref_ts.\n');
end
end