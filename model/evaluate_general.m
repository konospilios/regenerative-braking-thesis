mdl = 'Simple_powertrain_test1';
simOut = sim(mdl,'StopTime',get_param(mdl,'StopTime'));

vars = {'v_ref_ts','v_ts','act_ts','acc_ts','brk_ts','ibat_ts','vbat_ts','soc_ts'};

for k = 1:numel(vars)
    disp(checkTS(vars{k}, simOut));
end

function msg = checkTS(name, simOut)
    got = false; ts = [];
    % base workspace?
    if evalin('base',sprintf('exist(''%s'',''var'')',name))
        ts = evalin('base',name); got = true;
    end
    % logsout?
    if ~got && isprop(simOut,'logsout') && ~isempty(simOut.logsout)
        try, ts = simOut.logsout.get(name).Values; got = true; end
    end
    if ~got
        msg = sprintf('%s : NOT FOUND', name); return
    end
    if isa(ts,'timeseries') && ~isempty(ts.Time) && ~isempty(ts.Data)
        t = ts.Time; y = ts.Data;
        msg = sprintf('%s : %d pts, t=[%.3g..%.3g], y=[%.3g..%.3g]', ...
              name, numel(t), t(1), t(end), min(y(:)), max(y(:)));
    else
        msg = sprintf('%s : EMPTY or not a timeseries', name);
    end
end