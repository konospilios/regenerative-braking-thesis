classdef utils_rl
methods(Static)
    function ts = pullTS(name, simOut)
        % base workspace
        if evalin('base',sprintf('exist(''%s'',''var'')',name))
            ts = evalin('base',name); return
        end
        % simOut direct vars
        if isa(simOut,'Simulink.SimulationOutput')
            wn = simOut.who;
            if any(strcmp(wn,name)), ts = simOut.get(name); return; end
            % logsout
            if isprop(simOut,'logsout') && ~isempty(simOut.logsout)
                ds = simOut.logsout; try, ts = ds.get(name).Values; return; end
            end
            % yout
            if isprop(simOut,'yout') && ~isempty(simOut.yout)
                ds = simOut.yout;  try, ts = ds.get(name).Values; return; end
            end
        end
        error('Signal "%s" not found. Log it as To Workspace or logsout.',name);
    end

    function [t,y] = tsXY(sig)
        if isa(sig,'timeseries')
            t = sig.Time; y = sig.Data;
        elseif istimetable(sig)
            t = seconds(sig.Time - sig.Time(1)); y = sig.Variables;
        else
            y = sig; t = (0:numel(y)-1)'; % numeric fallback
        end
        y = squeeze(y); if isvector(y), y = y(:); end
    end
end
end