mdl = 'Simple_powertrain_test1';

% Path names you use around the plant:
% Replace these with your actual block paths if different.
swAcc = [mdl '/AccCmdSwitch'];   % the Switch feeding Acc_cmd
swBrk = [mdl '/BrkCmdSwitch'];   % the Switch feeding Brake_cmd

% mode = 'heuristic' or 'rl'
mode = 'heuristic';

if strcmpi(mode,'heuristic')
    set_param(swAcc,'Criteria','u2>=Threshold');  % select bottom input
    set_param(swBrk,'Criteria','u2>=Threshold');
elseif strcmpi(mode,'rl')
    set_param(swAcc,'Criteria','u1>=Threshold');  % select top input
    set_param(swBrk,'Criteria','u1>=Threshold');
end

disp(['Switched control mode to: ' mode]);