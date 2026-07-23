function [acc, brk] = actionMap(u)
u   = max(-1,min(1,u));
acc = max(u,0);
brk = max(-u,0);