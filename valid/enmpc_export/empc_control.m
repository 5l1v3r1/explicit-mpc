function out=empc_control(rsSVM,wienerModel,in)

Ns=1;
tstart=tic;
for i=1:Ns
    [control_action,ind_region]=...
        pwnl_control(rsSVM,wienerModel,...
        numel(rsSVM(1).coef_sv),numel(wienerModel(1).xgrid),in);
end
telapse=toc(tstart);
exec_time=telapse/Ns;

out=[control_action,ind_region,exec_time];
