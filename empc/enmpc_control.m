function [uENMPC,iENMPC]=enmpc_control(ENMPC,Z)

Z=ENMPC.aZ.*Z+ENMPC.bZ;

N=numel(ENMPC.region);
iENMPC=N+1;
i=1;
while i<=N && iENMPC==N+1
    pointLocation=svmpredict(1,Z',ENMPC.region(i));
    if pointLocation==1
        iENMPC=i;
    else
        i=i+1;
    end
end

uENMPC=min(1,max(0,...
       splinterp1(ENMPC.model(iENMPC).Yd,...
       ENMPC.model(iENMPC).a*ENMPC.model(iENMPC).L'*Z+ENMPC.model(iENMPC).b)));

uENMPC=ENMPC.aq*uENMPC+ENMPC.bq;