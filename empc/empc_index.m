function regionIndex=empc_index(ENMPC,Z)

N=numel(ENMPC.region);
regionIndex=N+1;
i=1;
while i<=N && regionIndex==N+1
    pointLocation=svmpredict(1,Z,ENMPC.region(i));
    if pointLocation==1
        regionIndex=i;
    else
        i=i+1;
    end
end

