function [trainIndex,wienerModel,sparsity_ratio]=...
    robust_reg(wienerModel,trainData)

nm=wienerModel.nm; 
beta=wienerModel.beta;
eta=wienerModel.eta;
eps=wienerModel.eps;
sigma=wienerModel.sigma;
[nData,nz]=size(trainData(:,1:end-1));
nb=2*nm;
np=nb+nz+nData;

Pi=[-Bdeeta(eta,nm,beta,1)*Psel(1:nb,np);
    Bdeeta(trainData(:,end),nm,beta,0)*Psel(1:nb,np)-...
    trainData(:,1:end-1)*Psel(nb+1:nb+nz,np)-...
    Psel(nb+nz+1:np,np);
    -Bdeeta(trainData(:,end),nm,beta,0)*Psel(1:nb,np)+...
    trainData(:,1:end-1)*Psel(nb+1:nb+nz,np)-...
    Psel(nb+nz+1:np,np);
    -Psel(nb+nz+1:np,np)];
Upsilon=[-eps*ones(numel(eta),1);
         sigma*ones(2*nData,1);
         zeros(nData,1)];

Niter=2e1;
delta=1e-4;
weightFactor=ones(size(trainData,1),1);
Omega=ones(1,nData)*diag(weightFactor)*Psel(nb+nz+1:np,np);
sparsity_ratio_prev=0;
for k=1:Niter
	p_opt=cplexlp(Omega,Pi,Upsilon);
    weightFactor=1./(delta+p_opt(nb+nz+1:end,1));
    Omega=ones(1,nData)*diag(weightFactor)*Psel(nb+nz+1:np,np); 
    sparsity_ratio=numel(find(p_opt(nb+nz+1:end,1)<=delta))/numel(p_opt);
    if sparsity_ratio<=sparsity_ratio_prev
        try
            p_opt=p_opt_prev;
            sparsity_ratio=sparsity_ratio_prev;
        end
        break;
    else
        sparsity_ratio_prev=sparsity_ratio;
        p_opt_prev=p_opt;
    end
end

wienerModel.mu=p_opt(1:nb,1);
wienerModel.L=p_opt(nb+1:nb+nz,1);

err=abs(trainData(:,end)-...
    wiener_predict(wienerModel,trainData(:,1:end-1)));

trainIndex=find(err<=sigma);

if isempty(trainIndex)
    trainIndex=find(p_opt(nb+nz+1:end,1)<=delta);
end

