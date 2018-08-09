close all;clear;clc;
addpath(genpath(pwd));
load('exp_data.mat');
load('ref_gen.mat');

expData=[X Xd Ud U;xd xd ud ud];
normalizedData=(expData-repmat(min(expData),size(expData,1),1))...
    ./(max(expData)-min(expData));

trainData=...
    consolidator(normalizedData(:,1:end-1),normalizedData(:,end),@mean,.75e-2);
validData=...
    consolidator(normalizedData(:,1:end-1),normalizedData(:,end),@mean,.25e-2);

stData=[xd xd ud ud];
stData=(stData-repmat(min(expData),size(stData,1),1))...
    ./(max(expData)-min(expData));

clearvars -except expData trainData validData stData;

wienerModel=...
    struct('eps',1,'beta',1,'nm',8,'sigma',1e-2,'eta',unique([0:2.5e-3:1])');

%% ALG
N=0;
while ~isempty(trainData)
    [trainIndex,wienerModel,sparsity_ratio]=robust_reg(wienerModel,trainData);
    if numel(trainIndex)>0 & numel(trainIndex)<size(trainData,1)
        [svmModel,predictLabel]=region_est(trainData,trainIndex,15);
        N=N+1;
        ENMPC.region(N)=svmModel;
        ENMPC.model(N)=wienerModel;
        trainData(predictLabel==1,:)=[];
        disp(['l1_result: ' num2str(numel(trainIndex)) ...
               ' svm_result: ' num2str(numel(predictLabel(predictLabel==1,:))) ...
               ' iter_result: ' num2str(size(trainData,1))]);
    end
    if numel(trainIndex)==size(trainData,1)
        N=N+1;
        ENMPC.model(N)=wienerModel;
        trainData=[];
    end
end
delete clone1.log;

ENMPC.Zmin=min(expData(:,1:end-1));ENMPC.Zmax=max(expData(:,1:end-1));
ENMPC.qmin=min(expData(:,end));ENMPC.qmax=max(expData(:,end));
save enmpc ENMPC;

%% POSTPROC
load enmpc;
for i=1:size(validData,1)
    regionIndex(i,1)=empc_index(ENMPC,validData(i,1:end-1));
end
for i=1:numel(ENMPC.model)
    err(i)=norm(validData(regionIndex==i,end)-...
        wiener_predict(ENMPC.model(i),validData(regionIndex==i,1:end-1)),Inf);
end
stem(err);hold on;

%%
close all;
for i=1:numel(ENMPC.model)
    ENMPC.model(i)=wiener_train(wienerModel,validData(regionIndex==i,:));
    err(i)=norm(validData(regionIndex==i,end)-wiener_predict(ENMPC.model(i),validData(regionIndex==i,1:end-1)),Inf);
end
stem(err);hold on;
save enmpcx ENMPC;
