function [svmModel,predictLabel]=region_est(trainData,trainIndex,nSV)

trainLabel=-1*ones(size(trainData,1),1);
trainLabel(trainIndex)=1;
mat2svm([trainLabel trainData]);
[trainLabel,trainData]=libsvmread('mySVMdata.txt');

assignin('base','svmObj',-1e10);
opt.algorithm=NLOPT_GN_DIRECT_L_RAND;
opt.xtol_abs=[1e-3;1e-3];
opt.ftol_abs=1e-3;
opt.maxeval=5e1;
opt.max_objective=@(x) svm_obj(trainLabel,trainData,nSV,x);
opt.lower_bounds=[-3;-3];
opt.upper_bounds=[3;3];
opt.verbose=0;
opt.initial_step=[1e-1;1e-1];
nlopt_optimize_mex(opt,[0 0]);

svmModel=evalin('base','svmModel');
predictLabel=svmpredict(ones(size(trainData,1),1),...
    trainData(:,1:end-1),svmModel);

delete('mySVMdata.txt');

function J=svm_obj(trainLabel,trainData,nSV,x)

% Training
options=sprintf('-c %f -g %f -h 0 -q',10^(x(1)),1/2/(10^(x(2)))^2);
svmModel=svmtrain(trainLabel,trainData(:,1:end-1),options);

% Reduced expansion
svmModel=svm_rs(svmModel,nSV);

% Threshold adjustment
[~,~,decValues]=svmpredict(trainLabel(trainLabel==-1,:),...
                           trainData(trainLabel==-1,1:end-1),svmModel);
svmModel.rho=svmModel.rho+max(0,max(decValues));

% Criteria
[predictLabel,~,~]=svmpredict(trainLabel(trainLabel==1,:),...
                              trainData(trainLabel==1,1:end-1),svmModel);
J=numel(predictLabel(predictLabel==1))/numel(predictLabel);

% Storage
if J>=evalin('base','svmObj')
   assignin('base','svmModel',svmModel); 
   assignin('base','svmObj',J);
end

function svmModel=svm_rs(svmModel,nSV)
model=libSVM_to_STPRtool(svmModel);
red_model=rsrbf(model,struct('nsv',nSV));
svmModel=update_libSVM(svmModel,red_model);

function STPRtool_model=libSVM_to_STPRtool(libSVM_model)
STPRtool_model.Alpha=full(libSVM_model.sv_coef);
STPRtool_model.b=libSVM_model.rho;
STPRtool_model.sv.X=full(libSVM_model.SVs');
try STPRtool_model.sv.y=...
        [libSVM_model.Label(1)*ones(1,libSVM_model.nSV(1)),...
        libSVM_model.Label(2)*ones(1,libSVM_model.nSV(2))];
catch
    STPRtool_model.sv.y=svmpredict(ones(libSVM_model.totalSV,1),libSVM_model.SVs,libSVM_model);
end
STPRtool_model.sv.inx=[1:libSVM_model.totalSV];

switch(libSVM_model.Parameters(2)) % according to kernel type
    case 0 % linear
        STPRtool_model.options.ker='linear';
        STPRtool_model.options.arg=[];
    case 1 % ploynomial
        STPRtool_model.options.ker='poly';
        STPRtool_model.options.arg(1)=libSVM_model.Parameters(3); % degree
        STPRtool_model.options.arg(2)=libSVM_model.Parameters(5); % coef0
        % Gamma is fixed to 1 in STPRtool
    case 2 % RBF
        STPRtool_model.options.ker='rbf';
        STPRtool_model.options.arg=sqrt(1/(2*libSVM_model.Parameters(4)));
    case 3 % sigmoid
        STPRtool_model.options.ker='sigmoid';
        STPRtool_model.options.arg(1)=libSVM_model.Parameters(4); % Gamma
        STPRtool_model.options.arg(2)=libSVM_model.Parameters(5); % coef0
end

function libSVM_model=update_libSVM(model_template,STPRtool_model)
libSVM_model=model_template;
libSVM_model.rho=STPRtool_model.b;
libSVM_model.totalSV=STPRtool_model.nsv;
pindex=find(STPRtool_model.Alpha>0);
libSVM_model.sv_coef=STPRtool_model.Alpha(pindex);
libSVM_model.SVs=(STPRtool_model.sv.X(:,pindex))';
libSVM_model.nSV(1)=length(pindex);
nindex=find(STPRtool_model.Alpha<0);
libSVM_model.sv_coef=[libSVM_model.sv_coef; STPRtool_model.Alpha(nindex)];
libSVM_model.SVs=[libSVM_model.SVs; (STPRtool_model.sv.X(:,nindex))'];
libSVM_model.nSV(2)=length(nindex);
libSVM_model.SVs=sparse(libSVM_model.SVs);

function mat2svm(x)
% we need to convert to: class 1:value1 2:value2 ...
% x is 2D array 
% It produce output text file called 'mySVMdata.txt'
% simple example to generate random x :
%      x=randi(10,20);  y=randi(2,20,1); y(y==2)=-1; x=[y x]
clc
[h w]=size(x);

for i=1:h 
    s=[];
    for j=2:w
        s=[s num2str(j-1) ':' num2str(x(i,j)) ' '];
    end    
    ss=num2str(x(i,1));
    sss=[ss ' ' s];
    y{i,1}=sss;    
end
fid = fopen('mySVMdata.txt', 'w');
for r=1:h
    fprintf(fid, '%s \n', y{r,1});    
end
fclose(fid);