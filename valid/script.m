close all;clear;clc;

load enmpcx;
load ref_gen;

a=[-0.1832;558;118.45;9.6154e+03;...
    1.3712;5.1011e+03;641;426;6557;7.34;4484;0.016];
k=1;
x5d=50;

Ts=1e-4;
Tsim=5;

for i=1:numel(ENMPC.region)
    rsSVM(i).gamma=ENMPC.region(i).Parameters(4);
    rsSVM(i).rho=ENMPC.region(i).rho;
    rsSVM(i).coef_sv=ENMPC.region(i).sv_coef;
    rsSVM(i).sv_set=ENMPC.region(i).SVs';
end

for i=1:numel(ENMPC.model)
    wienerModel(i).z_min=ENMPC.Zmin;
    wienerModel(i).z_max=ENMPC.Zmax;
    wienerModel(i).u_min=ENMPC.qmin;
    wienerModel(i).u_max=ENMPC.qmax;
    wienerModel(i).L=ENMPC.model(i).L;
    wienerModel(i).xgrid=...
        Bdeeta(ENMPC.model(i).eta,ENMPC.model(i).nm,ENMPC.model(i).beta,0)*ENMPC.model(i).mu;
    wienerModel(i).ygrid=ENMPC.model(i).eta;
end

open('empc_stirling_sim.mdl');
sim('empc_stirling_sim.mdl');

t=X.time;
x1=X.signals(1).values;
x2=X.signals(2).values;
x3=X.signals(3).values;
x4=X.signals(4).values;
u_enmpc=U.signals(1).values;
u_nmpc=U.signals(2).values;
i=I.signals.values;
t_enmpc=T.signals(1).values;
t_nmpc=T.signals(2).values;

save se_valid t x1 x2 x3 x4 u_enmpc u_nmpc i t_enmpc t_nmpc;