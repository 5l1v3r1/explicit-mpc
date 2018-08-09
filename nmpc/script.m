%% NMPC Formulation
clear all;close all;clc;

% Model definition
DifferentialState x1 x2 x3 x4;
Control u1 s2_xl s2_xu s4_xl;

% Parameters
a=[-0.1832;558;118.45;9.6154e+03;...
    1.3712;5.1011e+03;641;426;6557;7.34;4484;0.016];
k=1;
x5d=50;
xmin=[1;4.5;1;1];xmax=[60;5.5;300;40];
umin=0;umax=0.9;
Tp=3e-3;Np=20;

% System equations
f=acado.DifferentialEquation();
f.add(dot(x1)==-a(1)*x1-a(3)*x2+a(2));                    
f.add(dot(x2)==-a(4)*x2+a(6)*x1-a(7)*x3);     
f.add(dot(x3)==a(8)*(x2-k*x4*u1));             
f.add(dot(x4)==a(9)*(-x5d+k*x3*u1));

% Optimization problem
ocp=acado.OCP(0,Tp,Np);

h=[diffStates;controls];
hN=[diffStates];

W=diag([1 1 1 1 1 1e4 1e4 1e4]);
WN=diag([1e-3 1e-3 1e-3 1e-3]);

ocp.minimizeLSQ(W,h);
ocp.minimizeLSQEndTerm(WN,hN);

ocp.subjectTo(f);

ocp.subjectTo(xmin(2)<=x2+1e-1*s2_xl);
ocp.subjectTo(x2-1e-1*s2_xu<=xmax(2));
ocp.subjectTo(xmin(4)<=x4+s4_xl);
ocp.subjectTo(umin<=u1<=umax);
ocp.subjectTo(s2_xl>=0);
ocp.subjectTo(s2_xu>=0);
ocp.subjectTo(s4_xl>=0);

% Code generation
mpc=acado.OCPexport(ocp);
mpc.set('QP_SOLVER','QP_QPOASES');
mpc.set('INTEGRATOR_TYPE','INT_RK4');
mpc.set('NUM_INTEGRATOR_STEPS',20);
mpc.exportCode('export_MPC');
copyfile('D:\PhD\ENMPC\stirling-engine\ACADOtoolkit\external_packages\qpoases',...
    'export_MPC/qpoases')
cd export_MPC
make_acado_solver('../acado_MPCstep')
cd ..
    
delete test_RUN.mexw64 test_RUN.m test_data_acadodata_M1.txt test_data_acadodata_M2.txt test.cpp;