function y=nmpc_acado(u)
    
Np=20;

x1=u(1);
x2=u(2);
x3=u(3);
x4=u(4);
x1d=u(5);
x2d=u(6);
x3d=u(7);
x4d=u(8);
ud=u(9);

% Reference
input.y=repmat([x1d x2d x3d x4d ud 0 0 0],Np,1);
input.yN=[x1d x2d x3d x4d];

% Initialization
input.x=repmat([x1 x2 x3 x4],Np+1,1);
input.u=ud*ones(Np,4);

% Current states
input.x0=[x1 x2 x3 x4];

% Solve OCP
Ns=1;
tstart=tic;
for i=1:Ns
    output=acado_MPCstep(input);
end
telapse=toc(tstart);
tnmpc=telapse/Ns;
umpc=output.u(1,1);

%
y=[umpc;tnmpc];
