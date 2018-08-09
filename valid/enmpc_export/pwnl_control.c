/*
 *  Implementation of explicit nonlinear model predictive control algorithm in C
 *
 *  Compiling syntax:
 *      mex -g pwnl_control.c
 *  Calling syntax:
 *      [ control_action, ind_region, exec_time ] = pwnl_control ( rs_svm, wiener_model, n_sv, n_grid, z )
 */

#include <time.h>
#include "mex.h"

#define TIME_MAX 1

double rbf_kernel(const double *z_1, const double *z_2, const double gamma, const int n_z)
{
	double sum = 0;
    for (int i=0;i<n_z;++i)
        sum += (z_2[i] - z_1[i])*(z_2[i] - z_1[i]);
	return exp(-gamma*sum);
}

double svm_predict(const double *coef_sv, const double *sv_set, const double gamma, const double rho, const int n_z, const int n_sv, const double *z)
{        
    double *sv;
    sv = malloc(sizeof(double) * n_z);
    
	double dec_value = -rho;
	for(int ind_sv=0;ind_sv<n_sv;++ind_sv)
    {
        for(int ind_el=0;ind_el<n_z;++ind_el)
            sv[ind_el] = sv_set[ind_sv*n_z+ind_el];   
		dec_value += coef_sv[ind_sv] * rbf_kernel(z, sv, gamma, n_z);
    }
    
    free(sv);
	return dec_value;
}

double dot_product(const double *L, const int n_z, const double *z)
{
    double sum = 0;
    for(int i=0;i<n_z;++i)
        sum += L[i] * z[i];
    return sum;
}

double nakeinterp1(const double *xgrid, const double *ygrid, const int n_grid, const double x)
{
    double y;
    int i1 = 0, i9 = n_grid-1, imid;
    while (i9>i1+1)
    {
        imid = (i1+i9+1)/2;
        (xgrid[imid]<x) ? (i1 = imid) : (i9 = imid);
    } 
    if (i1==i9)
        y = ygrid[i1];
    else
        y = ygrid[i1] + (ygrid[i9]-ygrid[i1])*(x-xgrid[i1])/(xgrid[i9]-xgrid[i1]);
    return y;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *z_min, *z_max, u_min, u_max, *z, *Z, *sv_set, *coef_sv, *L, *xgrid, *ygrid, gamma, rho;
    double dec_value, control_action;
	int n_z, n_sv, n_grid, n_svm, ind_region, count_region;

    n_svm = (int)mxGetNumberOfElements(prhs[0]);
    n_sv = (int)mxGetScalar(prhs[2]);
    n_grid = (int)mxGetScalar(prhs[3]);
    n_z = (int)mxGetNumberOfElements(prhs[4]);
    Z = mxGetPr(prhs[4]); 
    z = malloc(sizeof(double) * n_z);
        
    z_min = mxGetPr(mxGetField(prhs[1], 0, "z_min"));
    z_max = mxGetPr(mxGetField(prhs[1], 0, "z_max"));
    u_min = mxGetScalar(mxGetField(prhs[1], 0, "u_min"));
    u_max = mxGetScalar(mxGetField(prhs[1], 0, "u_max"));
    
    for(int i=0;i<n_z;++i)
        z[i] = (Z[i]-z_min[i]) / (z_max[i]-z_min[i]);
    
    for(int count_t=0;count_t<TIME_MAX;++count_t){
        ind_region = n_svm + 1;
        count_region = 1;
        while(count_region <= n_svm && ind_region == n_svm + 1)
        {
            coef_sv = mxGetPr(mxGetField(prhs[0], count_region-1, "coef_sv"));
            sv_set = mxGetPr(mxGetField(prhs[0], count_region-1, "sv_set"));
            gamma = mxGetScalar(mxGetField(prhs[0], count_region-1, "gamma"));
            rho = mxGetScalar(mxGetField(prhs[0], count_region-1, "rho"));
            dec_value = svm_predict(coef_sv, sv_set, gamma, rho, n_z, n_sv, z);
            (dec_value >= 0) ? (ind_region = count_region) : (++count_region);
        }
    }
    
    for(int count_t=0;count_t<TIME_MAX;++count_t){
        L = mxGetPr(mxGetField(prhs[1], ind_region-1, "L"));
        xgrid = mxGetPr(mxGetField(prhs[1], ind_region-1, "xgrid"));
        ygrid = mxGetPr(mxGetField(prhs[1], ind_region-1, "ygrid"));
        control_action = nakeinterp1(xgrid, ygrid, n_grid, dot_product(L, n_z, z));
    }
        
    control_action = min(1, max(0, control_action)) * (u_max-u_min) + u_min;
    
    free(z);
         
    plhs[0] = mxCreateDoubleScalar(control_action);
    plhs[1] = mxCreateDoubleScalar(ind_region);    
}