/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[36] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[37] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[38] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[39] = acadoVariables.u[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 16] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 16 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 16 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 16 + 3] = acadoWorkspace.state[23];
acadoWorkspace.evGu[lRun1 * 16 + 4] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 16 + 5] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 16 + 6] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 16 + 7] = acadoWorkspace.state[27];
acadoWorkspace.evGu[lRun1 * 16 + 8] = acadoWorkspace.state[28];
acadoWorkspace.evGu[lRun1 * 16 + 9] = acadoWorkspace.state[29];
acadoWorkspace.evGu[lRun1 * 16 + 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 16 + 11] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 16 + 12] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 16 + 13] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 16 + 14] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 16 + 15] = acadoWorkspace.state[35];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[0];
out[5] = u[1];
out[6] = u[2];
out[7] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 8] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 8 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 8 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 8 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 8 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 8 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 8 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 8 + 7] = acadoWorkspace.objValueOut[7];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15];
Gu2[4] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[4] + Gx1[6]*Gu1[8] + Gx1[7]*Gu1[12];
Gu2[5] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[5] + Gx1[6]*Gu1[9] + Gx1[7]*Gu1[13];
Gu2[6] = + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[6] + Gx1[6]*Gu1[10] + Gx1[7]*Gu1[14];
Gu2[7] = + Gx1[4]*Gu1[3] + Gx1[5]*Gu1[7] + Gx1[6]*Gu1[11] + Gx1[7]*Gu1[15];
Gu2[8] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[4] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[12];
Gu2[9] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[5] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[13];
Gu2[10] = + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[10] + Gx1[11]*Gu1[14];
Gu2[11] = + Gx1[8]*Gu1[3] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[11] + Gx1[11]*Gu1[15];
Gu2[12] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12];
Gu2[13] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13];
Gu2[14] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14];
Gu2[15] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = (real_t)1.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = 0.0;
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = (real_t)1.0000000000000000e+04;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = (real_t)1.0000000000000000e+04;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = 0.0;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = (real_t)1.0000000000000000e+04;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 3)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = +dOld[0];
dNew[1] = +dOld[1];
dNew[2] = +dOld[2];
dNew[3] = +dOld[3];
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)1.0000000000000000e-03*dOld[0];
dNew[1] = + (real_t)1.0000000000000000e-03*dOld[1];
dNew[2] = + (real_t)1.0000000000000000e-03*dOld[2];
dNew[3] = + (real_t)1.0000000000000000e-03*dOld[3];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[4];
RDy1[1] = + (real_t)1.0000000000000000e+04*Dy1[5];
RDy1[2] = + (real_t)1.0000000000000000e+04*Dy1[6];
RDy1[3] = + (real_t)1.0000000000000000e+04*Dy1[7];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = +Dy1[0];
QDy1[1] = +Dy1[1];
QDy1[2] = +Dy1[2];
QDy1[3] = +Dy1[3];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[4] + E1[8]*Gx1[8] + E1[12]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[5] + E1[8]*Gx1[9] + E1[12]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[6] + E1[8]*Gx1[10] + E1[12]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[7] + E1[8]*Gx1[11] + E1[12]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[5]*Gx1[4] + E1[9]*Gx1[8] + E1[13]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[5]*Gx1[5] + E1[9]*Gx1[9] + E1[13]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[5]*Gx1[6] + E1[9]*Gx1[10] + E1[13]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[5]*Gx1[7] + E1[9]*Gx1[11] + E1[13]*Gx1[15];
H101[8] += + E1[2]*Gx1[0] + E1[6]*Gx1[4] + E1[10]*Gx1[8] + E1[14]*Gx1[12];
H101[9] += + E1[2]*Gx1[1] + E1[6]*Gx1[5] + E1[10]*Gx1[9] + E1[14]*Gx1[13];
H101[10] += + E1[2]*Gx1[2] + E1[6]*Gx1[6] + E1[10]*Gx1[10] + E1[14]*Gx1[14];
H101[11] += + E1[2]*Gx1[3] + E1[6]*Gx1[7] + E1[10]*Gx1[11] + E1[14]*Gx1[15];
H101[12] += + E1[3]*Gx1[0] + E1[7]*Gx1[4] + E1[11]*Gx1[8] + E1[15]*Gx1[12];
H101[13] += + E1[3]*Gx1[1] + E1[7]*Gx1[5] + E1[11]*Gx1[9] + E1[15]*Gx1[13];
H101[14] += + E1[3]*Gx1[2] + E1[7]*Gx1[6] + E1[11]*Gx1[10] + E1[15]*Gx1[14];
H101[15] += + E1[3]*Gx1[3] + E1[7]*Gx1[7] + E1[11]*Gx1[11] + E1[15]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 16; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = +Gx1[0];
Gx2[1] = +Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = +Gx1[8];
Gx2[9] = +Gx1[9];
Gx2[10] = +Gx1[10];
Gx2[11] = +Gx1[11];
Gx2[12] = +Gx1[12];
Gx2[13] = +Gx1[13];
Gx2[14] = +Gx1[14];
Gx2[15] = +Gx1[15];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e-03*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e-03*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e-03*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e-03*Gx1[3];
Gx2[4] = + (real_t)1.0000000000000000e-03*Gx1[4];
Gx2[5] = + (real_t)1.0000000000000000e-03*Gx1[5];
Gx2[6] = + (real_t)1.0000000000000000e-03*Gx1[6];
Gx2[7] = + (real_t)1.0000000000000000e-03*Gx1[7];
Gx2[8] = + (real_t)1.0000000000000000e-03*Gx1[8];
Gx2[9] = + (real_t)1.0000000000000000e-03*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e-03*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e-03*Gx1[11];
Gx2[12] = + (real_t)1.0000000000000000e-03*Gx1[12];
Gx2[13] = + (real_t)1.0000000000000000e-03*Gx1[13];
Gx2[14] = + (real_t)1.0000000000000000e-03*Gx1[14];
Gx2[15] = + (real_t)1.0000000000000000e-03*Gx1[15];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = +Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
Gu2[4] = +Gu1[4];
Gu2[5] = +Gu1[5];
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
Gu2[8] = +Gu1[8];
Gu2[9] = +Gu1[9];
Gu2[10] = +Gu1[10];
Gu2[11] = +Gu1[11];
Gu2[12] = +Gu1[12];
Gu2[13] = +Gu1[13];
Gu2[14] = +Gu1[14];
Gu2[15] = +Gu1[15];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e-03*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e-03*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000000e-03*Gu1[2];
Gu2[3] = + (real_t)1.0000000000000000e-03*Gu1[3];
Gu2[4] = + (real_t)1.0000000000000000e-03*Gu1[4];
Gu2[5] = + (real_t)1.0000000000000000e-03*Gu1[5];
Gu2[6] = + (real_t)1.0000000000000000e-03*Gu1[6];
Gu2[7] = + (real_t)1.0000000000000000e-03*Gu1[7];
Gu2[8] = + (real_t)1.0000000000000000e-03*Gu1[8];
Gu2[9] = + (real_t)1.0000000000000000e-03*Gu1[9];
Gu2[10] = + (real_t)1.0000000000000000e-03*Gu1[10];
Gu2[11] = + (real_t)1.0000000000000000e-03*Gu1[11];
Gu2[12] = + (real_t)1.0000000000000000e-03*Gu1[12];
Gu2[13] = + (real_t)1.0000000000000000e-03*Gu1[13];
Gu2[14] = + (real_t)1.0000000000000000e-03*Gu1[14];
Gu2[15] = + (real_t)1.0000000000000000e-03*Gu1[15];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[4] + Hx[2]*Gx[8] + Hx[3]*Gx[12];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[5] + Hx[2]*Gx[9] + Hx[3]*Gx[13];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[6] + Hx[2]*Gx[10] + Hx[3]*Gx[14];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[7] + Hx[2]*Gx[11] + Hx[3]*Gx[15];
A01[4] = + Hx[4]*Gx[0] + Hx[5]*Gx[4] + Hx[6]*Gx[8] + Hx[7]*Gx[12];
A01[5] = + Hx[4]*Gx[1] + Hx[5]*Gx[5] + Hx[6]*Gx[9] + Hx[7]*Gx[13];
A01[6] = + Hx[4]*Gx[2] + Hx[5]*Gx[6] + Hx[6]*Gx[10] + Hx[7]*Gx[14];
A01[7] = + Hx[4]*Gx[3] + Hx[5]*Gx[7] + Hx[6]*Gx[11] + Hx[7]*Gx[15];
A01[8] = + Hx[8]*Gx[0] + Hx[9]*Gx[4] + Hx[10]*Gx[8] + Hx[11]*Gx[12];
A01[9] = + Hx[8]*Gx[1] + Hx[9]*Gx[5] + Hx[10]*Gx[9] + Hx[11]*Gx[13];
A01[10] = + Hx[8]*Gx[2] + Hx[9]*Gx[6] + Hx[10]*Gx[10] + Hx[11]*Gx[14];
A01[11] = + Hx[8]*Gx[3] + Hx[9]*Gx[7] + Hx[10]*Gx[11] + Hx[11]*Gx[15];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 240) + (col * 4)] = + Hx[0]*E[0] + Hx[1]*E[4] + Hx[2]*E[8] + Hx[3]*E[12];
acadoWorkspace.A[(row * 240) + (col * 4 + 1)] = + Hx[0]*E[1] + Hx[1]*E[5] + Hx[2]*E[9] + Hx[3]*E[13];
acadoWorkspace.A[(row * 240) + (col * 4 + 2)] = + Hx[0]*E[2] + Hx[1]*E[6] + Hx[2]*E[10] + Hx[3]*E[14];
acadoWorkspace.A[(row * 240) + (col * 4 + 3)] = + Hx[0]*E[3] + Hx[1]*E[7] + Hx[2]*E[11] + Hx[3]*E[15];
acadoWorkspace.A[(row * 240 + 80) + (col * 4)] = + Hx[4]*E[0] + Hx[5]*E[4] + Hx[6]*E[8] + Hx[7]*E[12];
acadoWorkspace.A[(row * 240 + 80) + (col * 4 + 1)] = + Hx[4]*E[1] + Hx[5]*E[5] + Hx[6]*E[9] + Hx[7]*E[13];
acadoWorkspace.A[(row * 240 + 80) + (col * 4 + 2)] = + Hx[4]*E[2] + Hx[5]*E[6] + Hx[6]*E[10] + Hx[7]*E[14];
acadoWorkspace.A[(row * 240 + 80) + (col * 4 + 3)] = + Hx[4]*E[3] + Hx[5]*E[7] + Hx[6]*E[11] + Hx[7]*E[15];
acadoWorkspace.A[(row * 240 + 160) + (col * 4)] = + Hx[8]*E[0] + Hx[9]*E[4] + Hx[10]*E[8] + Hx[11]*E[12];
acadoWorkspace.A[(row * 240 + 160) + (col * 4 + 1)] = + Hx[8]*E[1] + Hx[9]*E[5] + Hx[10]*E[9] + Hx[11]*E[13];
acadoWorkspace.A[(row * 240 + 160) + (col * 4 + 2)] = + Hx[8]*E[2] + Hx[9]*E[6] + Hx[10]*E[10] + Hx[11]*E[14];
acadoWorkspace.A[(row * 240 + 160) + (col * 4 + 3)] = + Hx[8]*E[3] + Hx[9]*E[7] + Hx[10]*E[11] + Hx[11]*E[15];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3];
acadoWorkspace.evHxd[1] = + Hx[4]*tmpd[0] + Hx[5]*tmpd[1] + Hx[6]*tmpd[2] + Hx[7]*tmpd[3];
acadoWorkspace.evHxd[2] = + Hx[8]*tmpd[0] + Hx[9]*tmpd[1] + Hx[10]*tmpd[2] + Hx[11]*tmpd[3];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 24. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(1.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(1.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(1.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(1.0000000000000001e-01);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(-1.0000000000000001e-01);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(1.0000000000000000e+00);

/* Compute outputs: */
out[0] = (((real_t)(1.0000000000000001e-01)*u[1])+xd[1]);
out[1] = (((real_t)(-1.0000000000000001e-01)*u[2])+xd[1]);
out[2] = (u[3]+xd[3]);
out[3] = a[0];
out[4] = a[1];
out[5] = a[2];
out[6] = a[3];
out[7] = a[4];
out[8] = a[5];
out[9] = a[6];
out[10] = a[7];
out[11] = a[8];
out[12] = a[9];
out[13] = a[10];
out[14] = a[11];
out[15] = a[12];
out[16] = a[13];
out[17] = a[14];
out[18] = a[15];
out[19] = a[16];
out[20] = a[17];
out[21] = a[18];
out[22] = a[19];
out[23] = a[20];
out[24] = a[21];
out[25] = a[22];
out[26] = a[23];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.d[ 4 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 16 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 16 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 32 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.d[ 8 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGx[ 32 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 64 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 80 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGx[ 48 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 128 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 144 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.d[ 16 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGx[ 64 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.E[ 176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 208 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.E[ 224 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.d[ 20 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGx[ 80 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.E[ 256 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 272 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.E[ 304 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 320 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGx[ 96 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.E[ 352 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.E[ 368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 416 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 432 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.d[ 28 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGx[ 112 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.E[ 464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 496 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 512 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.E[ 528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 544 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.E[ 560 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.d[ 32 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGx[ 128 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.E[ 592 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 608 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.E[ 640 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.E[ 688 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.E[ 704 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.E[ 736 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.E[ 752 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 768 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.E[ 784 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.E[ 800 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.E[ 832 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.E[ 848 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 864 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.d[ 40 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 160 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 880 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.E[ 896 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.E[ 912 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.E[ 928 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.E[ 944 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.E[ 976 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.E[ 992 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.E[ 1008 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 1024 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.E[ 1040 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.d[ 44 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.evGx[ 176 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.E[ 1056 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.E[ 1072 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.E[ 1088 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.E[ 1104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.E[ 1120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1136 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.E[ 1152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.E[ 1168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.E[ 1184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.E[ 1216 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 176 ]), &(acadoWorkspace.E[ 1232 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.evGx[ 192 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.E[ 1248 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.E[ 1264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.E[ 1280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.E[ 1312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.E[ 1328 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.E[ 1344 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.E[ 1360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.E[ 1376 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1392 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.E[ 1408 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.E[ 1424 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.E[ 1440 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.d[ 52 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGx[ 208 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.E[ 1456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.E[ 1472 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.E[ 1488 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.E[ 1520 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.E[ 1536 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.E[ 1552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.E[ 1568 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.E[ 1584 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.E[ 1600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.E[ 1616 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.E[ 1632 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1648 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 208 ]), &(acadoWorkspace.E[ 1664 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.d[ 56 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.evGx[ 224 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.E[ 1680 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.E[ 1696 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.E[ 1712 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.E[ 1728 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.E[ 1744 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.E[ 1760 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.E[ 1776 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.E[ 1792 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.E[ 1808 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.E[ 1824 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.E[ 1840 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.E[ 1856 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.E[ 1872 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.E[ 1888 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 224 ]), &(acadoWorkspace.E[ 1904 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.d[ 60 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.evGx[ 240 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.E[ 1920 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.E[ 1936 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.E[ 1952 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.E[ 1968 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.E[ 1984 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.E[ 2000 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.E[ 2016 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.E[ 2032 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.E[ 2048 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.E[ 2064 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.E[ 2080 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.E[ 2096 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.E[ 2112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1888 ]), &(acadoWorkspace.E[ 2128 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1904 ]), &(acadoWorkspace.E[ 2144 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.E[ 2160 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.d[ 64 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.evGx[ 256 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.E[ 2176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.E[ 2192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.E[ 2208 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.E[ 2224 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.E[ 2240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.E[ 2256 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.E[ 2272 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.E[ 2288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.E[ 2304 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.E[ 2320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.E[ 2336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.E[ 2352 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.E[ 2368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.E[ 2384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2144 ]), &(acadoWorkspace.E[ 2400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2416 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 256 ]), &(acadoWorkspace.E[ 2432 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.d[ 68 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGx[ 272 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.E[ 2448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.E[ 2464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.E[ 2480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.E[ 2496 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.E[ 2512 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.E[ 2528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.E[ 2544 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.E[ 2560 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.E[ 2576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.E[ 2592 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.E[ 2608 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.E[ 2624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.E[ 2640 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.E[ 2656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.E[ 2672 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2416 ]), &(acadoWorkspace.E[ 2688 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2432 ]), &(acadoWorkspace.E[ 2704 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 272 ]), &(acadoWorkspace.E[ 2720 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.E[ 2736 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.E[ 2752 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.E[ 2768 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.E[ 2784 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.E[ 2800 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.E[ 2816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.E[ 2832 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.E[ 2848 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.E[ 2864 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.E[ 2880 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.E[ 2896 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.E[ 2912 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.E[ 2928 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.E[ 2944 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.E[ 2960 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.E[ 2976 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2704 ]), &(acadoWorkspace.E[ 2992 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2720 ]), &(acadoWorkspace.E[ 3008 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.E[ 3024 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.d[ 76 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 304 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.E[ 3040 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.E[ 3056 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.E[ 3072 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.E[ 3088 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.E[ 3104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.E[ 3120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.E[ 3136 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.E[ 3152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.E[ 3168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.E[ 3184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.E[ 3200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.E[ 3216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.E[ 3232 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.E[ 3248 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.E[ 3264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.E[ 3280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.E[ 3296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3008 ]), &(acadoWorkspace.E[ 3312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.E[ 3328 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 304 ]), &(acadoWorkspace.E[ 3344 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1088 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1696 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1712 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1744 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1888 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1904 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 1936 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 1952 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 1984 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2144 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2176 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2224 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2416 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2432 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2464 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2480 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2512 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2704 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2720 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2736 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2752 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2768 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2800 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 3008 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3040 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3056 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3088 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3104 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3136 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3152 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3168 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3184 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3200 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3216 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3232 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3248 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3264 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QE[ 3280 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.QE[ 3296 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3312 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3328 ]), &(acadoWorkspace.QE[ 3328 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3344 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 448 ]), &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 880 ]), &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1456 ]), &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2176 ]), &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2736 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 3040 ]), &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 256 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 464 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 736 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 896 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1072 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1264 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1472 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1696 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1936 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2192 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2464 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2752 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3056 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 272 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 368 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 752 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1088 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1280 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1712 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1952 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2480 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2768 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3072 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 928 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1504 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2224 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2784 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3088 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 224 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 304 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 640 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 944 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1120 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1312 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1520 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1744 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1984 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2240 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2512 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2800 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3104 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 656 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1136 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1328 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1760 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2000 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2528 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2816 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3120 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 544 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 976 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1552 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2272 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2544 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2832 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3136 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 560 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 992 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1168 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1360 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1568 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1792 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2032 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2288 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2560 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2848 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3152 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 848 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1184 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1376 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1808 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2048 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2576 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2864 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3168 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1024 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1600 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2320 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2592 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2880 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3184 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1040 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1216 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1408 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1616 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1840 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2080 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2336 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2608 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2896 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3200 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 160 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1232 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1424 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1856 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2096 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2624 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2912 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3216 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 176 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1648 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2368 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2640 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2928 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3232 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1664 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1888 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2128 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2384 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2656 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2944 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3248 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 208 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 224 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1904 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 224 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2144 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 224 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 224 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2672 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 224 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2960 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 224 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3264 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 224 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2416 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2688 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2976 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3280 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 256 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2432 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 256 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2704 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 256 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2992 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 256 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3296 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 256 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 272 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2720 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 272 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3008 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 272 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3312 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 272 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3024 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3328 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 304 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3344 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 304 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2176 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2736 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3040 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1696 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1936 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2192 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2464 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2752 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3056 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1088 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1712 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1952 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2480 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2768 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3072 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2224 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3088 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1744 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1984 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2240 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2512 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2800 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3104 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3120 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 0, 12 );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 0, 13 );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 0, 14 );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 0, 15 );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 0, 16 );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 0, 17 );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 0, 18 );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 0, 19 );
acado_setBlockH11( 0, 19, &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1696 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 1936 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2192 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2464 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2752 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3056 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1088 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1712 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 1952 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2480 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2768 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3072 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2224 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3088 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1744 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 1984 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2240 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2512 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2800 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3104 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3120 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 1, 12 );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 1, 13 );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 1, 14 );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 1, 15 );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 1, 16 );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 1, 17 );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 1, 18 );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 1, 19 );
acado_setBlockH11( 1, 19, &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1088 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1712 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 1952 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2480 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2768 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3072 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2224 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3088 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1744 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 1984 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2240 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2512 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2800 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3104 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3120 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 2, 12 );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 2, 13 );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 2, 14 );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 2, 15 );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 2, 16 );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 2, 17 );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 2, 18 );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 2, 19 );
acado_setBlockH11( 2, 19, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2224 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3088 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1744 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1984 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2240 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2512 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2800 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3104 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3120 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 3, 12 );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 3, 13 );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 3, 14 );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 3, 15 );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 3, 16 );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 3, 17 );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 3, 18 );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 3, 19 );
acado_setBlockH11( 3, 19, &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1744 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 1984 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2240 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2512 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2800 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3104 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3120 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 4, 12 );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 4, 13 );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 4, 14 );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 4, 15 );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 4, 16 );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 4, 17 );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 4, 18 );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 4, 19 );
acado_setBlockH11( 4, 19, &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1760 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2000 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2528 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2816 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3120 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 5, 12 );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 5, 13 );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 5, 14 );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 5, 15 );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 5, 16 );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 5, 17 );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 5, 18 );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 5, 19 );
acado_setBlockH11( 5, 19, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2272 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3136 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 6, 12 );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 6, 13 );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 6, 14 );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 6, 15 );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 6, 16 );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 6, 17 );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 6, 18 );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 6, 19 );
acado_setBlockH11( 6, 19, &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1792 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2032 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2288 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2560 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2848 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3152 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 7, 12 );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 7, 13 );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 7, 14 );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 7, 15 );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 7, 16 );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 7, 17 );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 7, 18 );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 7, 19 );
acado_setBlockH11( 7, 19, &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1808 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2048 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2576 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2864 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 8, 12 );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 8, 13 );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 8, 14 );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 8, 15 );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 8, 16 );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 8, 17 );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 8, 18 );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 8, 19 );
acado_setBlockH11( 8, 19, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2320 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3184 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 9, 12 );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 9, 13 );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 9, 14 );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 9, 15 );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 9, 16 );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 9, 17 );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 9, 18 );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 9, 19 );
acado_setBlockH11( 9, 19, &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 10, 10 );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QE[ 1840 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2080 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2336 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2608 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2896 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3200 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 10, 12 );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 10, 13 );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 10, 14 );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 10, 15 );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 10, 16 );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 10, 17 );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 10, 18 );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 10, 19 );
acado_setBlockH11( 10, 19, &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 11, 11 );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.QE[ 1856 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QE[ 2096 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2624 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2912 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 11, 12 );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 11, 13 );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 11, 14 );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 11, 15 );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 11, 16 );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 11, 17 );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 11, 18 );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 11, 19 );
acado_setBlockH11( 11, 19, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 12, 12 );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QE[ 2368 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3232 ]) );

acado_zeroBlockH11( 12, 13 );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 12, 14 );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 12, 15 );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 12, 16 );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 12, 17 );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 12, 18 );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 12, 19 );
acado_setBlockH11( 12, 19, &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 13, 13 );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1888 ]), &(acadoWorkspace.QE[ 1888 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.QE[ 2128 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.QE[ 2384 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QE[ 2656 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 2944 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3248 ]) );

acado_zeroBlockH11( 13, 14 );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1888 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 13, 15 );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 13, 16 );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 13, 17 );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 13, 18 );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 13, 19 );
acado_setBlockH11( 13, 19, &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 14, 14 );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1904 ]), &(acadoWorkspace.QE[ 1904 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2144 ]), &(acadoWorkspace.QE[ 2144 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.QE[ 2672 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QE[ 2960 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 14, 15 );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2144 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 14, 16 );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 14, 17 );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 14, 18 );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 14, 19 );
acado_setBlockH11( 14, 19, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 15, 15 );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2416 ]), &(acadoWorkspace.QE[ 2416 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QE[ 3280 ]) );

acado_zeroBlockH11( 15, 16 );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2416 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 15, 17 );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 15, 18 );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 15, 19 );
acado_setBlockH11( 15, 19, &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 16, 16 );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2432 ]), &(acadoWorkspace.QE[ 2432 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2704 ]), &(acadoWorkspace.QE[ 2704 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.QE[ 2992 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.QE[ 3296 ]) );

acado_zeroBlockH11( 16, 17 );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2704 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 16, 18 );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 16, 19 );
acado_setBlockH11( 16, 19, &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 17, 17 );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2720 ]), &(acadoWorkspace.QE[ 2720 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 3008 ]), &(acadoWorkspace.QE[ 3008 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 17, 18 );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 3008 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 17, 19 );
acado_setBlockH11( 17, 19, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 18, 18 );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 3328 ]), &(acadoWorkspace.QE[ 3328 ]) );

acado_zeroBlockH11( 18, 19 );
acado_setBlockH11( 18, 19, &(acadoWorkspace.E[ 3328 ]), &(acadoWorkspace.QE[ 3344 ]) );

acado_setBlockH11_R1( 19, 19 );
acado_setBlockH11( 19, 19, &(acadoWorkspace.E[ 3344 ]), &(acadoWorkspace.QE[ 3344 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );
acado_copyHTH( 10, 0 );
acado_copyHTH( 10, 1 );
acado_copyHTH( 10, 2 );
acado_copyHTH( 10, 3 );
acado_copyHTH( 10, 4 );
acado_copyHTH( 10, 5 );
acado_copyHTH( 10, 6 );
acado_copyHTH( 10, 7 );
acado_copyHTH( 10, 8 );
acado_copyHTH( 10, 9 );
acado_copyHTH( 11, 0 );
acado_copyHTH( 11, 1 );
acado_copyHTH( 11, 2 );
acado_copyHTH( 11, 3 );
acado_copyHTH( 11, 4 );
acado_copyHTH( 11, 5 );
acado_copyHTH( 11, 6 );
acado_copyHTH( 11, 7 );
acado_copyHTH( 11, 8 );
acado_copyHTH( 11, 9 );
acado_copyHTH( 11, 10 );
acado_copyHTH( 12, 0 );
acado_copyHTH( 12, 1 );
acado_copyHTH( 12, 2 );
acado_copyHTH( 12, 3 );
acado_copyHTH( 12, 4 );
acado_copyHTH( 12, 5 );
acado_copyHTH( 12, 6 );
acado_copyHTH( 12, 7 );
acado_copyHTH( 12, 8 );
acado_copyHTH( 12, 9 );
acado_copyHTH( 12, 10 );
acado_copyHTH( 12, 11 );
acado_copyHTH( 13, 0 );
acado_copyHTH( 13, 1 );
acado_copyHTH( 13, 2 );
acado_copyHTH( 13, 3 );
acado_copyHTH( 13, 4 );
acado_copyHTH( 13, 5 );
acado_copyHTH( 13, 6 );
acado_copyHTH( 13, 7 );
acado_copyHTH( 13, 8 );
acado_copyHTH( 13, 9 );
acado_copyHTH( 13, 10 );
acado_copyHTH( 13, 11 );
acado_copyHTH( 13, 12 );
acado_copyHTH( 14, 0 );
acado_copyHTH( 14, 1 );
acado_copyHTH( 14, 2 );
acado_copyHTH( 14, 3 );
acado_copyHTH( 14, 4 );
acado_copyHTH( 14, 5 );
acado_copyHTH( 14, 6 );
acado_copyHTH( 14, 7 );
acado_copyHTH( 14, 8 );
acado_copyHTH( 14, 9 );
acado_copyHTH( 14, 10 );
acado_copyHTH( 14, 11 );
acado_copyHTH( 14, 12 );
acado_copyHTH( 14, 13 );
acado_copyHTH( 15, 0 );
acado_copyHTH( 15, 1 );
acado_copyHTH( 15, 2 );
acado_copyHTH( 15, 3 );
acado_copyHTH( 15, 4 );
acado_copyHTH( 15, 5 );
acado_copyHTH( 15, 6 );
acado_copyHTH( 15, 7 );
acado_copyHTH( 15, 8 );
acado_copyHTH( 15, 9 );
acado_copyHTH( 15, 10 );
acado_copyHTH( 15, 11 );
acado_copyHTH( 15, 12 );
acado_copyHTH( 15, 13 );
acado_copyHTH( 15, 14 );
acado_copyHTH( 16, 0 );
acado_copyHTH( 16, 1 );
acado_copyHTH( 16, 2 );
acado_copyHTH( 16, 3 );
acado_copyHTH( 16, 4 );
acado_copyHTH( 16, 5 );
acado_copyHTH( 16, 6 );
acado_copyHTH( 16, 7 );
acado_copyHTH( 16, 8 );
acado_copyHTH( 16, 9 );
acado_copyHTH( 16, 10 );
acado_copyHTH( 16, 11 );
acado_copyHTH( 16, 12 );
acado_copyHTH( 16, 13 );
acado_copyHTH( 16, 14 );
acado_copyHTH( 16, 15 );
acado_copyHTH( 17, 0 );
acado_copyHTH( 17, 1 );
acado_copyHTH( 17, 2 );
acado_copyHTH( 17, 3 );
acado_copyHTH( 17, 4 );
acado_copyHTH( 17, 5 );
acado_copyHTH( 17, 6 );
acado_copyHTH( 17, 7 );
acado_copyHTH( 17, 8 );
acado_copyHTH( 17, 9 );
acado_copyHTH( 17, 10 );
acado_copyHTH( 17, 11 );
acado_copyHTH( 17, 12 );
acado_copyHTH( 17, 13 );
acado_copyHTH( 17, 14 );
acado_copyHTH( 17, 15 );
acado_copyHTH( 17, 16 );
acado_copyHTH( 18, 0 );
acado_copyHTH( 18, 1 );
acado_copyHTH( 18, 2 );
acado_copyHTH( 18, 3 );
acado_copyHTH( 18, 4 );
acado_copyHTH( 18, 5 );
acado_copyHTH( 18, 6 );
acado_copyHTH( 18, 7 );
acado_copyHTH( 18, 8 );
acado_copyHTH( 18, 9 );
acado_copyHTH( 18, 10 );
acado_copyHTH( 18, 11 );
acado_copyHTH( 18, 12 );
acado_copyHTH( 18, 13 );
acado_copyHTH( 18, 14 );
acado_copyHTH( 18, 15 );
acado_copyHTH( 18, 16 );
acado_copyHTH( 18, 17 );
acado_copyHTH( 19, 0 );
acado_copyHTH( 19, 1 );
acado_copyHTH( 19, 2 );
acado_copyHTH( 19, 3 );
acado_copyHTH( 19, 4 );
acado_copyHTH( 19, 5 );
acado_copyHTH( 19, 6 );
acado_copyHTH( 19, 7 );
acado_copyHTH( 19, 8 );
acado_copyHTH( 19, 9 );
acado_copyHTH( 19, 10 );
acado_copyHTH( 19, 11 );
acado_copyHTH( 19, 12 );
acado_copyHTH( 19, 13 );
acado_copyHTH( 19, 14 );
acado_copyHTH( 19, 15 );
acado_copyHTH( 19, 16 );
acado_copyHTH( 19, 17 );
acado_copyHTH( 19, 18 );

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.Qd[ 44 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.Qd[ 52 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.Qd[ 68 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.Qd[ 76 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 16 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 160 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 448 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 880 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1056 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1248 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1456 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1680 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1920 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2176 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2448 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2736 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 3040 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 256 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 464 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 736 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 896 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1072 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1264 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1472 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1696 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1936 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2192 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2464 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2752 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3056 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 272 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 368 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 752 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1088 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1280 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1712 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1952 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2480 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2768 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3072 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 928 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1504 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2224 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2784 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3088 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 224 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 304 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 640 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 944 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1120 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1312 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1520 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1744 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1984 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2240 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2512 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2800 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3104 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 656 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1136 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1328 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1760 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2000 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2528 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2816 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3120 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 544 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 976 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1552 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2272 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2544 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2832 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3136 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 560 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 992 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1168 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1360 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1568 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1792 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2032 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2288 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2560 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2848 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3152 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 848 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1184 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1376 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1808 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2048 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2576 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2864 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3168 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1024 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1600 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2320 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2592 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2880 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3184 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1040 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1216 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1408 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1616 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1840 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2080 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2336 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2608 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2896 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3200 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1232 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1424 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1856 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2096 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2624 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2912 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3216 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1648 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2368 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2640 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2928 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3232 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1664 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1888 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2128 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2384 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2656 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2944 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3248 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1904 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2144 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2672 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2960 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3264 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2416 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2688 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2976 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3280 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2432 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2704 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2992 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3296 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2720 ]), &(acadoWorkspace.g[ 68 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3008 ]), &(acadoWorkspace.g[ 68 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3312 ]), &(acadoWorkspace.g[ 68 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3024 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3328 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3344 ]), &(acadoWorkspace.g[ 76 ]) );
acadoWorkspace.lb[0] = - acadoVariables.u[0];
acadoWorkspace.lb[1] = - acadoVariables.u[1];
acadoWorkspace.lb[2] = - acadoVariables.u[2];
acadoWorkspace.lb[3] = - acadoVariables.u[3];
acadoWorkspace.lb[4] = - acadoVariables.u[4];
acadoWorkspace.lb[5] = - acadoVariables.u[5];
acadoWorkspace.lb[6] = - acadoVariables.u[6];
acadoWorkspace.lb[7] = - acadoVariables.u[7];
acadoWorkspace.lb[8] = - acadoVariables.u[8];
acadoWorkspace.lb[9] = - acadoVariables.u[9];
acadoWorkspace.lb[10] = - acadoVariables.u[10];
acadoWorkspace.lb[11] = - acadoVariables.u[11];
acadoWorkspace.lb[12] = - acadoVariables.u[12];
acadoWorkspace.lb[13] = - acadoVariables.u[13];
acadoWorkspace.lb[14] = - acadoVariables.u[14];
acadoWorkspace.lb[15] = - acadoVariables.u[15];
acadoWorkspace.lb[16] = - acadoVariables.u[16];
acadoWorkspace.lb[17] = - acadoVariables.u[17];
acadoWorkspace.lb[18] = - acadoVariables.u[18];
acadoWorkspace.lb[19] = - acadoVariables.u[19];
acadoWorkspace.lb[20] = - acadoVariables.u[20];
acadoWorkspace.lb[21] = - acadoVariables.u[21];
acadoWorkspace.lb[22] = - acadoVariables.u[22];
acadoWorkspace.lb[23] = - acadoVariables.u[23];
acadoWorkspace.lb[24] = - acadoVariables.u[24];
acadoWorkspace.lb[25] = - acadoVariables.u[25];
acadoWorkspace.lb[26] = - acadoVariables.u[26];
acadoWorkspace.lb[27] = - acadoVariables.u[27];
acadoWorkspace.lb[28] = - acadoVariables.u[28];
acadoWorkspace.lb[29] = - acadoVariables.u[29];
acadoWorkspace.lb[30] = - acadoVariables.u[30];
acadoWorkspace.lb[31] = - acadoVariables.u[31];
acadoWorkspace.lb[32] = - acadoVariables.u[32];
acadoWorkspace.lb[33] = - acadoVariables.u[33];
acadoWorkspace.lb[34] = - acadoVariables.u[34];
acadoWorkspace.lb[35] = - acadoVariables.u[35];
acadoWorkspace.lb[36] = - acadoVariables.u[36];
acadoWorkspace.lb[37] = - acadoVariables.u[37];
acadoWorkspace.lb[38] = - acadoVariables.u[38];
acadoWorkspace.lb[39] = - acadoVariables.u[39];
acadoWorkspace.lb[40] = - acadoVariables.u[40];
acadoWorkspace.lb[41] = - acadoVariables.u[41];
acadoWorkspace.lb[42] = - acadoVariables.u[42];
acadoWorkspace.lb[43] = - acadoVariables.u[43];
acadoWorkspace.lb[44] = - acadoVariables.u[44];
acadoWorkspace.lb[45] = - acadoVariables.u[45];
acadoWorkspace.lb[46] = - acadoVariables.u[46];
acadoWorkspace.lb[47] = - acadoVariables.u[47];
acadoWorkspace.lb[48] = - acadoVariables.u[48];
acadoWorkspace.lb[49] = - acadoVariables.u[49];
acadoWorkspace.lb[50] = - acadoVariables.u[50];
acadoWorkspace.lb[51] = - acadoVariables.u[51];
acadoWorkspace.lb[52] = - acadoVariables.u[52];
acadoWorkspace.lb[53] = - acadoVariables.u[53];
acadoWorkspace.lb[54] = - acadoVariables.u[54];
acadoWorkspace.lb[55] = - acadoVariables.u[55];
acadoWorkspace.lb[56] = - acadoVariables.u[56];
acadoWorkspace.lb[57] = - acadoVariables.u[57];
acadoWorkspace.lb[58] = - acadoVariables.u[58];
acadoWorkspace.lb[59] = - acadoVariables.u[59];
acadoWorkspace.lb[60] = - acadoVariables.u[60];
acadoWorkspace.lb[61] = - acadoVariables.u[61];
acadoWorkspace.lb[62] = - acadoVariables.u[62];
acadoWorkspace.lb[63] = - acadoVariables.u[63];
acadoWorkspace.lb[64] = - acadoVariables.u[64];
acadoWorkspace.lb[65] = - acadoVariables.u[65];
acadoWorkspace.lb[66] = - acadoVariables.u[66];
acadoWorkspace.lb[67] = - acadoVariables.u[67];
acadoWorkspace.lb[68] = - acadoVariables.u[68];
acadoWorkspace.lb[69] = - acadoVariables.u[69];
acadoWorkspace.lb[70] = - acadoVariables.u[70];
acadoWorkspace.lb[71] = - acadoVariables.u[71];
acadoWorkspace.lb[72] = - acadoVariables.u[72];
acadoWorkspace.lb[73] = - acadoVariables.u[73];
acadoWorkspace.lb[74] = - acadoVariables.u[74];
acadoWorkspace.lb[75] = - acadoVariables.u[75];
acadoWorkspace.lb[76] = - acadoVariables.u[76];
acadoWorkspace.lb[77] = - acadoVariables.u[77];
acadoWorkspace.lb[78] = - acadoVariables.u[78];
acadoWorkspace.lb[79] = - acadoVariables.u[79];
acadoWorkspace.ub[0] = (real_t)9.0000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)9.0000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)9.0000000000000002e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)9.0000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)9.0000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)9.0000000000000002e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)9.0000000000000002e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)9.0000000000000002e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)9.0000000000000002e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+12 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)9.0000000000000002e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+12 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)9.0000000000000002e-01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)9.0000000000000002e-01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.0000000000000000e+12 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+12 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)9.0000000000000002e-01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)1.0000000000000000e+12 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.0000000000000000e+12 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)9.0000000000000002e-01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)1.0000000000000000e+12 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+12 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)9.0000000000000002e-01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)1.0000000000000000e+12 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+12 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)9.0000000000000002e-01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)1.0000000000000000e+12 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.0000000000000000e+12 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)9.0000000000000002e-01 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)1.0000000000000000e+12 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.0000000000000000e+12 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.0000000000000000e+12 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)9.0000000000000002e-01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)1.0000000000000000e+12 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+12 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.0000000000000000e+12 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)9.0000000000000002e-01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)1.0000000000000000e+12 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+12 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)1.0000000000000000e+12 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)9.0000000000000002e-01 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)1.0000000000000000e+12 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)1.0000000000000000e+12 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)1.0000000000000000e+12 - acadoVariables.u[79];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun1 * 4 + 3];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 3] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 3 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 3 + 2] = acadoWorkspace.conValueOut[2];

acadoWorkspace.evHx[lRun1 * 12] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 12 + 1] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 12 + 2] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 12 + 3] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 12 + 4] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 12 + 5] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 12 + 6] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 12 + 7] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 12 + 8] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 12 + 9] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 12 + 10] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 12 + 11] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHu[lRun1 * 12] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHu[lRun1 * 12 + 1] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHu[lRun1 * 12 + 2] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHu[lRun1 * 12 + 3] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHu[lRun1 * 12 + 4] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHu[lRun1 * 12 + 5] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHu[lRun1 * 12 + 6] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHu[lRun1 * 12 + 7] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHu[lRun1 * 12 + 8] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHu[lRun1 * 12 + 9] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHu[lRun1 * 12 + 10] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHu[lRun1 * 12 + 11] = acadoWorkspace.conValueOut[26];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];

acado_multHxC( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 12 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.A01[ 24 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.A01[ 48 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.A01[ 132 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.A01[ 144 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.A01[ 156 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.A01[ 168 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.A01[ 192 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.A01[ 204 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.A01[ 216 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A01[ 228 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 16 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 32 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 48 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 64 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 80 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 96 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 112 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 128 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 144 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 160 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 176 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 192 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 208 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 224 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 240 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 256 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 272 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 288 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 304 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 320 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 336 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 352 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 368 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 384 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 400 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 416 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 432 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 448 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 464 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 480 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 496 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 512 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 528 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 544 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 560 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 576 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 592 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 608 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 624 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 640 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 656 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 672 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 688 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 704 ]), 9, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 720 ]), 10, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 736 ]), 10, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 752 ]), 10, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 768 ]), 10, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 784 ]), 10, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 800 ]), 10, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 816 ]), 10, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 832 ]), 10, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 848 ]), 10, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 864 ]), 10, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 880 ]), 11, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 896 ]), 11, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 912 ]), 11, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 928 ]), 11, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 944 ]), 11, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 960 ]), 11, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 976 ]), 11, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 992 ]), 11, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 1008 ]), 11, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 1024 ]), 11, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 1040 ]), 11, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1056 ]), 12, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1072 ]), 12, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1088 ]), 12, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1104 ]), 12, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1120 ]), 12, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1136 ]), 12, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1152 ]), 12, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1168 ]), 12, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1184 ]), 12, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1200 ]), 12, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1216 ]), 12, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 1232 ]), 12, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1248 ]), 13, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1264 ]), 13, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1280 ]), 13, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1296 ]), 13, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1312 ]), 13, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1328 ]), 13, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1344 ]), 13, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1360 ]), 13, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1376 ]), 13, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1392 ]), 13, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1408 ]), 13, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1424 ]), 13, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 1440 ]), 13, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1456 ]), 14, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1472 ]), 14, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1488 ]), 14, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1504 ]), 14, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1520 ]), 14, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1536 ]), 14, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1552 ]), 14, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1568 ]), 14, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1584 ]), 14, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1600 ]), 14, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1616 ]), 14, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1632 ]), 14, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1648 ]), 14, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 1664 ]), 14, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1680 ]), 15, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1696 ]), 15, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1712 ]), 15, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1728 ]), 15, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1744 ]), 15, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1760 ]), 15, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1776 ]), 15, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1792 ]), 15, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1808 ]), 15, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1824 ]), 15, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1840 ]), 15, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1856 ]), 15, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1872 ]), 15, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1888 ]), 15, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1904 ]), 15, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 1920 ]), 16, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 1936 ]), 16, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 1952 ]), 16, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 1968 ]), 16, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 1984 ]), 16, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2000 ]), 16, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2016 ]), 16, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2032 ]), 16, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2048 ]), 16, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2064 ]), 16, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2080 ]), 16, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2096 ]), 16, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2112 ]), 16, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2128 ]), 16, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2144 ]), 16, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 2160 ]), 16, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2176 ]), 17, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2192 ]), 17, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2208 ]), 17, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2224 ]), 17, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2240 ]), 17, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2256 ]), 17, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2272 ]), 17, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2288 ]), 17, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2304 ]), 17, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2320 ]), 17, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2336 ]), 17, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2352 ]), 17, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2368 ]), 17, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2384 ]), 17, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2400 ]), 17, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2416 ]), 17, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 2432 ]), 17, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2448 ]), 18, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2464 ]), 18, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2480 ]), 18, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2496 ]), 18, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2512 ]), 18, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2528 ]), 18, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2544 ]), 18, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2560 ]), 18, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2576 ]), 18, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2592 ]), 18, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2608 ]), 18, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2624 ]), 18, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2640 ]), 18, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2656 ]), 18, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2672 ]), 18, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2688 ]), 18, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2704 ]), 18, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 2720 ]), 18, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2736 ]), 19, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2752 ]), 19, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2768 ]), 19, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2784 ]), 19, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2800 ]), 19, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2816 ]), 19, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2832 ]), 19, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2848 ]), 19, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2864 ]), 19, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2880 ]), 19, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2896 ]), 19, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2912 ]), 19, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2928 ]), 19, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2944 ]), 19, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2960 ]), 19, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2976 ]), 19, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 2992 ]), 19, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 3008 ]), 19, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 3024 ]), 19, 18 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3] = acadoWorkspace.evHu[3];
acadoWorkspace.A[80] = acadoWorkspace.evHu[4];
acadoWorkspace.A[81] = acadoWorkspace.evHu[5];
acadoWorkspace.A[82] = acadoWorkspace.evHu[6];
acadoWorkspace.A[83] = acadoWorkspace.evHu[7];
acadoWorkspace.A[160] = acadoWorkspace.evHu[8];
acadoWorkspace.A[161] = acadoWorkspace.evHu[9];
acadoWorkspace.A[162] = acadoWorkspace.evHu[10];
acadoWorkspace.A[163] = acadoWorkspace.evHu[11];
acadoWorkspace.A[244] = acadoWorkspace.evHu[12];
acadoWorkspace.A[245] = acadoWorkspace.evHu[13];
acadoWorkspace.A[246] = acadoWorkspace.evHu[14];
acadoWorkspace.A[247] = acadoWorkspace.evHu[15];
acadoWorkspace.A[324] = acadoWorkspace.evHu[16];
acadoWorkspace.A[325] = acadoWorkspace.evHu[17];
acadoWorkspace.A[326] = acadoWorkspace.evHu[18];
acadoWorkspace.A[327] = acadoWorkspace.evHu[19];
acadoWorkspace.A[404] = acadoWorkspace.evHu[20];
acadoWorkspace.A[405] = acadoWorkspace.evHu[21];
acadoWorkspace.A[406] = acadoWorkspace.evHu[22];
acadoWorkspace.A[407] = acadoWorkspace.evHu[23];
acadoWorkspace.A[488] = acadoWorkspace.evHu[24];
acadoWorkspace.A[489] = acadoWorkspace.evHu[25];
acadoWorkspace.A[490] = acadoWorkspace.evHu[26];
acadoWorkspace.A[491] = acadoWorkspace.evHu[27];
acadoWorkspace.A[568] = acadoWorkspace.evHu[28];
acadoWorkspace.A[569] = acadoWorkspace.evHu[29];
acadoWorkspace.A[570] = acadoWorkspace.evHu[30];
acadoWorkspace.A[571] = acadoWorkspace.evHu[31];
acadoWorkspace.A[648] = acadoWorkspace.evHu[32];
acadoWorkspace.A[649] = acadoWorkspace.evHu[33];
acadoWorkspace.A[650] = acadoWorkspace.evHu[34];
acadoWorkspace.A[651] = acadoWorkspace.evHu[35];
acadoWorkspace.A[732] = acadoWorkspace.evHu[36];
acadoWorkspace.A[733] = acadoWorkspace.evHu[37];
acadoWorkspace.A[734] = acadoWorkspace.evHu[38];
acadoWorkspace.A[735] = acadoWorkspace.evHu[39];
acadoWorkspace.A[812] = acadoWorkspace.evHu[40];
acadoWorkspace.A[813] = acadoWorkspace.evHu[41];
acadoWorkspace.A[814] = acadoWorkspace.evHu[42];
acadoWorkspace.A[815] = acadoWorkspace.evHu[43];
acadoWorkspace.A[892] = acadoWorkspace.evHu[44];
acadoWorkspace.A[893] = acadoWorkspace.evHu[45];
acadoWorkspace.A[894] = acadoWorkspace.evHu[46];
acadoWorkspace.A[895] = acadoWorkspace.evHu[47];
acadoWorkspace.A[976] = acadoWorkspace.evHu[48];
acadoWorkspace.A[977] = acadoWorkspace.evHu[49];
acadoWorkspace.A[978] = acadoWorkspace.evHu[50];
acadoWorkspace.A[979] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1056] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1057] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1058] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1059] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1136] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1137] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1138] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1139] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1220] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1221] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1222] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1223] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1300] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1301] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1302] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1303] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1380] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1381] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1382] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1383] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1464] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1465] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1466] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1467] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1544] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1545] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1546] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1547] = acadoWorkspace.evHu[79];
acadoWorkspace.A[1624] = acadoWorkspace.evHu[80];
acadoWorkspace.A[1625] = acadoWorkspace.evHu[81];
acadoWorkspace.A[1626] = acadoWorkspace.evHu[82];
acadoWorkspace.A[1627] = acadoWorkspace.evHu[83];
acadoWorkspace.A[1708] = acadoWorkspace.evHu[84];
acadoWorkspace.A[1709] = acadoWorkspace.evHu[85];
acadoWorkspace.A[1710] = acadoWorkspace.evHu[86];
acadoWorkspace.A[1711] = acadoWorkspace.evHu[87];
acadoWorkspace.A[1788] = acadoWorkspace.evHu[88];
acadoWorkspace.A[1789] = acadoWorkspace.evHu[89];
acadoWorkspace.A[1790] = acadoWorkspace.evHu[90];
acadoWorkspace.A[1791] = acadoWorkspace.evHu[91];
acadoWorkspace.A[1868] = acadoWorkspace.evHu[92];
acadoWorkspace.A[1869] = acadoWorkspace.evHu[93];
acadoWorkspace.A[1870] = acadoWorkspace.evHu[94];
acadoWorkspace.A[1871] = acadoWorkspace.evHu[95];
acadoWorkspace.A[1952] = acadoWorkspace.evHu[96];
acadoWorkspace.A[1953] = acadoWorkspace.evHu[97];
acadoWorkspace.A[1954] = acadoWorkspace.evHu[98];
acadoWorkspace.A[1955] = acadoWorkspace.evHu[99];
acadoWorkspace.A[2032] = acadoWorkspace.evHu[100];
acadoWorkspace.A[2033] = acadoWorkspace.evHu[101];
acadoWorkspace.A[2034] = acadoWorkspace.evHu[102];
acadoWorkspace.A[2035] = acadoWorkspace.evHu[103];
acadoWorkspace.A[2112] = acadoWorkspace.evHu[104];
acadoWorkspace.A[2113] = acadoWorkspace.evHu[105];
acadoWorkspace.A[2114] = acadoWorkspace.evHu[106];
acadoWorkspace.A[2115] = acadoWorkspace.evHu[107];
acadoWorkspace.A[2196] = acadoWorkspace.evHu[108];
acadoWorkspace.A[2197] = acadoWorkspace.evHu[109];
acadoWorkspace.A[2198] = acadoWorkspace.evHu[110];
acadoWorkspace.A[2199] = acadoWorkspace.evHu[111];
acadoWorkspace.A[2276] = acadoWorkspace.evHu[112];
acadoWorkspace.A[2277] = acadoWorkspace.evHu[113];
acadoWorkspace.A[2278] = acadoWorkspace.evHu[114];
acadoWorkspace.A[2279] = acadoWorkspace.evHu[115];
acadoWorkspace.A[2356] = acadoWorkspace.evHu[116];
acadoWorkspace.A[2357] = acadoWorkspace.evHu[117];
acadoWorkspace.A[2358] = acadoWorkspace.evHu[118];
acadoWorkspace.A[2359] = acadoWorkspace.evHu[119];
acadoWorkspace.A[2440] = acadoWorkspace.evHu[120];
acadoWorkspace.A[2441] = acadoWorkspace.evHu[121];
acadoWorkspace.A[2442] = acadoWorkspace.evHu[122];
acadoWorkspace.A[2443] = acadoWorkspace.evHu[123];
acadoWorkspace.A[2520] = acadoWorkspace.evHu[124];
acadoWorkspace.A[2521] = acadoWorkspace.evHu[125];
acadoWorkspace.A[2522] = acadoWorkspace.evHu[126];
acadoWorkspace.A[2523] = acadoWorkspace.evHu[127];
acadoWorkspace.A[2600] = acadoWorkspace.evHu[128];
acadoWorkspace.A[2601] = acadoWorkspace.evHu[129];
acadoWorkspace.A[2602] = acadoWorkspace.evHu[130];
acadoWorkspace.A[2603] = acadoWorkspace.evHu[131];
acadoWorkspace.A[2684] = acadoWorkspace.evHu[132];
acadoWorkspace.A[2685] = acadoWorkspace.evHu[133];
acadoWorkspace.A[2686] = acadoWorkspace.evHu[134];
acadoWorkspace.A[2687] = acadoWorkspace.evHu[135];
acadoWorkspace.A[2764] = acadoWorkspace.evHu[136];
acadoWorkspace.A[2765] = acadoWorkspace.evHu[137];
acadoWorkspace.A[2766] = acadoWorkspace.evHu[138];
acadoWorkspace.A[2767] = acadoWorkspace.evHu[139];
acadoWorkspace.A[2844] = acadoWorkspace.evHu[140];
acadoWorkspace.A[2845] = acadoWorkspace.evHu[141];
acadoWorkspace.A[2846] = acadoWorkspace.evHu[142];
acadoWorkspace.A[2847] = acadoWorkspace.evHu[143];
acadoWorkspace.A[2928] = acadoWorkspace.evHu[144];
acadoWorkspace.A[2929] = acadoWorkspace.evHu[145];
acadoWorkspace.A[2930] = acadoWorkspace.evHu[146];
acadoWorkspace.A[2931] = acadoWorkspace.evHu[147];
acadoWorkspace.A[3008] = acadoWorkspace.evHu[148];
acadoWorkspace.A[3009] = acadoWorkspace.evHu[149];
acadoWorkspace.A[3010] = acadoWorkspace.evHu[150];
acadoWorkspace.A[3011] = acadoWorkspace.evHu[151];
acadoWorkspace.A[3088] = acadoWorkspace.evHu[152];
acadoWorkspace.A[3089] = acadoWorkspace.evHu[153];
acadoWorkspace.A[3090] = acadoWorkspace.evHu[154];
acadoWorkspace.A[3091] = acadoWorkspace.evHu[155];
acadoWorkspace.A[3172] = acadoWorkspace.evHu[156];
acadoWorkspace.A[3173] = acadoWorkspace.evHu[157];
acadoWorkspace.A[3174] = acadoWorkspace.evHu[158];
acadoWorkspace.A[3175] = acadoWorkspace.evHu[159];
acadoWorkspace.A[3252] = acadoWorkspace.evHu[160];
acadoWorkspace.A[3253] = acadoWorkspace.evHu[161];
acadoWorkspace.A[3254] = acadoWorkspace.evHu[162];
acadoWorkspace.A[3255] = acadoWorkspace.evHu[163];
acadoWorkspace.A[3332] = acadoWorkspace.evHu[164];
acadoWorkspace.A[3333] = acadoWorkspace.evHu[165];
acadoWorkspace.A[3334] = acadoWorkspace.evHu[166];
acadoWorkspace.A[3335] = acadoWorkspace.evHu[167];
acadoWorkspace.A[3416] = acadoWorkspace.evHu[168];
acadoWorkspace.A[3417] = acadoWorkspace.evHu[169];
acadoWorkspace.A[3418] = acadoWorkspace.evHu[170];
acadoWorkspace.A[3419] = acadoWorkspace.evHu[171];
acadoWorkspace.A[3496] = acadoWorkspace.evHu[172];
acadoWorkspace.A[3497] = acadoWorkspace.evHu[173];
acadoWorkspace.A[3498] = acadoWorkspace.evHu[174];
acadoWorkspace.A[3499] = acadoWorkspace.evHu[175];
acadoWorkspace.A[3576] = acadoWorkspace.evHu[176];
acadoWorkspace.A[3577] = acadoWorkspace.evHu[177];
acadoWorkspace.A[3578] = acadoWorkspace.evHu[178];
acadoWorkspace.A[3579] = acadoWorkspace.evHu[179];
acadoWorkspace.A[3660] = acadoWorkspace.evHu[180];
acadoWorkspace.A[3661] = acadoWorkspace.evHu[181];
acadoWorkspace.A[3662] = acadoWorkspace.evHu[182];
acadoWorkspace.A[3663] = acadoWorkspace.evHu[183];
acadoWorkspace.A[3740] = acadoWorkspace.evHu[184];
acadoWorkspace.A[3741] = acadoWorkspace.evHu[185];
acadoWorkspace.A[3742] = acadoWorkspace.evHu[186];
acadoWorkspace.A[3743] = acadoWorkspace.evHu[187];
acadoWorkspace.A[3820] = acadoWorkspace.evHu[188];
acadoWorkspace.A[3821] = acadoWorkspace.evHu[189];
acadoWorkspace.A[3822] = acadoWorkspace.evHu[190];
acadoWorkspace.A[3823] = acadoWorkspace.evHu[191];
acadoWorkspace.A[3904] = acadoWorkspace.evHu[192];
acadoWorkspace.A[3905] = acadoWorkspace.evHu[193];
acadoWorkspace.A[3906] = acadoWorkspace.evHu[194];
acadoWorkspace.A[3907] = acadoWorkspace.evHu[195];
acadoWorkspace.A[3984] = acadoWorkspace.evHu[196];
acadoWorkspace.A[3985] = acadoWorkspace.evHu[197];
acadoWorkspace.A[3986] = acadoWorkspace.evHu[198];
acadoWorkspace.A[3987] = acadoWorkspace.evHu[199];
acadoWorkspace.A[4064] = acadoWorkspace.evHu[200];
acadoWorkspace.A[4065] = acadoWorkspace.evHu[201];
acadoWorkspace.A[4066] = acadoWorkspace.evHu[202];
acadoWorkspace.A[4067] = acadoWorkspace.evHu[203];
acadoWorkspace.A[4148] = acadoWorkspace.evHu[204];
acadoWorkspace.A[4149] = acadoWorkspace.evHu[205];
acadoWorkspace.A[4150] = acadoWorkspace.evHu[206];
acadoWorkspace.A[4151] = acadoWorkspace.evHu[207];
acadoWorkspace.A[4228] = acadoWorkspace.evHu[208];
acadoWorkspace.A[4229] = acadoWorkspace.evHu[209];
acadoWorkspace.A[4230] = acadoWorkspace.evHu[210];
acadoWorkspace.A[4231] = acadoWorkspace.evHu[211];
acadoWorkspace.A[4308] = acadoWorkspace.evHu[212];
acadoWorkspace.A[4309] = acadoWorkspace.evHu[213];
acadoWorkspace.A[4310] = acadoWorkspace.evHu[214];
acadoWorkspace.A[4311] = acadoWorkspace.evHu[215];
acadoWorkspace.A[4392] = acadoWorkspace.evHu[216];
acadoWorkspace.A[4393] = acadoWorkspace.evHu[217];
acadoWorkspace.A[4394] = acadoWorkspace.evHu[218];
acadoWorkspace.A[4395] = acadoWorkspace.evHu[219];
acadoWorkspace.A[4472] = acadoWorkspace.evHu[220];
acadoWorkspace.A[4473] = acadoWorkspace.evHu[221];
acadoWorkspace.A[4474] = acadoWorkspace.evHu[222];
acadoWorkspace.A[4475] = acadoWorkspace.evHu[223];
acadoWorkspace.A[4552] = acadoWorkspace.evHu[224];
acadoWorkspace.A[4553] = acadoWorkspace.evHu[225];
acadoWorkspace.A[4554] = acadoWorkspace.evHu[226];
acadoWorkspace.A[4555] = acadoWorkspace.evHu[227];
acadoWorkspace.A[4636] = acadoWorkspace.evHu[228];
acadoWorkspace.A[4637] = acadoWorkspace.evHu[229];
acadoWorkspace.A[4638] = acadoWorkspace.evHu[230];
acadoWorkspace.A[4639] = acadoWorkspace.evHu[231];
acadoWorkspace.A[4716] = acadoWorkspace.evHu[232];
acadoWorkspace.A[4717] = acadoWorkspace.evHu[233];
acadoWorkspace.A[4718] = acadoWorkspace.evHu[234];
acadoWorkspace.A[4719] = acadoWorkspace.evHu[235];
acadoWorkspace.A[4796] = acadoWorkspace.evHu[236];
acadoWorkspace.A[4797] = acadoWorkspace.evHu[237];
acadoWorkspace.A[4798] = acadoWorkspace.evHu[238];
acadoWorkspace.A[4799] = acadoWorkspace.evHu[239];
acadoWorkspace.lbA[0] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = (real_t)4.5000000000000000e+00 - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[59];

acadoWorkspace.ubA[0] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = (real_t)5.5000000000000000e+00 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[59];

acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 39 ]), &(acadoWorkspace.ubA[ 39 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.lbA[ 45 ]), &(acadoWorkspace.ubA[ 45 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.lbA[ 51 ]), &(acadoWorkspace.ubA[ 51 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.lbA[ 54 ]), &(acadoWorkspace.ubA[ 54 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.lbA[ 57 ]), &(acadoWorkspace.ubA[ 57 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.g[ 76 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.QDy[ 76 ]) );

acadoWorkspace.QDy[80] = + (real_t)1.0000000000000000e-03*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[81] = + (real_t)1.0000000000000000e-03*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[82] = + (real_t)1.0000000000000000e-03*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[83] = + (real_t)1.0000000000000000e-03*acadoWorkspace.DyN[3];

acadoWorkspace.QDy[4] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[79];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QDy[ 44 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2176 ]), &(acadoWorkspace.QDy[ 68 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QDy[ 76 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1888 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1904 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2144 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2416 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2432 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2704 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2720 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3008 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3328 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3344 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 76 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[1] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[2] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[3] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[4] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[5] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[6] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[7] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[8] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[9] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[10] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[11] += + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[12] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[13] += + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[14] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[15] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[16] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[17] += + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[18] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[19] += + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[20] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[21] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[22] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[23] += + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[24] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[25] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[26] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[27] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[28] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[29] += + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[30] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[31] += + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[32] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[33] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[34] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[35] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[36] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[37] += + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[38] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[39] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[40] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[41] += + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[42] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[43] += + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[44] += + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[45] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[46] += + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[47] += + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[48] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[49] += + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[50] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[51] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[52] += + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[53] += + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[54] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[55] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[56] += + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[57] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[58] += + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[59] += + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[60] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[61] += + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[62] += + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[63] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[64] += + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[65] += + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[66] += + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[67] += + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[68] += + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[69] += + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[70] += + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[71] += + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[72] += + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[73] += + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[74] += + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[75] += + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[76] += + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[77] += + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[78] += + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[79] += + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[3];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[3];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[59] -= acadoWorkspace.pacA01Dx0[59];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[59] -= acadoWorkspace.pacA01Dx0[59];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[0];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[6];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[7];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[8];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[9];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[10];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[11];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[12];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[13];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[14];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[15];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[16];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[17];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[18];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[19];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[20];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[21];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[22];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[23];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[24];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[25];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[26];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[27];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[28];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[29];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[30];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[31];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[32];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[33];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[34];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[35];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[36];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[37];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[38];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[39];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[40];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[41];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[42];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[43];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[44];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[45];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[46];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[47];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[48];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[49];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[50];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[51];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[52];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[53];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[54];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[55];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[56];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[57];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[58];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[59];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[60];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[61];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[62];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[63];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[64];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[65];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[66];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[67];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[68];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[69];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[70];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[71];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[72];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[73];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[74];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[75];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[76];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[77];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[78];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[79];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 16 ]), acadoWorkspace.x, &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), acadoWorkspace.x, &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 160 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), acadoWorkspace.x, &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 448 ]), acadoWorkspace.x, &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 880 ]), acadoWorkspace.x, &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1056 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1248 ]), acadoWorkspace.x, &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1456 ]), acadoWorkspace.x, &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1680 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1696 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1712 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1744 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1792 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1808 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1856 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1888 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1904 ]), &(acadoWorkspace.x[ 56 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1920 ]), acadoWorkspace.x, &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1936 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1952 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1984 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2032 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2048 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2096 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2128 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2144 ]), &(acadoWorkspace.x[ 56 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.x[ 60 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2176 ]), acadoWorkspace.x, &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2192 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2224 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2272 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2288 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2336 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2368 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2384 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.x[ 56 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2416 ]), &(acadoWorkspace.x[ 60 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2432 ]), &(acadoWorkspace.x[ 64 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2448 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2464 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2512 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2528 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2560 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2576 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2608 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2624 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2656 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2672 ]), &(acadoWorkspace.x[ 56 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.x[ 60 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2704 ]), &(acadoWorkspace.x[ 64 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2720 ]), &(acadoWorkspace.x[ 68 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2736 ]), acadoWorkspace.x, &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2752 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2768 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2816 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2848 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2864 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2896 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2912 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2944 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2960 ]), &(acadoWorkspace.x[ 56 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.x[ 60 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2992 ]), &(acadoWorkspace.x[ 64 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3008 ]), &(acadoWorkspace.x[ 68 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.x[ 72 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3040 ]), acadoWorkspace.x, &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3056 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3088 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3104 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3136 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3152 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3184 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3232 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3248 ]), &(acadoWorkspace.x[ 52 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.x[ 56 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.x[ 60 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3296 ]), &(acadoWorkspace.x[ 64 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.x[ 68 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3328 ]), &(acadoWorkspace.x[ 72 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3344 ]), &(acadoWorkspace.x[ 76 ]), &(acadoVariables.x[ 80 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[36] = acadoVariables.u[index * 4];
acadoWorkspace.state[37] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[38] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[39] = acadoVariables.u[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[80] = xEnd[0];
acadoVariables.x[81] = xEnd[1];
acadoVariables.x[82] = xEnd[2];
acadoVariables.x[83] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[80];
acadoWorkspace.state[1] = acadoVariables.x[81];
acadoWorkspace.state[2] = acadoVariables.x[82];
acadoWorkspace.state[3] = acadoVariables.x[83];
if (uEnd != 0)
{
acadoWorkspace.state[36] = uEnd[0];
acadoWorkspace.state[37] = uEnd[1];
acadoWorkspace.state[38] = uEnd[2];
acadoWorkspace.state[39] = uEnd[3];
}
else
{
acadoWorkspace.state[36] = acadoVariables.u[76];
acadoWorkspace.state[37] = acadoVariables.u[77];
acadoWorkspace.state[38] = acadoVariables.u[78];
acadoWorkspace.state[39] = acadoVariables.u[79];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[80] = acadoWorkspace.state[0];
acadoVariables.x[81] = acadoWorkspace.state[1];
acadoVariables.x[82] = acadoWorkspace.state[2];
acadoVariables.x[83] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[76] = uEnd[0];
acadoVariables.u[77] = uEnd[1];
acadoVariables.u[78] = uEnd[2];
acadoVariables.u[79] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79];
kkt = fabs( kkt );
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index + 80];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 8 */
real_t tmpDy[ 8 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 8] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 8];
acadoWorkspace.Dy[lRun1 * 8 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 8 + 1];
acadoWorkspace.Dy[lRun1 * 8 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 8 + 2];
acadoWorkspace.Dy[lRun1 * 8 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 8 + 3];
acadoWorkspace.Dy[lRun1 * 8 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 8 + 4];
acadoWorkspace.Dy[lRun1 * 8 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 8 + 5];
acadoWorkspace.Dy[lRun1 * 8 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 8 + 6];
acadoWorkspace.Dy[lRun1 * 8 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 8 + 7];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 8];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 8 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 8 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 8 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 8 + 4];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 8 + 5]*(real_t)1.0000000000000000e+04;
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 8 + 6]*(real_t)1.0000000000000000e+04;
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 8 + 7]*(real_t)1.0000000000000000e+04;
objVal += + acadoWorkspace.Dy[lRun1 * 8]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 8 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 8 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 8 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 8 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 8 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 8 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 8 + 7]*tmpDy[7];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e-03;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e-03;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e-03;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)1.0000000000000000e-03;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

