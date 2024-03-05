#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include <vector>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

using namespace std;
using CppAD::AD;
typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

int casadi_f0(casadi_real** arg, casadi_real** res) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=cos(a0);
  a2=4.8965888601467475e-12;
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a3=sin(a3);
  a5=(a5-a3);
  a6=(a1*a5);
  a7=arg[0]? arg[0][2] : 0;
  a8=cos(a7);
  a9=(a6*a8);
  a10=(a2*a3);
  a10=(a10+a4);
  a11=(a1*a10);
  a7=sin(a7);
  a12=(a11*a7);
  a9=(a9-a12);
  a12=arg[0]? arg[0][3] : 0;
  a13=cos(a12);
  a14=(a2*a13);
  a12=sin(a12);
  a14=(a14-a12);
  a15=(a9*a14);
  a6=(a6*a7);
  a16=(a11*a8);
  a6=(a6+a16);
  a16=(a2*a12);
  a16=(a13+a16);
  a17=(a6*a16);
  a15=(a15-a17);
  a17=arg[0]? arg[0][4] : 0;
  a18=cos(a17);
  a19=(a15*a18);
  a0=sin(a0);
  a17=sin(a17);
  a20=(a0*a17);
  a19=(a19-a20);
  a20=arg[0]? arg[0][5] : 0;
  a21=cos(a20);
  a22=(a19*a21);
  a23=(a2*a12);
  a23=(a23+a13);
  a9=(a9*a23);
  a13=(a2*a13);
  a13=(a13-a12);
  a12=(a6*a13);
  a9=(a9+a12);
  a20=sin(a20);
  a12=(a9*a20);
  a22=(a22-a12);
  if (res[0]!=0) res[0][0]=a22;
  a5=(a0*a5);
  a22=(a5*a8);
  a10=(a0*a10);
  a12=(a10*a7);
  a22=(a22-a12);
  a12=(a22*a14);
  a5=(a5*a7);
  a24=(a10*a8);
  a5=(a5+a24);
  a24=(a5*a16);
  a12=(a12-a24);
  a24=(a12*a18);
  a25=(a1*a17);
  a24=(a24+a25);
  a25=(a24*a21);
  a22=(a22*a23);
  a26=(a5*a13);
  a22=(a22+a26);
  a26=(a22*a20);
  a25=(a25-a26);
  if (res[0]!=0) res[0][1]=a25;
  a25=(a2*a3);
  a25=(a4+a25);
  a26=(a25*a8);
  a4=(a2*a4);
  a4=(a4-a3);
  a3=(a4*a7);
  a26=(a26+a3);
  a14=(a26*a14);
  a8=(a4*a8);
  a25=(a25*a7);
  a8=(a8-a25);
  a16=(a8*a16);
  a14=(a14+a16);
  a16=(a14*a18);
  a25=(a16*a21);
  a13=(a8*a13);
  a26=(a26*a23);
  a13=(a13-a26);
  a26=(a13*a20);
  a25=(a25+a26);
  a25=(-a25);
  if (res[0]!=0) res[0][2]=a25;
  a25=0.;
  if (res[0]!=0) res[0][3]=a25;
  a15=(a15*a17);
  a26=(a0*a18);
  a15=(a15+a26);
  a26=(a2*a15);
  a19=(a19*a20);
  a23=(a9*a21);
  a19=(a19+a23);
  a26=(a26+a19);
  a26=(-a26);
  if (res[0]!=0) res[0][4]=a26;
  a18=(a1*a18);
  a12=(a12*a17);
  a18=(a18-a12);
  a12=(a2*a18);
  a24=(a24*a20);
  a26=(a22*a21);
  a24=(a24+a26);
  a12=(a12-a24);
  if (res[0]!=0) res[0][5]=a12;
  a14=(a14*a17);
  a17=(a2*a14);
  a21=(a13*a21);
  a16=(a16*a20);
  a21=(a21-a16);
  a17=(a17-a21);
  if (res[0]!=0) res[0][6]=a17;
  if (res[0]!=0) res[0][7]=a25;
  a19=(a2*a19);
  a19=(a19-a15);
  if (res[0]!=0) res[0][8]=a19;
  a24=(a2*a24);
  a24=(a18+a24);
  if (res[0]!=0) res[0][9]=a24;
  a2=(a2*a21);
  a2=(a14+a2);
  if (res[0]!=0) res[0][10]=a2;
  if (res[0]!=0) res[0][11]=a25;
  a25=9.4649999999999998e-02;
  a9=(a25*a9);
  a2=3.9224999999999999e-01;
  a6=(a2*a6);
  a21=4.2499999999999999e-01;
  a11=(a21*a11);
  a24=-1.1970000000000000e-01;
  a19=(a24*a0);
  a11=(a11-a19);
  a19=1.3585000000000000e-01;
  a17=(a19*a0);
  a11=(a11-a17);
  a6=(a6+a11);
  a11=9.2999999999999999e-02;
  a0=(a11*a0);
  a6=(a6-a0);
  a9=(a9+a6);
  a6=8.2299999999999998e-02;
  a15=(a6*a15);
  a9=(a9-a15);
  if (res[0]!=0) res[0][12]=a9;
  a18=(a6*a18);
  a22=(a25*a22);
  a11=(a11*a1);
  a5=(a2*a5);
  a24=(a24*a1);
  a10=(a21*a10);
  a24=(a24+a10);
  a19=(a19*a1);
  a24=(a24+a19);
  a5=(a5+a24);
  a11=(a11+a5);
  a22=(a22+a11);
  a18=(a18+a22);
  if (res[0]!=0) res[0][13]=a18;
  a6=(a6*a14);
  a25=(a25*a13);
  a2=(a2*a8);
  a21=(a21*a4);
  a4=8.9159000000000002e-02;
  a21=(a21+a4);
  a2=(a2+a21);
  a25=(a25+a2);
  a6=(a6+a25);
  if (res[0]!=0) res[0][14]=a6;
  a6=1.;
  if (res[0]!=0) res[0][15]=a6;
  return 0;
}

int T_fk(casadi_real** arg, casadi_real** res){
  return casadi_f0(arg, res);
}

int main()
{
    casadi_real **arg;
    arg = new casadi_real*[1];
    arg[0] = new casadi_real[6];
    arg[0][0] = 0; // n_in
    arg[0][1] = 0; // n_out
    arg[0][2] = 0; // n_res
    arg[0][3] = 0; // n_jac
    arg[0][4] = 0; // n_hess
    arg[0][5] = 0; // n_fseed
    casadi_real **res;
    res = new casadi_real*[1];
    res[0] = new casadi_real[16];
    T_fk(arg, res);
    cout << "T_fk_X: " << res[0][12] << endl;
    cout << "T_fk_y: " << res[0][13] << endl;
    cout << "T_fk_z: " << res[0][14] << endl;
    cout << "T_fk_1: " << res[0][15] << endl;
    //cout << "T_fk: " << res[0][16] << endl;
    return 0;
}