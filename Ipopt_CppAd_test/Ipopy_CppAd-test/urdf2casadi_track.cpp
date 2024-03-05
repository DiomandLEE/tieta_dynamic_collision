#include <iostream>
#include <cppad/ipopt/solve.hpp>

using namespace std;

namespace {

using CppAD::AD;
class FG_eval
{
	public:
	    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	    void operator()(ADvector& fg, const ADvector& x)
	    {
	        assert(fg.size() == 2);
	        assert(x.size() == 6);
	        // variables
	        AD<double> x1 = x[0];
	        AD<double> x2 = x[1];
	        AD<double> x3 = x[2];
	        AD<double> x4 = x[3];
            AD<double> x5 = x[4];
            AD<double> x6 = x[5];

            ADvector vars(6);
            vars[0] = x1;
            vars[1] = x2;
            vars[2] = x3;
            vars[3] = x4;
            vars[4] = x5;
            vars[5] = x6;

            ADvector res;
            casadi_f0(vars, res);

            AD<double> EE_x = res[12];
            AD<double> EE_y = res[13];

            fg[0] = 0;
            // f(x) objective function
            fg[0] += 1000 * CppAD::pow(EE_x - 0.6, 2);
            fg[0] += 1000 * CppAD::pow(EE_y - 0.3, 2);



                // constraints
                fg[1] = 0;
            // fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
            return;
	    }

    int casadi_f0(ADvector arg, ADvector& res) {
        AD<double> a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a3, a4, a5, a6, a7, a8, a9;
        a0= arg[0];
        a1=CppAD::cos(a0);
        a2=4.8965888601467475e-12;
        a3= arg[1];
        a4=CppAD::cos(a3);
        a5=(a2*a4);
        a3=CppAD::sin(a3);
        a5=(a5-a3);
        a6=(a1*a5);
        a7= arg[2];
        a8=CppAD::cos(a7);
        a9=(a6*a8);
        a10=(a2*a3);
        a10=(a10+a4);
        a11=(a1*a10);
        a7=CppAD::sin(a7);
        a12=(a11*a7);
        a9=(a9-a12);
        a12= arg[3];
        a13=CppAD::cos(a12);
        a14=(a2*a13);
        a12=CppAD::sin(a12);
        a14=(a14-a12);
        a15=(a9*a14);
        a6=(a6*a7);
        a16=(a11*a8);
        a6=(a6+a16);
        a16=(a2*a12);
        a16=(a13+a16);
        a17=(a6*a16);
        a15=(a15-a17);
        a17=arg[4];
        a18=CppAD::cos(a17);
        a19=(a15*a18);
        a0=CppAD::sin(a0);
        a17=CppAD::sin(a17);
        a20=(a0*a17);
        a19=(a19-a20);
        a20=arg[5];
        a21=CppAD::cos(a20);
        a22=(a19*a21);
        a23=(a2*a12);
        a23=(a23+a13);
        a9=(a9*a23);
        a13=(a2*a13);
        a13=(a13-a12);
        a12=(a6*a13);
        a9=(a9+a12);
        a20=CppAD::sin(a20);
        a12=(a9*a20);
        a22=(a22-a12);
        res.push_back(a22); // if (res[0]!=0) res[0][0]=a22;
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
        res.push_back(a25); // if (res[0]!=0) res[0][1]=a25;
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
        res.push_back(a25); // if (res[0]!=0) res[0][2]=a25;
        a25=0.;
        res.push_back(a25); // if (res[0]!=0) res[0][3]=a25;
        a15=(a15*a17);
        a26=(a0*a18);
        a15=(a15+a26);
        a26=(a2*a15);
        a19=(a19*a20);
        a23=(a9*a21);
        a19=(a19+a23);
        a26=(a26+a19);
        a26=(-a26);
        res.push_back(a26); // if (res[0]!=0) res[0][4]=a26;
        a18=(a1*a18);
        a12=(a12*a17);
        a18=(a18-a12);
        a12=(a2*a18);
        a24=(a24*a20);
        a26=(a22*a21);
        a24=(a24+a26);
        a12=(a12-a24);
        res.push_back(a12); // if (res[0]!=0) res[0][5]=a12;
        a14=(a14*a17);
        a17=(a2*a14);
        a21=(a13*a21);
        a16=(a16*a20);
        a21=(a21-a16);
        a17=(a17-a21);
        res.push_back(a17); //if (res[0]!=0) res[0][6]=a17;
        res.push_back(a25); //if (res[0]!=0) res[0][7]=a25;
        a19=(a2*a19);
        a19=(a19-a15);
        res.push_back(a19); //if (res[0]!=0) res[0][8]=a19;
        a24=(a2*a24);
        a24=(a18+a24);
        res.push_back(a24); //if (res[0]!=0) res[0][9]=a24;
        a2=(a2*a21);
        a2=(a14+a2);
        res.push_back(a2); //if (res[0]!=0) res[0][10]=a2;
        res.push_back(a25); //if (res[0]!=0) res[0][11]=a25;
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
        res.push_back(a9); //if (res[0]!=0) res[0][12]=a9;
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
        res.push_back(a18); //if (res[0]!=0) res[0][13]=a18;
        a6=(a6*a14);
        a25=(a25*a13);
        a2=(a2*a8);
        a21=(a21*a4);
        a4=8.9159000000000002e-02;
        a21=(a21+a4);
        a2=(a2+a21);
        a25=(a25+a2);
        a6=(a6+a25);
        res.push_back(a6); //if (res[0]!=0) res[0][14]=a6;
        a6=1.;
        res.push_back(a6); //if (res[0]!=0) res[0][15]=a6;
        return 0;
}
};

}

bool get_started(void)
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t nx = 6; // number of varibles
    size_t ng = 1; // number of constraints
    Dvector x0(nx); // initial condition of varibles
    x0[0] = 0.0;
    x0[1] = 0.0;
    x0[2] = 0.0;
    x0[3] = 0.0;
    x0[4] = 0.0;
    x0[5] = 0.0;


    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    for(i = 0; i < 6; i++)
    {
        xl[i] = -3.14;
        xu[i] = 3.14;
    }


    // lower and upper bounds for constraints
    // Dvector gl(ng), gu(ng);
    // for(i = 0; i < ng; i++)
    // {
    //     gl[i] = -1.0e19;
    //     gu[i] = 0.0;
    // }
    Dvector gl(ng), gu(ng);
    gl[0] = 0.0;    gu[0] = 0.0;

    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    string options;
    // turn off any printing
    //options += "Integer print_level  0\n";
    //options += "String sb            yes\n";
    // maximum iterations
    //options += "Retape  true\n";
    options += "Integer max_iter     30\n";
    //approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-11\n";
    //derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";


    CppAD::ipopt::solve_result<Dvector> solution; // solution
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, x0, xl, xu, gl, gu, fg_eval, solution);

    cout<<"solution: "<<solution.x<<endl;

    //
    //check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    //
    // double check_x[]  = {1.000000, 4.743000, 3.82115, 1.379408};
    // double check_zl[] = {1.087871, 0.,       0.,       0.      };
    // double check_zu[] = {0.,       0.,       0.,       0.      };
    // double rel_tol    = 1e-6; // relative tolerance
    // double abs_tol    = 1e-6; // absolute tolerance
    // for(i = 0; i < nx; i++)
    // {
    //     ok &= CppAD::NearEqual(check_x[i], solution.x[i], rel_tol, abs_tol);
    //     ok &= CppAD::NearEqual(check_zl[i], solution.zl[i], rel_tol, abs_tol);
    //     ok &= CppAD::NearEqual(check_zu[i], solution.zu[i], rel_tol, abs_tol);
    // }

    return ok;
}

int main()
{
    cout << "CppAD : Hello World Demo!" << endl;
    get_started();
    return 0;
}



