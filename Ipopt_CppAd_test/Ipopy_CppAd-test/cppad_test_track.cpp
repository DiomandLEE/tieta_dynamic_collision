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
	        assert(x.size() == 10);
	        // variables
	        AD<double> x1 = x[0];
	        AD<double> x2 = x[1];
	        AD<double> x3 = x[2];
	        AD<double> x4 = x[3];
            AD<double> x5 = x[4];

            AD<double> y1 = x[5];
	        AD<double> y2 = x[6];
	        AD<double> y3 = x[7];
	        AD<double> y4 = x[8];
            AD<double> y5 = x[9];

	        std::cout << "??????" << x1 << ","  << x2 << std::endl;
            fg[0] = 0;
            // f(x) objective function
            fg[0] += CppAD::pow(x1 - 0, 2);
            fg[0] += CppAD::pow(x2 - 1, 2);
            fg[0] += CppAD::pow(x3 - 2, 2);
            fg[0] += CppAD::pow(x4 - 3, 2);
            fg[0] += CppAD::pow(x5 - 4, 2);

            fg[0] += CppAD::pow(y1 - 0, 2);
            fg[0] += CppAD::pow(y2 - 0, 2);
            fg[0] += CppAD::pow(y3 - 1, 2);
            fg[0] += CppAD::pow(y4 - 0, 2);
            fg[0] += CppAD::pow(y5 - 0, 2);
            // constraints
            fg[1] = 0;
            // fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
            return;
	    }
};

}

bool get_started(void)
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t nx = 10; // number of varibles
    size_t ng = 1; // number of constraints
    Dvector x0(nx); // initial condition of varibles
    x0[0] = 0.0;
    x0[1] = 0.0;
    x0[2] = 0.0;
    x0[3] = 0.0;
    x0[4] = 0.0;
    x0[5] = 0.0;
    x0[6] = 0.0;
    x0[7] = 0.0;
    x0[8] = 0.0;
    x0[9] = 0.0;


    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    for(i = 0; i < 5; i++)
    {
        xl[i] = -0.5;
        xu[i] = 5.0;
    }
    for(i = 5; i < 10; i++)
    {
        xl[i] = -3;
        xu[i] = 3;
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
    options += "Integer max_iter     10\n";
    //approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
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

