
public class AlglibDemo
{
    private static void nlcfunc2_fvec(double[] x, double[] fi, object obj)
    {
        //
        // this callback calculates
        //
        //     f0(x0,x1,x2) = x0+x1
        //     f1(x0,x1,x2) = x2-exp(x0)
        //     f2(x0,x1,x2) = x0^2+x1^2-1
        //
        // and Jacobian matrix J = [dfi/dxj]
        //
        fi[0] = x[0] + x[1];
        fi[1] = x[2] - System.Math.Exp(x[0]);
        fi[2] = x[0] * x[0] + x[1] * x[1] - 1.0;
    }
    public static int Main(string[] args)
    {
        try
        {
            //
            // This example demonstrates minimization of
            //
            //     f(x0,x1) = x0+x1
            //
            // subject to nonlinears constraints
            //
            //    x0^2 + x1^2 - 1 <= 0
            //    x2-exp(x0)       = 0
            //
            // using surrogate RBF models.
            //
            // This optimization mode is intended for expensive objectives/constraints
            // that  lack   derivative   information  (e.g.  obtained  from  numerical
            // simulation or by observing some physical  system).  It  provides  rapid
            // convergence to medium-quality approximate solutions (error in objective
            // being as low as 1% or 0.1%).
            //
            double[] x0 = new double[] { 0, 0, 0 };
            double[] s = new double[] { 1, 1, 1 };
            double epsx = 0.000001;
            int maxits = 0;
            alglib.minnlcstate state;
            alglib.minnlcreport rep;
            double[] x1;

            //
            // Create optimizer object and tune its settings:
            // * epsx=0.000001  stopping condition for inner iterations
            // * s=[1,1]        all variables have unit scale
            // * set ORBIT solver
            //
            alglib.minnlccreate(3, x0, out state);
            alglib.minnlcsetcond(state, epsx, maxits);
            alglib.minnlcsetscale(state, s);
            alglib.minnlcsetalgoorbit(state, 0, 0);

            //
            // Set constraints:
            //
            // Since  version  4.01,  ALGLIB  supports  the  most  general  form of
            // nonlinear constraints: two-sided   constraints  NL<=C(x)<=NU,   with
            // elements being possibly infinite (means that this specific bound  is
            // ignored). It includes equality constraints,  upper/lower  inequality
            // constraints, range constraints. In particular, a pair of constraints
            //
            //        x2-exp(x0)       = 0
            //        x0^2 + x1^2 - 1 <= 0
            //
            // can be specified by passing NL=[0,-INF], NU=[0,0] to minnlcsetnlc2().
            //
            // Constraining functions themselves are passed  as  part  of  a  function
            // vector (see below).
            //
            double[] nl = new double[] { 0, -System.Double.PositiveInfinity };
            double[] nu = new double[] { 0, 0 };
            alglib.minnlcsetnlc2(state, nl, nu);

            //
            // Optimize and test results.
            //
            // Optimizer object accepts vector function (no derivative information  is
            // needed), with first component being objective  and  next  2  components
            // being nonlinear constraints.
            //
            // So, our vector function has form
            //
            //     {f0,f1,f2} = { x0+x1 , x2-exp(x0) , x0^2+x1^2-1 }
            //
            // with f0 being objective function, f1 being equality constraint "f1=0",
            // f2 being inequality constraint "f2<=0".
            //
            alglib.minnlcoptimize(state, nlcfunc2_fvec, null, null);
            alglib.minnlcresults(state, out x1, out rep);
            System.Console.WriteLine("{0}", alglib.ap.format(x1, 2)); // EXPECTED: [-0.70710,-0.70710,0.49306]
        }
        catch(alglib.alglibexception alglib_exception)
        {
            System.Console.WriteLine("ALGLIB exception with message '{0}'", alglib_exception.msg);
            return 1;
        }
        return 0;
    }
}
