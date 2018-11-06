using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;

namespace Frame3ddn
{
    public class Solver
    {
        public Output Solve(Input input)
        {
            //Fixed values
            int ok = 1;
            double rmsResid = 1.0;
            double error = 1.0;
            int axialStrainWarning = 0;
            int axialSign = 1;
            int writeMatrix = 0;

            string title = input.Title;
            IReadOnlyList<Node> nodes = input.Nodes;
            IReadOnlyList<FrameElement> frameElements = input.FrameElements;
            int nN = input.Nodes.Count;
            List<float> rj = input.Nodes.Select(n => n.Radius).ToList();
            List<Vec3Float> xyz = input.Nodes.Select(n => n.Position).ToList(); 
            int DoF = 6 * nN;
            int nR = input.ReactionInputs.Count;
            double[] q = new double[DoF];
            float[] r = new float[DoF];
            for (int i = 0; i < nR; i++) 
            {
                int j = input.ReactionInputs[i].Number;  //This index number is decreased by 1 when importing
                r[j * 6 + 0] = input.ReactionInputs[i].Position.X;
                r[j * 6 + 1] = input.ReactionInputs[i].Position.Y;
                r[j * 6 + 2] = input.ReactionInputs[i].Position.Z;
                r[j * 6 + 3] = input.ReactionInputs[i].R.X;
                r[j * 6 + 4] = input.ReactionInputs[i].R.Y;
                r[j * 6 + 5] = input.ReactionInputs[i].R.Z;
            }

            for (int i = 0; i < DoF; i++)
            {
                q[i] = (r[i] == (double) 1) ? 0 : 1;
            }

            int nE = input.FrameElements.Count; 

            List<double> L = input.FrameElements.Select(f =>
                    Coordtrans.CalculateSQDistance(input.Nodes[f.NodeIdx1].Position, input.Nodes[f.NodeIdx2].Position))
                .ToList();

            List<double> Le = new List<double>(); 
            for (int i = 0; i < L.Count; i++) 
            {
                Le.Add(L[i] - input.Nodes[input.FrameElements[i].NodeIdx1].Radius -
                       input.Nodes[input.FrameElements[i].NodeIdx2].Radius);
            }

            List<int> N1 = input.FrameElements.Select(f => f.NodeIdx1).ToList(); 
            List<int> N2 = input.FrameElements.Select(f => f.NodeIdx2).ToList(); 
            List<float> Ax = input.FrameElements.Select(f => f.Ax).ToList(); 
            List<float> Asy = input.FrameElements.Select(f => f.Asy).ToList();
            List<float> Asz = input.FrameElements.Select(f => f.Asz).ToList();
            List<float> Jx = input.FrameElements.Select(f => f.Jx).ToList();
            List<float> Iy = input.FrameElements.Select(f => f.Iy).ToList();
            List<float> Iz = input.FrameElements.Select(f => f.Iz).ToList();
            List<float> E = input.FrameElements.Select(f => f.E).ToList();
            List<float> G = input.FrameElements.Select(f => f.G).ToList();
            List<float> p = input.FrameElements.Select(f => f.Roll).ToList(); 
            List<float> d = input.FrameElements.Select(f => f.Density).ToList();
            int nL = input.LoadCases.Count; 
            List<float> gX = input.LoadCases.Select(l => l.Gravity.X).ToList(); 
            List<float> gY = input.LoadCases.Select(l => l.Gravity.Y).ToList(); 
            List<float> gZ = input.LoadCases.Select(l => l.Gravity.Z).ToList(); 
            List<int> nF = input.LoadCases.Select(l => l.NodeLoads.Count).ToList();
            List<int> nU = input.LoadCases.Select(l => l.UniformLoads.Count).ToList();
            List<int> nW = input.LoadCases.Select(l => l.TrapLoads.Count).ToList();
            float[,,] U = new float[nL, nE, 4];
            for (int i = 0; i < nL; i ++)
            {
                for (int j = 0; j < nE; j++)
                {
                    //U[i, j, 0] will store node index, but not all of them will be assigned a value.
                    //For those not assigned, they are the same value for node index 0.
                    //This will cause some trouble later, so have to be initialized with another value.
                    //The problem is caused by converting from 1 based array in the C code to 0 based array in C# here.
                    U[i, j, 0] = -1;
                }
            }
            float[,,] W = new float[nL, nE*10, 13];
            for (int i = 0; i < nL; i++)
            {
                for (int j = 0; j < nE * 10; j++)
                {
                    W[i, j, 0] = -1;//Same reason for array U
                }
            }
            double[,,] eqFMech = new double[nL, nE, 12];
            double[,] FMech = new double[nL, DoF];
            bool shear = input.IncludeShearDeformation;
            bool geom = input.IncludeGeometricStiffness;
            float dx = input.XAxisIncrementForInternalForces;

            double[] F = new double[DoF];
            double[] dF = new double[DoF];
            double[] D = new double[DoF];
            double[] dD = new double[DoF];
            double[] R = new double[DoF];
            double[] dR = new double[DoF];
            double[,] Q = new double[nE, 12];
            double[,] K = new double[DoF, DoF];

            ////These are not used
            int[] nT = new int[nL];
            int[] nP = new int[nL];
            int[] nD = new int[nL];
            double[,] FTemp = new double[nL,DoF];
            double[,,] eqFTemp = new double[nL,nE,12];
            int iter = 0;
            double tol = 1.0e-9;
            float[,,] P = new float[nL,10*nE,5];
            double[,] Dp = new double[nL, DoF];
            ////

            IReadOnlyList<LoadCase> loadCases = input.LoadCases;

            Frame3ddIO.AssembleLoads(nN, nE, nL, DoF, xyz, L, Le, N1, N2, Ax, Asy, Asz, Iy, Iz, E, G, p, d, gX, gY, gZ, shear, nF, nU, nW, FMech, eqFMech,
                U, W, loadCases);

            //ReadMassData()--Not implemented
            //ReadCondensationData()--Not implemented

            ////Start anlyz
            /* begin load case analysis loop */
            List<LoadCaseOutput> loadCaseOutputs = new List<LoadCaseOutput>();
            for (int lc = 0; lc < nL; lc++)
            {
                /*  initialize displacements and displ. increment to {0}  */
                /*  initialize reactions     and react. increment to {0}  */
                for (int i = 0; i < DoF; i++)
                    D[i] = dD[i] = R[i] = dR[i] = 0.0;

                /*  initialize internal element end forces Q = {0}	*/
                for (int i = 0; i < nE; i++)
                    for (int j = 0; j < 11; j++)
                        Q[i, j] = 0.0;

                K = Frame3dd.AssembleK(DoF, nE, xyz, r, L, Le, N1, N2, Ax, Asy, Asz, Jx, Iy, Iz, E, G, p, shear, geom, Q);

                //if (nT[lc] > 0){
                //Aplly temperature loads only --Not implemented
                //}

                if (nF[lc] > 0 || nU[lc] > 0 || nW[lc] > 0 || nP[lc] > 0 || nD[lc] > 0 ||
                    gX[lc] != 0 || gY[lc] != 0 || gZ[lc] != 0)
                {
                    for (int i = 0; i < DoF; i++)
                        if (!Common.isDoubleZero(r[i]))
                            dD[i] = Dp[lc, i];
                }

                double[] tempLoadMech = Common.GetRow(FMech, lc);
                var solveSystemResult = Frame3dd.SolveSystem(K, dD, tempLoadMech, dR, DoF, q, r, ok, rmsResid);
                ok = solveSystemResult.ok;
                rmsResid = solveSystemResult.rmsResid;
                for (int i = 0; i < DoF; i++)
                {
                    FMech[lc, i] = tempLoadMech[i];
                }

                for (int i = 0; i < DoF; i++)
                {
                    if (!Common.isDoubleZero(q[i]))
                    {
                        D[i] += dD[i];
                    }
                    else
                    {
                        D[i] = Dp[lc, i]; dD[i] = 0.0;
                    }
                }

                for (int i = 0; i < DoF; i++)
                    if (!Common.isDoubleZero(r[i]))
                        R[i] += dR[i];

                /*  combine {F} = {F_t} + {F_m} */
                for (int i = 0; i < DoF; i++)
                    F[i] = FTemp[lc, i] + FMech[lc, i];

                double[,] tempTArray = Common.GetArray(eqFTemp, lc);
                double[,] tempMArray = Common.GetArray(eqFMech, lc);

                Frame3dd.ElementEndForces(Q, nE, xyz, L, Le, N1, N2,
                    Ax, Asy, Asz, Jx, Iy, Iz, E, G, p,
                    tempTArray, tempMArray, D, shear, geom,
                    axialStrainWarning);

                error = Frame3dd.EquilibriumError(dF, F, K, D, DoF, q, r);

                //if (geom && verbose)
                //    fprintf(stdout, "\n Non-Linear Elastic Analysis ...\n");

                if (geom)
                {
                    //Not implemented
                }

                while (geom && error > tol && iter < 500 && ok >= 0)
                {
                    //Not implemented
                }

                //Not implemented
                /*   strain limit failure ... */
                //if (axial_strain_warning > 0 && ExitCode == 0) ExitCode = 182;
                /*   strain limit _and_ buckling failure ... */
                //if (axial_strain_warning > 0 && ExitCode == 181) ExitCode = 183;
                //if (geom) compute_reaction_forces(R, F, K, D, DoF, r);

                if (writeMatrix != 0)/* write static stiffness matrix */
                {
                    //save_ut_dmatrix("Ks", K, DoF, "w");//Not implemented
                }
                /*  display RMS equilibrium error */
                //if (verbose && ok >= 0)
                //    evaluate(error, rms_resid, tol, geom);

                var staticResults = Frame3ddIO.GetStaticResults(nN, nE, nL, lc, DoF, N1, N2, F, D, R, r, Q, error, ok, axialSign);

                ////Not implemented
                //if (filetype == 1)
                //{       // .CSV format output
                //    write_static_csv(OUT_file, title,
                //        nN, nE, nL, lc, DoF, N1, N2, F, D, R, r, Q, error, ok);
                //}

                //if (filetype == 2)
                //{       // .m matlab format output
                //    write_static_mfile(OUT_file, title, nN, nE, nL, lc, DoF,
                //        N1, N2, F, D, R, r, Q, error, ok);
                //}

                float[,] tempU2DArray = Common.GetArray(U, lc);
                float[,] tempW2DArray = Common.GetArray(W, lc);
                float[,] tempP2DArray = Common.GetArray(P, lc);

                List<PeakFrameElementInternalForce> internalForce = Frame3ddIO.GetInternalForces(lc, nL, title, dx, xyz,
                    Q, nN, nE, L, N1, N2,
                    Ax, Asy, Asz, Jx, Iy, Iz, E, G, p,
                    d, gX[lc], gY[lc], gZ[lc],
                    nU[lc], tempU2DArray, nW[lc], tempW2DArray, nP[lc], tempP2DArray,
                    D, shear, error);

                loadCaseOutputs.Add(new LoadCaseOutput(error, staticResults.nodeDisplacements, staticResults.frameElementEndForces,
                        staticResults.reactionOutputs, internalForce));

                //static_mesh ()
                //...
            }

            Output output = new Output(loadCaseOutputs);
            return output;
        }

        // ref main.c:265
        static int DoF(Input input) => input.Nodes.Count * 6;

        /// <summary>
        /// global stiffness matrix
        /// </summary>
        // ref main.c:343
        static double[,] K(Input input)
        {
            var dof = DoF(input);
            var k = new double[dof, dof];
            return k;
        }

    }
}
