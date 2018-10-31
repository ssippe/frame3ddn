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
            //Fixed value
            int ok = 1;
            double rmsResid = 1.0;
            int axialStrainWarning = 0;

            IReadOnlyList<Node> nodes = input.Nodes;
            IReadOnlyList<FrameElement> frameElements = input.FrameElements;
            int nN = input.Nodes.Count;
            List<float> rj = input.Nodes.Select(n => n.Radius).ToList();
            List<Vec3Float> xyz = input.Nodes.Select(n => n.Position).ToList(); //V
            int DoF = 6 * nN;
            int nR = input.ReactionInputs.Count;
            double[] q = new double[DoF];
            float[] r = new float[DoF];
            for (int i = 0; i < nR; i++) //><
            {
                int j = input.ReactionInputs[i].Number; //>< This number is corrected wehen importing
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

            int nE = input.FrameElements.Count; //V

            List<double> L = input.FrameElements.Select(f =>
                    Coordtrans.CalculateSQDistance(input.Nodes[f.NodeIdx1].Position, input.Nodes[f.NodeIdx2].Position))
                .ToList();

            List<double> Le = new List<double>(); //V
            for (int i = 0; i < L.Count; i++) //><
            {
                Le.Add(L[i] - input.Nodes[input.FrameElements[i].NodeIdx1].Radius -
                       input.Nodes[input.FrameElements[i].NodeIdx2].Radius);
            }

            List<int> N1 = input.FrameElements.Select(f => f.NodeIdx1).ToList(); //V
            List<int> N2 = input.FrameElements.Select(f => f.NodeIdx2).ToList(); //V
            List<float> Ax = input.FrameElements.Select(f => f.Ax).ToList(); //V
            List<float> Asy = input.FrameElements.Select(f => f.Asy).ToList();
            List<float> Asz = input.FrameElements.Select(f => f.Asz).ToList();
            List<float> Jx = input.FrameElements.Select(f => f.Jx).ToList();
            List<float> Iy = input.FrameElements.Select(f => f.Iy).ToList();
            List<float> Iz = input.FrameElements.Select(f => f.Iz).ToList();
            List<float> E = input.FrameElements.Select(f => f.E).ToList();
            List<float> G = input.FrameElements.Select(f => f.G).ToList();
            List<float> p = input.FrameElements.Select(f => f.Roll).ToList(); //V
            List<float> d = input.FrameElements.Select(f => f.Density).ToList();
            int nL = input.LoadCases.Count; //V
            List<float> gX = input.LoadCases.Select(l => l.Gravity.X).ToList(); //V
            List<float> gY = input.LoadCases.Select(l => l.Gravity.Y).ToList(); //V
            List<float> gZ = input.LoadCases.Select(l => l.Gravity.Z).ToList(); //V
            List<int> nF = input.LoadCases.Select(l => l.NodeLoads.Count).ToList();
            List<int> nU = input.LoadCases.Select(l => l.UniformLoads.Count).ToList();
            List<int> nW = input.LoadCases.Select(l => l.TrapLoads.Count).ToList();
            float[,,] U = new float[nL, nE, 4];//pass in
            float[,,] W = new float[nL, nE*10, 13];//pass in
            double[,,] eqFMech = new double[nL, nE, 12];
            double[,] FMech = new double[nL, DoF];
            bool shear = input.IncludeShearDeformation;
            bool geom = input.IncludeGeometricStiffness;

            ////These countings are not used so far
            int[] nT = new int[nL];
            int[] nP = new int[nL];
            int[] nD = new int[nL];
            double[,] FTemp = new double[nL,DoF];
            double[,,] eqFTemp = new double[nL,nE,12];
            ////

            

            IReadOnlyList<LoadCase> loadCases = input.LoadCases;

            AssembleLoads(nN, nE, nL, DoF, xyz, L, Le, N1, N2, Ax, Asy, Asz, Iy, Iz, E, G, p, d, gX, gY, gZ, shear, nF, nU, nW, FMech, eqFMech,
                U, W, loadCases);
#pragma warning disable CS0219 // Variable is assigned but its value is never used
#pragma warning disable CS0168 // Variable is assigned but its value is never used

            //to be passed back//
            double totalMass;
            double structMass;
            double[] D = new double[DoF];
            double[] dD = new double[DoF];
            double[] R = new double[DoF];
            double[] dR = new double[DoF];
            double[,] Q = new double[nE,12];
            double[,] K = new double[DoF,DoF];
            //to be passed back//

            //Empty values. Only used to prevent error//
            double[,] Dp = new double[nL,DoF];
            //Empty values. Only used to prevent error//


            //ReadMassData()--NoN
            //ReadCondensationData()--NoN

            ////Start anlyz
            /* begin load case analysis loop */
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

                K = AssembleK(DoF, nE, xyz, r, L, Le, N1, N2, Ax, Asy, Asz, Jx, Iy, Iz, E, G, p, shear, geom, Q);//V

                //if (nT[lc] > 0){
                //Aplly temperature loads only --NoN
                //}

                if (nF[lc] > 0 || nU[lc] > 0 || nW[lc] > 0 || nP[lc] > 0 || nD[lc] > 0 ||
                    gX[lc] != 0 || gY[lc] != 0 || gZ[lc] != 0)
                {
                    for (int i = 0; i < DoF; i++)
                        if (!Common.isDoubleZero(r[i]))
                            dD[i] = Dp[lc, i];
                }

                double[] tempLoadMech = Common.GetRow(FMech, lc);
                SolveSystem(K, dD, tempLoadMech, dR, DoF, q, r, ok, rmsResid);//a
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

                ////--NoN
                /*  combine {F} = {F_t} + {F_m} */
                //// 


                double[,] tempTArray = Common.GetArray(eqFTemp, lc);
                double[,] tempMArray = Common.GetArray(eqFMech, lc);
                ElementEndForces(Q, nE, xyz, L, Le, N1, N2,
                    Ax, Asy, Asz, Jx, Iy, Iz, E, G, p,
                    tempTArray, tempMArray, D, shear, geom,
                    axialStrainWarning);
            }

            

            return null;
        }

        private void ElementEndForces(double[,] Q, int nE, List<Vec3Float> xyz, List<double> L, List<double> Le, List<int> N1, List<int> N2,
            List<float> Ax, List<float> Asy, List<float> Asz, List<float> Jx, List<float> Iy, List<float> Iz, List<float> E, List<float> G, List<float> p,
            double[,] eqFTempArray, double[,] eqFMechArray, double[] D, bool shear, bool geom,
            int axialStrainWarning)
        {
            double axialStrain = 0;
            int m, j;

            double[] s = new double[12];

            axialStrainWarning = 0;//to return
            for (m = 0; m < nE; m++)
            {
                double[] tempFRow = Common.GetRow(eqFMechArray, m);
                double[] tempTRow = Common.GetRow(eqFTempArray, m);
                FrameElementForce(s, xyz, L[m], Le[m], N1[m], N2[m],
                    Ax[m], Asy[m], Asz[m], Jx[m], Iy[m], Iz[m], E[m], G[m], p[m],
                    tempTRow, tempFRow, D, shear, geom, axialStrain);

                for (j = 0; j < 12; j++) Q[m, j] = s[j];

                if (Math.Abs(axialStrain) > 0.001)
                {
                    Console.WriteLine(" Warning! Frame element %2d has an average axial strain of %8.6f\n", m, axialStrain);
                    ++axialStrainWarning;
                }

            }
        }

        private void FrameElementForce(double[] s, List<Vec3Float> xyz, double L, double Le, int n1, int n2,
            float Ax, float Asy, float Asz, float J, float Iy, float Iz, float E, float G, float p, double[] fT, double[] fM,
            double[] D, bool shear, bool geom, double axialStrain)//return s
        {
            double t1, t2, t3, t4, t5, t6, t7, t8, t9, /* coord Xformn	*/
                d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12,
                // x1, y1, z1, x2, y2, z2,	/* node coordinates	*/
                //  Ls,			/* stretched length of element */
                delta = 0.0,        /* stretch in the frame element */
                Ksy, Ksz, Dsy, Dsz, /* shear deformation coeff's	*/
                T = 0.0;        /* axial force for geometric stiffness */

            double f1 = 0, f2 = 0, f3 = 0, f4 = 0, f5 = 0, f6 = 0,
                f7 = 0, f8 = 0, f9 = 0, f10 = 0, f11 = 0, f12 = 0;

            double[] t = Coordtrans.coordTrans(xyz, L, n1, n2, p);

            n1 = 6 * n1; n2 = 6 * n2;

            d1 = D[n1 + 0]; d2 = D[n1 + 1]; d3 = D[n1 + 2];
            d4 = D[n1 + 3]; d5 = D[n1 + 4]; d6 = D[n1 + 5];
            d7 = D[n2 + 0]; d8 = D[n2 + 1]; d9 = D[n2 + 2];
            d10 = D[n2 + 3]; d11 = D[n2 + 4]; d12 = D[n2 + 5];

            if (shear)
            {
                Ksy = 12.0 * E * Iz / (G * Asy * Le * Le);
                Ksz = 12.0 * E * Iy / (G * Asz * Le * Le);
                Dsy = (1 + Ksy) * (1 + Ksy);
                Dsz = (1 + Ksz) * (1 + Ksz);
            }
            else
            {
                Ksy = Ksz = 0.0;
                Dsy = Dsz = 1.0;
            }
            delta = (d7 - d1) * t[0] + (d8 - d2) * t[1] + (d9 - d3) * t[2];
            axialStrain = delta / Le; // log(Ls/Le);

            s[0] = -(Ax * E / Le) * ((d7 - d1) * t[0] + (d8 - d2) * t[1] + (d9 - d3) * t[2]);

            if (geom) T = -s[0];

            s[1] = -(12.0* E * Iz / (Le * Le * Le * (1.0+ Ksy)) +
                   T / L * (1.2 + 2.0 * Ksy + Ksy * Ksy) / Dsy) *
                        ((d7 - d1) * t[3] + (d8 - d2) * t[4] + (d9 - d3) * t[5])
                + (6.0* E * Iz / (Le * Le * (1.0+ Ksy)) + T / 10.0 / Dsy) *
                        ((d4 + d10) * t[6] + (d5 + d11) * t[7] + (d6 + d12) * t[8]);
            s[2] = -(12.0* E * Iy / (Le * Le * Le * (1.0+ Ksz)) +
                  T / L * (1.2 + 2.0 * Ksz + Ksz * Ksz) / Dsz) *
                        ((d7 - d1) * t[6] + (d8 - d2) * t[7] + (d9 - d3) * t[8])
                - (6.0* E * Iy / (Le * Le * (1.0+ Ksz)) + T / 10.0 / Dsz) *
                        ((d4 + d10) * t[3] + (d5 + d11) * t[4] + (d6 + d12) * t[5]);
            s[3] = -(G * J / Le) * ((d10 - d4) * t[0] + (d11 - d5) * t[1] + (d12 - d6) * t[2]);
            s[4] = (6.0* E * Iy / (Le * Le * (1.0+ Ksz)) + T / 10.0 / Dsz) *
                        ((d7 - d1) * t[6] + (d8 - d2) * t[7] + (d9 - d3) * t[8])
                + ((4.0+ Ksz) * E * Iy / (Le * (1.0+ Ksz)) +
                    T * L * (2.0 / 15.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d4 * t[3] + d5 * t[4] + d6 * t[5])
                + ((2.0- Ksz) * E * Iy / (Le * (1.0+ Ksz)) -
                    T * L * (1.0 / 30.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d10 * t[3] + d11 * t[4] + d12 * t[5]);
            s[5] = -(6.0* E * Iz / (Le * Le * (1.0+ Ksy)) + T / 10.0 / Dsy) *
                        ((d7 - d1) * t[3] + (d8 - d2) * t[4] + (d9 - d3) * t[5])
                + ((4.0+ Ksy) * E * Iz / (Le * (1.0+ Ksy)) +
                    T * L * (2.0 / 15.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d4 * t[6] + d5 * t[7] + d6 * t[8])
                + ((2.0- Ksy) * E * Iz / (Le * (1.0+ Ksy)) -
                    T * L * (1.0 / 30.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d10 * t[6] + d11 * t[7] + d12 * t[8]);
            s[6] = -s[0];
            s[7] = -s[1];
            s[8] = -s[2];
            s[9] = -s[3];

            s[10] = (6.0* E * Iy / (Le * Le * (1.0+ Ksz)) + T / 10.0 / Dsz) *
                        ((d7 - d1) * t[6] + (d8 - d2) * t[7] + (d9 - d3) * t[8])
                + ((4.0+ Ksz) * E * Iy / (Le * (1.0+ Ksz)) +
                    T * L * (2.0 / 15.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d10 * t[3] + d11 * t[4] + d12 * t[5])
                + ((2.0- Ksz) * E * Iy / (Le * (1.0+ Ksz)) -
                    T * L * (1.0 / 30.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d4 * t[3] + d5 * t[4] + d6 * t[5]);
            s[11] = -(6.0* E * Iz / (Le * Le * (1.0+ Ksy)) + T / 10.0 / Dsy) *
                        ((d7 - d1) * t[3] + (d8 - d2) * t[4] + (d9 - d3) * t[5])
                + ((4.0+ Ksy) * E * Iz / (Le * (1.0+ Ksy)) +
                    T * L * (2.0 / 15.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d10 * t[6] + d11 * t[7] + d12 * t[8])
                + ((2.0- Ksy) * E * Iz / (Le * (1.0+ Ksy)) -
                    T * L * (1.0 / 30.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d4 * t[6] + d5 * t[7] + d6 * t[8]);

            // add fixed end forces to internal element forces
            // 18oct[1]012, 14may1204, 15may2014

            // add temperature fixed-end-forces to variables f1-f12
            // add mechanical load fixed-end-forces to variables f1-f12
            // f1 ...  f12 are in the global element coordinate system
            f1 = fT[0] + fM[0]; f2 = fT[1] + fM[1]; f3 = fT[2] + fM[2];
            f4 = fT[3] + fM[3]; f5 = fT[4] + fM[4]; f6 = fT[5] + fM[5];
            f7 = fT[6] + fM[6]; f8 = fT[7] + fM[7]; f9 = fT[8] + fM[8];
            f10 = fT[9] + fM[9]; f11 = fT[10] + fM[10]; f12 = fT[11] + fM[11];

            // transform f1 ... f12 to local element coordinate system and
            // add local fixed end forces (-equivalent loads) to internal loads 
            // {Q} = [T]{f}

            s[0] -= (f1 * t[0] + f2 * t[1] + f3 * t[2]);
            s[1] -= (f1 * t[3] + f2 * t[4] + f3 * t[5]);
            s[2] -= (f1 * t[6] + f2 * t[7] + f3 * t[8]);
            s[3] -= (f4 * t[0] + f5 * t[1] + f6 * t[2]);
            s[4] -= (f4 * t[3] + f5 * t[4] + f6 * t[5]);
            s[5] -= (f4 * t[6] + f5 * t[7] + f6 * t[8]);

            s[6] -= (f7 * t[0] + f8 * t[1] + f9 * t[2]);
            s[7] -= (f7 * t[3] + f8 * t[4] + f9 * t[5]);
            s[8] -= (f7 * t[6] + f8 * t[7] + f9 * t[8]);
            s[9] -= (f10 * t[0] + f11 * t[1] + f12 * t[2]);
            s[10] -= (f10 * t[3] + f11 * t[4] + f12 * t[5]);
            s[11] -= (f10 * t[6] + f11 * t[7] + f12 * t[8]);

        }


        /*
         * SOLVE_SYSTEM  -  solve {F} =   [K]{D} via L D L' decomposition        27dec01
         * Prescribed displacements are "mechanical loads" not "temperature loads"  
         */
        private (int ok, double rmsResid) SolveSystem(double[,] K, double[] D, double[] F, double[] R, int DoF, double[] q, float[] r, int ok, double rmsResid)
        {
            double[] diag = new double[DoF];

            ok = LdlDcmpPm(K, DoF, diag, F, D, R, q, r, 1, 0);//V
            //K = result.A;
            //diag = result.d;
            //D = result.x;
            //R = result.c;
            //ok = result.pd;

            if (ok < 0)
            {
                Console.WriteLine(" Make sure that all six");
                Console.WriteLine(" rigid body translations are restrained!\n");
            }
            else
            {
                ok = LdlDcmpPm(K, DoF, diag, F, D, R, q, r, 0, 1);
                rmsResid = ok = 1;
                do
                {   /* improve solution for D[q] and R[r] */
                    var result = LdlMprovePm(K, DoF, diag, F, D, R, q, r, rmsResid);
                    rmsResid = result.rmsResid;
                    ok = result.ok;
                } while (ok != 0);
            }

            return (ok, rmsResid);
        }

        private (int ok, double rmsResid) LdlMprovePm(double[,] A, int n, double[] d, double[] b, double[] x, double[] c, double[] q,
            float[] r, double rmsResid)
        {
            double sdp;     // accumulate the r.h.s. in double precision
            double rms_resid_new = 0.0; // the RMS error of the mprvd solution
            int j, i, pd;

            double[] dx = new double[n];// the residual error
            double[] dc = new double[n];// update to partial r.h.s. vector, c

            // calculate the r.h.s. of ...
            //  [A_qq]{dx_q} = {b_q} - [A_qq]*{x_q} - [A_qr]*{x_r}      
            //  {dx_r} is left unchanged at 0.0;
            for (i = 0; i < n; i++)
            {
                if (!Common.isDoubleZero(q[i]))
                {
                    sdp = b[i];
                    for (j = 0; j < n; j++)
                    {
                        if (!Common.isDoubleZero(q[j]))
                        {   // A_qq in upper triangle only
                            if (i <= j) {sdp -= A[i, j] * x[j];
                                int xxxxx = 1;
                            }
                            else sdp -= A[j, i] * x[j];
                        }
                    }
                    for (j = 0; j < n; j++)
                        if (!Common.isDoubleZero(r[j]))
                            sdp -= A[i, j] * x[j];
                    dx[i] = sdp;
                } // else dx[i] = 0.0; // x[i];
            }

            // solve for the residual error term, A is already factored
            LdlDcmpPm(A, n, d, dx, dx, dc, q, r, 0, 1);

            for (i = 0; i < n; i++)
                if (!Common.isDoubleZero(q[i]))
                    rms_resid_new += dx[i] * dx[i];

            rms_resid_new = Math.Sqrt(rms_resid_new / (double)n);

            int ok = 0;
            if (rms_resid_new / rmsResid < 0.90)
            { /*  enough improvement    */
                for (i = 0; i < n; i++)
                {   /*  update the solution 2014-05-14   */
                    if (!Common.isDoubleZero(q[i]))
                        x[i] += dx[i];
                    if (!Common.isDoubleZero(r[i]))
                        c[i] += dc[i];
                }
                rmsResid = rms_resid_new; /* return the new residual   */
                ok = 1;            /* the solution has improved */
            }

            return (ok, rmsResid);
        }

        private int LdlDcmpPm(double[,] A, int n, double[] d, double[] b, double[] x, double[] c, double[] q, float[] r, int reduce, int solve)
        {
            int i, j, k, m;
            int pd = 0;
            if (reduce != 0)
            {
                for (j = 0; j < n; j++)
                {
                    d[j] = 0.0;

                    if (!Common.isDoubleZero(q[j]))
                    { /* reduce column j, except where q[i]==0	*/

                        for (m = 0, i = 0; i < j; i++)  /* scan the sky-line	*/
                            if (Common.isDoubleZero(A[i, j]))
                                ++m;
                            else
                                break;

                        for (i = m; i < j; i++)
                        {
                            if (!Common.isDoubleZero(q[i]))
                            {
                                A[j, i] = A[i, j];
                                for (k = m; k < i; k++)
                                    if (!Common.isDoubleZero(q[k]))
                                        A[j, i] -= A[j, k] * A[i, k];
                            }
                        }

                        d[j] = A[j, j];
                        for (i = m; i < j; i++)
                            if (!Common.isDoubleZero(q[i]))
                                d[j] -= A[j, i] * A[j, i] / d[i];
                        for (i = m; i < j; i++)
                            if (!Common.isDoubleZero(q[i]))
                                A[j, i] /= d[i];

                        if (Common.isDoubleZero(d[j]))
                        {
                            Console.WriteLine(" ldl_dcmp_pm(): zero found on diagonal ...\n");
                            Console.WriteLine(" d[%d] = %11.4e\n", j, d[j]);
                            return pd;
                        }
                        if (d[j] < 0.0)
                            (pd)--;
                    }
                }
            }

            if (solve != 0)		/* back substitution to solve for {x}   */
            {
                for (i = 0; i < n; i++)
                {
                    if (!Common.isDoubleZero(q[i]))
                    {
                        x[i] = b[i];
                        for (j = 0; j < n; j++)
                            if (!Common.isDoubleZero(r[j]))
                                x[i] -= A[i, j] * x[j];
                    }
                }

                /* {x} is run through the same forward reduction as was [A] */
                for (i = 0; i < n; i++)
                    if (!Common.isDoubleZero(q[i]))
                        for (j = 0; j < i; j++)
                            if (!Common.isDoubleZero(q[j]))
                                x[i] -= A[i, j] * x[j];

                for (i = 0; i < n; i++)
                    if (!Common.isDoubleZero(q[i]))
                        x[i] /= d[i];

                /* now back substitution is conducted on {x};  [A] is preserved */

                for (i = n - 1; i > 0; i--)
                    if (!Common.isDoubleZero(q[i]))
                        for (j = 0; j < i; j++)
                            if (!Common.isDoubleZero(q[j]))
                                x[j] -= A[i, j] * x[i];

                /* finally, evaluate c_r	*/
                for (i = 0; i < n; i++)
                {
                    c[i] = 0.0;
                    if (!Common.isDoubleZero(r[i]))
                    {
                        c[i] = -b[i]; // changed from 0.0 to -b[i]; 2014-05-14
                        for (j = 0; j < n; j++)
                            c[i] += A[i, j] * x[j];
                    }
                }
            }

            return pd;
        }

        private double[,] AssembleK(
            int DoF, int nE,
            List<Vec3Float> xyz, float[] r, List<double> L, List<double> Le,
            List<int> N1, List<int> N2,
            List<float> Ax, List<float> Asy, List<float> Asz,
            List<float> Jx, List<float> Iy, List<float> Iz,
            List<float> E, List<float> G, List<float> p,
            bool shear, bool geom, double[,] Q)
        {
            //to be passed back//
            double[,] K = new double[DoF, DoF];
            //to be passed back//
            int ii, jj, ll;

            for (int i = 0; i < DoF; i++)
                for (int j = 0; j < DoF; j++)
                    K[i, j] = 0.0;

            double[,] k = new double[12, 12];
            int[,] ind = new int[12, nE];

            for (int i = 0; i < nE; i++)
            {
                ind[0, i] = 6 * N1[i]; ind[6, i] = 6 * N2[i];
                ind[1, i] = ind[0, i] + 1; ind[7, i] = ind[6, i] + 1;
                ind[2, i] = ind[0, i] + 2; ind[8, i] = ind[6, i] + 2;
                ind[3, i] = ind[0, i] + 3; ind[9, i] = ind[6, i] + 3;
                ind[4, i] = ind[0, i] + 4; ind[10, i] = ind[6, i] + 4;
                ind[5, i] = ind[0, i] + 5; ind[11, i] = ind[6, i] + 5;
            }

            for (int i = 0; i < nE; i++)
            {
                ElasticK(k, xyz, r, L[i], Le[i], N1[i], N2[i],
                    Ax[i], Asy[i], Asz[i], Jx[i], Iy[i], Iz[i], E[i], G[i], p[i], shear);//V 

                if (geom)
                {
                    throw new Exception("geom N/A");
                }

                for (int l = 0; l < 12; l++)
                {
                    ii = ind[l, i];
                    for (ll = 0; ll < 12; ll++)
                    {
                        jj = ind[ll, i];
                        K[ii, jj] += k[l, ll];
                    }
                }
            }

            return K;
        }

        private void ElasticK(double[,] k, List<Vec3Float> xyz, float[] r,
            double L, double Le,
            int n1, int n2,
            double Ax, double Asy, double Asz,
            double J, double Iy, double Iz,
            double E, double G, float p,
            bool shear)
        {
            double t1, t2, t3, t4, t5, t6, t7, t8, t9,     /* coord Xformn */
                Ksy, Ksz;       /* shear deformatn coefficients	*/
            int i, j;

            double[] t = Coordtrans.coordTrans(xyz, L, n1, n2, p);

            for (i = 0; i < 12; i++)
                for (j = 0; j < 12; j++)
                    k[i, j] = 0.0;

            if (shear)
            {
                Ksy = 12.0 * E * Iz / (G * Asy * Le * Le);
                Ksz = 12.0 * E * Iy / (G * Asz * Le * Le);
            }
            else
            {
                Ksy = Ksz = 0.0;
            }

            k[0, 0] = k[6, 6] = E * Ax / Le;
            k[1, 1] = k[7, 7] = 12.0* E * Iz / (Le * Le * Le * (1.0+ Ksy));
            k[2, 2] = k[8, 8] = 12.0* E * Iy / (Le * Le * Le * (1.0+ Ksz));
            k[3, 3] = k[9, 9] = G * J / Le;
            k[4, 4] = k[10, 10] = (4.0+ Ksz) * E * Iy / (Le * (1.0+ Ksz));
            k[5, 5] = k[11, 11] = (4.0+ Ksy) * E * Iz / (Le * (1.0+ Ksy));

            k[4, 2] = k[2, 4] = -6.0* E * Iy / (Le * Le * (1.0+ Ksz));
            k[5, 1] = k[1, 5] = 6.0* E * Iz / (Le * Le * (1.0+ Ksy));
            k[6, 0] = k[0, 6] = -k[0, 0];

            k[11, 7] = k[7, 11] = k[7, 5] = k[5, 7] = -k[5, 1];
            k[10, 8] = k[8, 10] = k[8, 4] = k[4, 8] = -k[4, 2];
            k[9, 3] = k[3, 9] = -k[3, 3];
            k[10, 2] = k[2, 10] = k[4, 2];
            k[11, 1] = k[1, 11] = k[5, 1];

            k[7, 1] = k[1, 7] = -k[1, 1];
            k[8, 2] = k[2, 8] = -k[2, 2];
            k[10, 4] = k[4, 10] = (2.0- Ksz) * E * Iy / (Le * (1.0+ Ksz));
            k[11, 5] = k[5, 11] = (2.0- Ksy) * E * Iz / (Le * (1.0+ Ksy));//V

            k = Coordtrans.Atma(t, k, r[n1], r[n2]);

            for (i = 0; i < 12; i++)
            {
                for (j = i + 1; j < 12; j++)
                {
                    if (k[i, j] != k[j, i])
                    {
                        if (Math.Abs(k[i, j] / k[j, i] - 1.0) > 1.0e-6
                            && (Math.Abs(k[i, j] / k[i, i]) > 1e-6
                                || Math.Abs(k[j, i] / k[i, i]) > 1e-6
                            )
                        )
                        {
                            Console.WriteLine("elastic_K: element stiffness matrix not symetric ...\n");
                            Console.WriteLine(" ... k[%d][%d] = %15.6e \n", i, j, k[i, j]);
                            Console.WriteLine(" ... k[%d][%d] = %15.6e   ", j, i, k[j, i]);
                            Console.WriteLine(" ... relative error = %e \n", Math.Abs(k[i, j] / k[j, i] - 1.0));
                            Console.WriteLine(" ... element matrix saved in file 'kt'\n");
                            SaveDmatrix("kt", k, 1, 12, 1, 12, 0, "w");
                        }

                        k[i, j] = k[j, i] = 0.5 * (k[i, j] + k[j, i]);
                    }
                }
            }
        }

        private void SaveDmatrix(string a, double[,] k, int b, int c, int d, int e, int f, string g)
        {
            Console.WriteLine("SaveDmatrix N/A");
            throw new Exception("SaveDmatrix N/A");
        }
        

        private void AssembleLoads(int nN, int nE, int nL, int DoF, List<Vec3Float> xyz, List<double> L, List<double> Le,
            List<int> N1, List<int> N2,
            List<float> Ax, List<float> Asy, List<float> Asz, List<float> Iy, List<float> Iz, List<float> E,
            List<float> G, List<float> p,
            List<float> d, List<float> gX, List<float> gY, List<float> gZ, bool shear, List<int> nF, List<int> nU, List<int> nW,
            double[,] FMech, double[,,] eqFMech,
            float[,,] U, float[,,] W, IReadOnlyList<LoadCase> loadCases)
        {
#pragma warning disable CS0219 // Variable is assigned but its value is never used
#pragma warning disable CS0168 // Variable is assigned but its value is never used
            double Ksy, Ksz; 		/* shear deformatn coefficients	*/
            double x1, x2, w1, w2;
            double Nx1,
                Vy1,
                Vz1,
                Mx1 = 0.0,
                My1 = 0.0,
                Mz1 = 0.0,
                Nx2,
                Vy2,
                Vz2,
                Mx2 = 0.0,
                My2 = 0.0,
                Mz2 = 0.0;
            double Ln, R1o, R2o, f01, f02;
            
            int n1, n2;
            //todo:init
            for (int lc = 0; lc < nL; lc++) //><
            {
                for (int n = 0; n < nE; n++) //><
                {
                    double[] t = Coordtrans.coordTrans(xyz, L[n], N1[n], N2[n], p[n]); //V

                    eqFMech[lc, n, 0] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0; //><
                    eqFMech[lc, n, 1] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0; //><
                    eqFMech[lc, n, 2] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0; //><

                    eqFMech[lc, n, 3] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((-t[3] * t[7] + t[4] * t[6]) * gY[lc] +
                                         (-t[3] * t[8] + t[5] * t[6]) * gZ[lc]); //><
                    eqFMech[lc, n, 4] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((-t[4] * t[6] + t[3] * t[7]) * gX[lc] +
                                         (-t[4] * t[8] + t[5] * t[7]) * gZ[lc]); //><
                    eqFMech[lc, n, 5] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((-t[5] * t[6] + t[3] * t[8]) * gX[lc] +
                                         (-t[5] * t[7] + t[4] * t[8]) * gY[lc]); //><

                    eqFMech[lc, n, 6] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0; //><
                    eqFMech[lc, n, 7] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0; //><
                    eqFMech[lc, n, 8] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0; //><

                    eqFMech[lc, n, 9] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((t[3] * t[7] - t[4] * t[6]) * gY[lc] +
                                         (t[3] * t[8] - t[5] * t[6]) * gZ[lc]); //><
                    eqFMech[lc, n, 10] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                         ((t[4] * t[6] - t[3] * t[7]) * gX[lc] +
                                          (t[4] * t[8] - t[5] * t[7]) * gZ[lc]); //><
                    eqFMech[lc, n, 11] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                         ((t[5] * t[6] - t[3] * t[8]) * gX[lc] +
                                          (t[5] * t[7] - t[4] * t[8]) * gY[lc]); //><
                }

                ////UnV we don't have nodeloads
                //for (int i = 0; i < nF[lc]; i++)
                //{
                //    NodeLoad nodeLoad = loadCases[lc].NodeLoads[i];
                //    int j = nodeLoad.NodeIdx;
                //    if (j < 0 || j > nN)
                //        Console.WriteLine(
                //            "\n  error in node load data: node number out of range ... Node : {0}\n   Perhaps you did not specify {1} node loads \n  or perhaps the Input Data file is missing expected data.\n",
                //            j, nF[lc]);

                //    FMech[lc, 6 * j + 0] = nodeLoad.Load.X;
                //    FMech[lc, 6 * j + 1] = nodeLoad.Load.Y;
                //    FMech[lc, 6 * j + 2] = nodeLoad.Load.Z;
                //    FMech[lc, 6 * j + 3] = nodeLoad.Moment.X;
                //    FMech[lc, 6 * j + 4] = nodeLoad.Moment.Y;
                //    FMech[lc, 6 * j + 5] = nodeLoad.Moment.Z;

                //    if (Common.isZeroVector(nodeLoad.Load) && Common.isZeroVector(nodeLoad.Load))
                //        Console.WriteLine("\n   Warning: All node loads applied at node %d  are zero\n", j);
                //}
                ////

                ////V
                for (int i = 0; i < nU[lc]; i++)
                {
                    UniformLoad uniformLoad = loadCases[lc].UniformLoads[i];
                    int n = uniformLoad.ElementIdx;
                    if (n < 0 || n > nE)
                        Console.WriteLine("\n  error in uniform distributed loads: element number %d is out of range\n",
                            n);

                    U[lc, i, 0] = (float) n;
                    U[lc, i, 1] = uniformLoad.Load.X;
                    U[lc, i, 2] = uniformLoad.Load.Y;
                    U[lc, i, 3] = uniformLoad.Load.Z;

                    if (Common.isZeroVector(uniformLoad.Load))
                        Console.WriteLine("\n   Warning: All distributed loads applied to frame element %d  are zero\n",
                            n);

                    Nx1 = Nx2 = U[lc, i, 1] * Le[n] / 2.0;
                    Vy1 = Vy2 = U[lc, i, 2] * Le[n] / 2.0;
                    Vz1 = Vz2 = U[lc, i, 3] * Le[n] / 2.0;
                    Mx1 = Mx2 = 0.0;
                    My1 = -U[lc, i, 3] * Le[n] * Le[n] / 12.0;
                    My2 = -My1;
                    Mz1 = U[lc, i, 2] * Le[n] * Le[n] / 12.0;
                    Mz2 = -Mz1;

                    double[] t = Coordtrans.coordTrans(xyz, L[n], N1[n], N2[n], p[n]); ////V

                    eqFMech[lc, n, 0] += (Nx1 * t[0] + Vy1 * t[3] + Vz1 * t[6]);
                    eqFMech[lc, n, 1] += (Nx1 * t[1] + Vy1 * t[4] + Vz1 * t[7]);
                    eqFMech[lc, n, 2] += (Nx1 * t[2] + Vy1 * t[5] + Vz1 * t[8]);
                    eqFMech[lc, n, 3] += (Mx1 * t[0] + My1 * t[3] + Mz1 * t[6]);
                    eqFMech[lc, n, 4] += (Mx1 * t[1] + My1 * t[4] + Mz1 * t[7]);
                    eqFMech[lc, n, 5] += (Mx1 * t[2] + My1 * t[5] + Mz1 * t[8]);

                    eqFMech[lc, n, 6] += (Nx2 * t[0] + Vy2 * t[3] + Vz2 * t[6]);
                    eqFMech[lc, n, 7] += (Nx2 * t[1] + Vy2 * t[4] + Vz2 * t[7]);
                    eqFMech[lc, n, 8] += (Nx2 * t[2] + Vy2 * t[5] + Vz2 * t[8]);
                    eqFMech[lc, n, 9] += (Mx2 * t[0] + My2 * t[3] + Mz2 * t[6]);
                    eqFMech[lc, n, 10] += (Mx2 * t[1] + My2 * t[4] + Mz2 * t[7]);
                    eqFMech[lc, n, 11] += (Mx2 * t[2] + My2 * t[5] + Mz2 * t[8]);
                    ////array element 0 and 6 are not exactly the same as they are in c program, because they are too close to zero, resulting in different approx value.
                }
                ////

                /* trapezoidally distributed loads ----------------------------- */
                ////V
                if (nW[lc] < 0 || nW[lc] > 10 * nE)
                    Console.WriteLine("\n  error: valid ranges for nW is 0 ... %d \n", 10 * nE);

                for (int i = 0; i < nW[lc]; i++)
                {
                    TrapLoad trapLoad = loadCases[lc].TrapLoads[i];
                    int n = trapLoad.ElementIdx;
                    if (n < 0 || n > nE)
                        Console.WriteLine(
                            "\n  error in trapezoidally-distributed loads: element number %d is out of range\n", n);
                    W[lc, i, 0] = (float) n;
                    W[lc, i, 1] = trapLoad.LocationStart.X;
                    W[lc, i, 2] = trapLoad.LocationEnd.X;
                    W[lc, i, 3] = trapLoad.LoadStart.X;
                    W[lc, i, 4] = trapLoad.LoadEnd.X;
                    W[lc, i, 5] = trapLoad.LocationStart.Y;
                    W[lc, i, 6] = trapLoad.LocationEnd.Y;
                    W[lc, i, 7] = trapLoad.LoadStart.Y;
                    W[lc, i, 8] = trapLoad.LoadEnd.Y;
                    W[lc, i, 9] = trapLoad.LocationStart.Z;
                    W[lc, i, 10] = trapLoad.LocationEnd.Z;
                    W[lc, i, 11] = trapLoad.LoadStart.Z;
                    W[lc, i, 12] = trapLoad.LoadEnd.Z;

                    Ln = L[n];

                    /* error checking */

                    if (W[lc, i, 4] == 0 && W[lc, i, 5] == 0 &&
                         W[lc, i, 8] == 0 && W[lc, i, 9] == 0 &&
                         W[lc, i, 12] == 0 && W[lc, i, 13] == 0)
                    {
                        Console.WriteLine("\n   Warning: All trapezoidal loads applied to frame element %d  are zero\n", n);
                        Console.WriteLine("     load case: %d , element %d , load %d\n ", lc, n, i);
                    }

                    if (W[lc, i, 1] < 0)
                        Console.WriteLine("\n   error in x-axis trapezoidal loads, load case: %d , element %d , load %d\n  starting location = %f < 0\n",
                        lc, n, i, W[lc, i, 1]);

                    if (W[lc, i, 1] > W[lc, i, 2])
                        Console.WriteLine("\n   error in x-axis trapezoidal loads, load case: %d , element %d , load %d\n  starting location = %f > ending location = %f \n",
                        lc, n, i, W[lc, i, 1], W[lc, i, 2]);

                    if (W[lc, i, 2] > Ln)
                        Console.WriteLine("\n   error in x-axis trapezoidal loads, load case: %d , element %d , load %d\n ending location = %f > L (%f) \n",
                        lc, n, i, W[lc, i, 2], Ln);

                    if (W[lc, i, 5] < 0)
                        Console.WriteLine("\n   error in y-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f < 0\n",
                        lc, n, i, W[lc, i, 5]);

                    if (W[lc, i, 5] > W[lc, i, 7])
                        Console.WriteLine("\n   error in y-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f > ending location = %f \n",
                        lc, n, i, W[lc, i, 5], W[lc, i, 6]);

                    if (W[lc, i, 6] > Ln)
                        Console.WriteLine("\n   error in y-axis trapezoidal loads, load case: %d , element %d , load %d\n ending location = %f > L (%f) \n",
                        lc, n, i, W[lc, i, 6], Ln);

                    if (W[lc, i, 9] < 0)
                        Console.WriteLine("\n   error in z-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f < 0\n",
                        lc, n, i, W[lc, i, 9]);

                    if (W[lc, i, 9] > W[lc, i, 10])
                        Console.WriteLine("\n   error in z-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f > ending location = %f \n",
                        lc, n, i, W[lc, i, 9], W[lc, i, 10]);

                    if (W[lc, i, 10] > Ln)
                        Console.WriteLine("\n   error in z-axis trapezoidal loads, load case: %d , element %d , load %d\n ending location = %f > L (%f) \n", lc, n, i, W[lc, i, 10], Ln);

                    if (shear)
                    {
                        Ksy = (12.0 * E[n] * Iz[n]) / (G[n] * Asy[n] * Le[n] * Le[n]);
                        Ksz = (12.0 * E[n] * Iy[n]) / (G[n] * Asz[n] * Le[n] * Le[n]);
                    }
                    else
                    {
                        Ksy = Ksz = 0.0;
                    }

                    /* x-axis trapezoidal loads (along the frame element length) */
                    x1 = W[lc, i, 1]; x2 = W[lc, i, 2];
                    w1 = W[lc, i, 3]; w2 = W[lc, i, 4];

                    Nx1 = (3.0 * (w1 + w2) * Ln * (x2 - x1) - (2.0 * w2 + w1) * x2 * x2 + (w2 - w1) * x2 * x1 + (2.0 * w1 + w2) * x1 * x1) / (6.0 * Ln);
                    Nx2 = (-(2.0 * w1 + w2) * x1 * x1 + (2.0 * w2 + w1) * x2 * x2 - (w2 - w1) * x1 * x2) / (6.0 * Ln);

                    /* y-axis trapezoidal loads (across the frame element length) */
                    x1 = W[lc, i, 5]; x2 = W[lc, i, 6];
                    w1 = W[lc, i, 7]; w2 = W[lc, i, 8];

                    R1o = ((2.0 * w1 + w2) * x1 * x1 - (w1 + 2.0 * w2) * x2 * x2 +
                           3.0 * (w1 + w2) * Ln * (x2 - x1) - (w1 - w2) * x1 * x2) / (6.0 * Ln);
                    R2o = ((w1 + 2.0 * w2) * x2 * x2 + (w1 - w2) * x1 * x2 -
                           (2.0 * w1 + w2) * x1 * x1) / (6.0 * Ln);

                    f01 = (3.0 * (w2 + 4.0 * w1) * x1 * x1 * x1 * x1 - 3.0 * (w1 + 4.0 * w2) * x2 * x2 * x2 * x2//NoAccu
                            - 15.0 * (w2 + 3.0 * w1) * Ln * x1 * x1 * x1 + 15.0 * (w1 + 3.0 * w2) * Ln * x2 * x2 * x2
                           - 3.0 * (w1 - w2) * x1 * x2 * (x1 * x1 + x2 * x2)
                           + 20.0 * (w2 + 2.0 * w1) * Ln * Ln * x1 * x1 - 20.0 * (w1 + 2.0 * w2) * Ln * Ln * x2 * x2
                           + 15.0 * (w1 - w2) * Ln * x1 * x2 * (x1 + x2)
                           - 3.0 * (w1 - w2) * x1 * x1 * x2 * x2 - 20.0 * (w1 - w2) * Ln * Ln * x1 * x2) / 360.0;

                    f02 = (3.0 * (w2 + 4.0 * w1) * x1 * x1 * x1 * x1 - 3.0 * (w1 + 4.0 * w2) * x2 * x2 * x2 * x2//NoAccu
                            - 3.0 * (w1 - w2) * x1 * x2 * (x1 * x1 + x2 * x2)
                            - 10.0 * (w2 + 2.0 * w1) * Ln * Ln * x1 * x1 + 10.0 * (w1 + 2.0 * w2) * Ln * Ln * x2 * x2
                           - 3.0 * (w1 - w2) * x1 * x1 * x2 * x2 + 10.0 * (w1 - w2) * Ln * Ln * x1 * x2) / 360.0;

                    Mz1 = -(4.0 * f01 + 2.0 * f02 + Ksy * (f01 - f02)) / (Ln * Ln * (1.0 + Ksy));
                    Mz2 = -(2.0 * f01 + 4.0 * f02 - Ksy * (f01 - f02)) / (Ln * Ln * (1.0 + Ksy));

                    Vy1 = R1o + Mz1 / Ln + Mz2 / Ln;
                    Vy2 = R2o - Mz1 / Ln - Mz2 / Ln;

                    x1 = W[lc, i, 9]; x2 = W[lc, i, 10];
                    w1 = W[lc, i, 11]; w2 = W[lc, i, 12];

                    R1o = ((2.0 * w1 + w2) * x1 * x1 - (w1 + 2.0 * w2) * x2 * x2 +
                           3.0 * (w1 + w2) * Ln * (x2 - x1) - (w1 - w2) * x1 * x2) / (6.0 * Ln);
                    R2o = ((w1 + 2.0 * w2) * x2 * x2 + (w1 - w2) * x1 * x2 -
                           (2.0 * w1 + w2) * x1 * x1) / (6.0 * Ln);

                    f01 = (3.0 * (w2 + 4.0 * w1) * x1 * x1 * x1 * x1 - 3.0 * (w1 + 4.0 * w2) * x2 * x2 * x2 * x2
                                                                     - 15.0 * (w2 + 3.0 * w1) * Ln * x1 * x1 * x1 + 15.0 * (w1 + 3.0 * w2) * Ln * x2 * x2 * x2
                           - 3.0 * (w1 - w2) * x1 * x2 * (x1 * x1 + x2 * x2)
                           + 20.0 * (w2 + 2.0 * w1) * Ln * Ln * x1 * x1 - 20.0 * (w1 + 2.0 * w2) * Ln * Ln * x2 * x2
                           + 15.0 * (w1 - w2) * Ln * x1 * x2 * (x1 + x2)
                           - 3.0 * (w1 - w2) * x1 * x1 * x2 * x2 - 20.0 * (w1 - w2) * Ln * Ln * x1 * x2) / 360.0;

                    f02 = (3.0 * (w2 + 4.0 * w1) * x1 * x1 * x1 * x1 - 3.0 * (w1 + 4.0 * w2) * x2 * x2 * x2 * x2
                                                                     - 3.0 * (w1 - w2) * x1 * x2 * (x1 * x1 + x2 * x2)
                                                                     - 10.0 * (w2 + 2.0 * w1) * Ln * Ln * x1 * x1 + 10.0 * (w1 + 2.0 * w2) * Ln * Ln * x2 * x2
                           - 3.0 * (w1 - w2) * x1 * x1 * x2 * x2 + 10.0 * (w1 - w2) * Ln * Ln * x1 * x2) / 360.0;

                    My1 = (4.0 * f01 + 2.0 * f02 + Ksz * (f01 - f02)) / (Ln * Ln * (1.0 + Ksz));
                    My2 = (2.0 * f01 + 4.0 * f02 - Ksz * (f01 - f02)) / (Ln * Ln * (1.0 + Ksz));

                    Vz1 = R1o - My1 / Ln - My2 / Ln;
                    Vz2 = R2o + My1 / Ln + My2 / Ln;

                    n1 = N1[n]; n2 = N2[n];

                    double[] t = Coordtrans.coordTrans(xyz, Ln, n1, n2, p[n]);

                    /* {F} = [T]'{Q} */
                    eqFMech[lc, n, 0] += (Nx1 * t[0] + Vy1 * t[3] + Vz1 * t[6]);
                    eqFMech[lc, n, 1] += (Nx1 * t[1] + Vy1 * t[4] + Vz1 * t[7]);
                    eqFMech[lc, n, 2] += (Nx1 * t[2] + Vy1 * t[5] + Vz1 * t[8]);
                    eqFMech[lc, n, 3] += (Mx1 * t[0] + My1 * t[3] + Mz1 * t[6]);
                    eqFMech[lc, n, 4] += (Mx1 * t[1] + My1 * t[4] + Mz1 * t[7]);
                    eqFMech[lc, n, 5] += (Mx1 * t[2] + My1 * t[5] + Mz1 * t[8]);

                    eqFMech[lc, n, 6] += (Nx2 * t[0] + Vy2 * t[3] + Vz2 * t[6]);
                    eqFMech[lc, n, 7] += (Nx2 * t[1] + Vy2 * t[4] + Vz2 * t[7]);
                    eqFMech[lc, n, 8] += (Nx2 * t[2] + Vy2 * t[5] + Vz2 * t[8]);
                    eqFMech[lc, n, 9] += (Mx2 * t[0] + My2 * t[3] + Mz2 * t[6]);
                    eqFMech[lc, n, 10] += (Mx2 * t[1] + My2 * t[4] + Mz2 * t[7]);
                    eqFMech[lc, n, 11] += (Mx2 * t[2] + My2 * t[5] + Mz2 * t[8]);
                }
                ////

                ////Jumped over interior loads, temp loads and prescribed displacements

                ////NoV not confident they are correct, Fmech shouldn't be used.
                for (int n = 0; n < nE; n++)
                {
                    n1 = N1[n]; n2 = N2[n];
                    for (int i = 0; i < 6; i++) FMech[lc, 6 * n1 + i] += eqFMech[lc, n, i];
                    for (int i = 6; i < 12; i++) FMech[lc, 6 * n2 - 6 + i] += eqFMech[lc, n, i];
                }////V
            }
        }

        //Dealing with dynamic data, todo check if it is true
        //private void ReadMassData()
        //{
        //    int i, j, jnt, m, b, nA;
        //    int full_len = 0, len = 0;
        //    double totalMass = 0.0;
        //    double structMass = 0.0;
        //}


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
