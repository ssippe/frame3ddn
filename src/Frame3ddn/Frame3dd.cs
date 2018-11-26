using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn
{
    class Frame3dd
    {
        public static void ElasticK(double[,] k, List<Vec3Float> xyz, float[] r,
            double L, double Le,
            int n1, int n2,
            double Ax, double Asy, double Asz,
            double J, double Iy, double Iz,
            double E, double G, float p,
            bool shear)
        {
            double Ksy, Ksz;       /* shear deformatn coefficients	*/
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
            k[1, 1] = k[7, 7] = 12.0 * E * Iz / (Le * Le * Le * (1.0 + Ksy));
            k[2, 2] = k[8, 8] = 12.0 * E * Iy / (Le * Le * Le * (1.0 + Ksz));
            k[3, 3] = k[9, 9] = G * J / Le;
            k[4, 4] = k[10, 10] = (4.0 + Ksz) * E * Iy / (Le * (1.0 + Ksz));
            k[5, 5] = k[11, 11] = (4.0 + Ksy) * E * Iz / (Le * (1.0 + Ksy));

            k[4, 2] = k[2, 4] = -6.0 * E * Iy / (Le * Le * (1.0 + Ksz));
            k[5, 1] = k[1, 5] = 6.0 * E * Iz / (Le * Le * (1.0 + Ksy));
            k[6, 0] = k[0, 6] = -k[0, 0];

            k[11, 7] = k[7, 11] = k[7, 5] = k[5, 7] = -k[5, 1];
            k[10, 8] = k[8, 10] = k[8, 4] = k[4, 8] = -k[4, 2];
            k[9, 3] = k[3, 9] = -k[3, 3];
            k[10, 2] = k[2, 10] = k[4, 2];
            k[11, 1] = k[1, 11] = k[5, 1];

            k[7, 1] = k[1, 7] = -k[1, 1];
            k[8, 2] = k[2, 8] = -k[2, 2];
            k[10, 4] = k[4, 10] = (2.0 - Ksz) * E * Iy / (Le * (1.0 + Ksz));
            k[11, 5] = k[5, 11] = (2.0 - Ksy) * E * Iz / (Le * (1.0 + Ksy));

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
                            Console.WriteLine($"elastic_K: element stiffness matrix not symetric ...");
                            Console.WriteLine($" ... k[{i}][{j}] = {k[i, j]} ");
                            Console.WriteLine($" ... k[{j}][{i}] = {k[j, i]}   ");
                            Console.WriteLine($" ... relative error = {Math.Abs(k[i, j] / k[j, i] - 1.0)} ");
                            Console.WriteLine($" ... element matrix saved in file 'kt'");
                        }

                        k[i, j] = k[j, i] = 0.5 * (k[i, j] + k[j, i]);
                    }
                }
            }
        }

        public static double[,] AssembleK(
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
                Frame3dd.ElasticK(k, xyz, r, L[i], Le[i], N1[i], N2[i],
                    Ax[i], Asy[i], Asz[i], Jx[i], Iy[i], Iz[i], E[i], G[i], p[i], shear);

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


        public static double EquilibriumError(double[] dF, double[] F, double[,] K, double[] D, int DoF, double[] q, float[] r)
        {
            double ss_dF = 0.0, //  sum of squares of dF
                ss_F = 0.0, //  sum of squares of F	
                errF = 0.0;
            int i, j;

            // compute equilibrium error at free coord's (q)
            for (i = 0; i < DoF; i++)
            {
                errF = 0.0;
                if (!Common.isDoubleZero(q[i]))
                {
                    errF = F[i];
                    for (j = 0; j < DoF; j++)
                    {
                        if (!Common.isDoubleZero(q[j]))
                        {   // K_qq in upper triangle only
                            if (i <= j) errF -= K[i, j] * D[j];
                            else errF -= K[j, i] * D[j];
                        }
                    }
                    for (j = 0; j < DoF; j++)
                        if (!Common.isDoubleZero(r[j])) errF -= K[i, j] * D[j];
                }
                dF[i] = errF;
            }

            for (i = 0; i < DoF; i++)
                if (!Common.isDoubleZero(q[i]))
                    ss_dF += (dF[i] * dF[i]);
            for (i = 0; i < DoF; i++)
                if (!Common.isDoubleZero(q[i]))
                    ss_F += (F[i] * F[i]);

            return (Math.Sqrt(ss_dF) / Math.Sqrt(ss_F));	// convergence criterion
        }

        public static void ElementEndForces(double[,] Q, int nE, List<Vec3Float> xyz, List<double> L, List<double> Le, List<int> N1, List<int> N2,
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

        private static void FrameElementForce(double[] s, List<Vec3Float> xyz, double L, double Le, int n1, int n2,
            float Ax, float Asy, float Asz, float J, float Iy, float Iz, float E, float G, float p, double[] fT, double[] fM,
            double[] D, bool shear, bool geom, double axialStrain)//return s
        {
            double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12,
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

            s[1] = -(12.0 * E * Iz / (Le * Le * Le * (1.0 + Ksy)) +
                   T / L * (1.2 + 2.0 * Ksy + Ksy * Ksy) / Dsy) *
                        ((d7 - d1) * t[3] + (d8 - d2) * t[4] + (d9 - d3) * t[5])
                + (6.0 * E * Iz / (Le * Le * (1.0 + Ksy)) + T / 10.0 / Dsy) *
                        ((d4 + d10) * t[6] + (d5 + d11) * t[7] + (d6 + d12) * t[8]);
            s[2] = -(12.0 * E * Iy / (Le * Le * Le * (1.0 + Ksz)) +
                  T / L * (1.2 + 2.0 * Ksz + Ksz * Ksz) / Dsz) *
                        ((d7 - d1) * t[6] + (d8 - d2) * t[7] + (d9 - d3) * t[8])
                - (6.0 * E * Iy / (Le * Le * (1.0 + Ksz)) + T / 10.0 / Dsz) *
                        ((d4 + d10) * t[3] + (d5 + d11) * t[4] + (d6 + d12) * t[5]);
            s[3] = -(G * J / Le) * ((d10 - d4) * t[0] + (d11 - d5) * t[1] + (d12 - d6) * t[2]);
            s[4] = (6.0 * E * Iy / (Le * Le * (1.0 + Ksz)) + T / 10.0 / Dsz) *
                        ((d7 - d1) * t[6] + (d8 - d2) * t[7] + (d9 - d3) * t[8])
                + ((4.0 + Ksz) * E * Iy / (Le * (1.0 + Ksz)) +
                    T * L * (2.0 / 15.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d4 * t[3] + d5 * t[4] + d6 * t[5])
                + ((2.0 - Ksz) * E * Iy / (Le * (1.0 + Ksz)) -
                    T * L * (1.0 / 30.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d10 * t[3] + d11 * t[4] + d12 * t[5]);
            s[5] = -(6.0 * E * Iz / (Le * Le * (1.0 + Ksy)) + T / 10.0 / Dsy) *
                        ((d7 - d1) * t[3] + (d8 - d2) * t[4] + (d9 - d3) * t[5])
                + ((4.0 + Ksy) * E * Iz / (Le * (1.0 + Ksy)) +
                    T * L * (2.0 / 15.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d4 * t[6] + d5 * t[7] + d6 * t[8])
                + ((2.0 - Ksy) * E * Iz / (Le * (1.0 + Ksy)) -
                    T * L * (1.0 / 30.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d10 * t[6] + d11 * t[7] + d12 * t[8]);
            s[6] = -s[0];
            s[7] = -s[1];
            s[8] = -s[2];
            s[9] = -s[3];

            s[10] = (6.0 * E * Iy / (Le * Le * (1.0 + Ksz)) + T / 10.0 / Dsz) *
                        ((d7 - d1) * t[6] + (d8 - d2) * t[7] + (d9 - d3) * t[8])
                + ((4.0 + Ksz) * E * Iy / (Le * (1.0 + Ksz)) +
                    T * L * (2.0 / 15.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d10 * t[3] + d11 * t[4] + d12 * t[5])
                + ((2.0 - Ksz) * E * Iy / (Le * (1.0 + Ksz)) -
                    T * L * (1.0 / 30.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz) *
                        (d4 * t[3] + d5 * t[4] + d6 * t[5]);
            s[11] = -(6.0 * E * Iz / (Le * Le * (1.0 + Ksy)) + T / 10.0 / Dsy) *
                        ((d7 - d1) * t[3] + (d8 - d2) * t[4] + (d9 - d3) * t[5])
                + ((4.0 + Ksy) * E * Iz / (Le * (1.0 + Ksy)) +
                    T * L * (2.0 / 15.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy) *
                        (d10 * t[6] + d11 * t[7] + d12 * t[8])
                + ((2.0 - Ksy) * E * Iz / (Le * (1.0 + Ksy)) -
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
        public static (int ok, double rmsResid) SolveSystem(double[,] K, double[] D, double[] F, double[] R, int DoF, double[] q, float[] r, int ok, double rmsResid)
        {
            double[] diag = new double[DoF];

            ok = LdlDcmpPm(K, DoF, diag, F, D, R, q, r, 1, 0);
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

        private static (int ok, double rmsResid) LdlMprovePm(double[,] A, int n, double[] d, double[] b, double[] x, double[] c, double[] q,
            float[] r, double rmsResid)
        {
            double sdp;     // accumulate the r.h.s. in double precision
            double rms_resid_new = 0.0; // the RMS error of the mprvd solution
            int j, i;

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
                            if (i <= j)
                            {
                                sdp -= A[i, j] * x[j];
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

        private static int LdlDcmpPm(double[,] A, int n, double[] d, double[] b, double[] x, double[] c, double[] q, float[] r, int reduce, int solve)
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

    }
}
