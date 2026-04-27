using Frame3ddn.Model;
using System;
using System.Collections.Generic;

namespace Frame3ddn
{
    class Frame3dd
    {
        public static void ElasticK(double[,] k, List<Vec3> xyz, float[] r,
            double L, double Le,
            int n1, int n2,
            double Ax, double Asy, double Asz,
            double J, double Iy, double Iz,
            double E, double G, float p,
            bool shear)
        {
            double Ksy, Ksz;       /* shear deformatn coefficients	*/
            int i, j;

            double[] t = CoordinateTransform.CoordTrans(xyz, L, n1, n2, p);

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

            k = CoordinateTransform.Atma(t, k, r[n1], r[n2]);

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

        /// <summary>
        /// Geometric stiffness matrix in global coordinates. Adds the geometric stiffness
        /// contribution (axial-force-dependent) onto the existing element stiffness <paramref name="k"/>,
        /// matching upstream <c>geometric_K</c> in frame3dd.c. T is the element axial force
        /// (compressive negative, per upstream convention <c>-Q[i][1]</c>).
        /// </summary>
        public static void GeometricK(double[,] k, List<Vec3> xyz, float[] r,
            double L, double Le, int n1, int n2,
            double Ax, double Asy, double Asz,
            double J, double Iy, double Iz,
            double E, double G, float p, double T,
            bool shear)
        {
            double[] t = CoordinateTransform.CoordTrans(xyz, L, n1, n2, p);
            double[,] kg = new double[12, 12];
            double Ksy, Ksz, Dsy, Dsz;

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

            // axial DoFs left at 0 in upstream (commented out as // T/L)
            kg[0, 0] = kg[6, 6] = 0.0;
            kg[0, 6] = kg[6, 0] = 0.0;

            kg[1, 1] = kg[7, 7] = T / L * (1.2 + 2.0 * Ksy + Ksy * Ksy) / Dsy;
            kg[2, 2] = kg[8, 8] = T / L * (1.2 + 2.0 * Ksz + Ksz * Ksz) / Dsz;
            kg[3, 3] = kg[9, 9] = T / L * J / Ax;
            kg[4, 4] = kg[10, 10] = T * L * (2.0 / 15.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz;
            kg[5, 5] = kg[11, 11] = T * L * (2.0 / 15.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy;

            kg[4, 2] = kg[2, 4] = kg[10, 2] = kg[2, 10] = -T / 10.0 / Dsz;
            kg[8, 4] = kg[4, 8] = kg[10, 8] = kg[8, 10] = T / 10.0 / Dsz;
            kg[5, 1] = kg[1, 5] = kg[11, 1] = kg[1, 11] = T / 10.0 / Dsy;
            kg[7, 5] = kg[5, 7] = kg[11, 7] = kg[7, 11] = -T / 10.0 / Dsy;

            kg[3, 9] = kg[9, 3] = -kg[3, 3];

            kg[7, 1] = kg[1, 7] = -T / L * (1.2 + 2.0 * Ksy + Ksy * Ksy) / Dsy;
            kg[8, 2] = kg[2, 8] = -T / L * (1.2 + 2.0 * Ksz + Ksz * Ksz) / Dsz;

            kg[10, 4] = kg[4, 10] = -T * L * (1.0 / 30.0 + Ksz / 6.0 + Ksz * Ksz / 12.0) / Dsz;
            kg[11, 5] = kg[5, 11] = -T * L * (1.0 / 30.0 + Ksy / 6.0 + Ksy * Ksy / 12.0) / Dsy;

            kg = CoordinateTransform.Atma(t, kg, r[n1], r[n2]);

            // Enforce symmetry (mirrors upstream).
            for (int i = 0; i < 12; i++)
                for (int j = i + 1; j < 12; j++)
                    if (kg[i, j] != kg[j, i])
                        kg[i, j] = kg[j, i] = 0.5 * (kg[i, j] + kg[j, i]);

            // Add geometric stiffness onto the existing elastic stiffness.
            for (int i = 0; i < 12; i++)
                for (int j = 0; j < 12; j++)
                    k[i, j] += kg[i, j];
        }

        /// <summary>
        /// Lumped element mass matrix in global coordinates. Mirrors upstream <c>lumped_M</c>
        /// in frame3dd.c. Translational mass goes on the diagonal directly (rotationally
        /// invariant); rotational inertia is projected onto the global axes via the direction
        /// cosines from <see cref="CoordinateTransform.CoordTrans"/>, so no Atma call is needed.
        /// </summary>
        public static void LumpedM(double[,] m, List<Vec3> xyz, double L, int n1, int n2,
            double Ax, double J, double Iy, double Iz, float p, double d, double EMs)
        {
            double[] t = CoordinateTransform.CoordTrans(xyz, L, n1, n2, p);

            double trans = (d * Ax * L + EMs) / 2.0;       // translational mass at each node
            double ry = d * Iy * L / 2.0;
            double rz = d * Iz * L / 2.0;
            double po = d * L * J / 2.0;                   // polar inertia (simple x-section)

            for (int i = 0; i < 12; i++)
                for (int j = 0; j < 12; j++)
                    m[i, j] = 0.0;

            m[0, 0] = m[1, 1] = m[2, 2] = m[6, 6] = m[7, 7] = m[8, 8] = trans;

            // Rotational diagonal: project polar/y/z inertia onto global axes.
            m[3, 3] = m[9, 9] = po * t[0] * t[0] + ry * t[3] * t[3] + rz * t[6] * t[6];
            m[4, 4] = m[10, 10] = po * t[1] * t[1] + ry * t[4] * t[4] + rz * t[7] * t[7];
            m[5, 5] = m[11, 11] = po * t[2] * t[2] + ry * t[5] * t[5] + rz * t[8] * t[8];

            m[3, 4] = m[4, 3] = m[9, 10] = m[10, 9] = po * t[0] * t[1] + ry * t[3] * t[4] + rz * t[6] * t[7];
            m[3, 5] = m[5, 3] = m[9, 11] = m[11, 9] = po * t[0] * t[2] + ry * t[3] * t[5] + rz * t[6] * t[8];
            m[4, 5] = m[5, 4] = m[10, 11] = m[11, 10] = po * t[1] * t[2] + ry * t[4] * t[5] + rz * t[7] * t[8];
        }

        /// <summary>
        /// Consistent element mass matrix in global coordinates. Mirrors upstream
        /// <c>consistent_M</c> in frame3dd.c. Builds the standard 12×12 consistent mass matrix
        /// in local element coords, then rotates to global via <see cref="CoordinateTransform.Atma"/>.
        /// Shear deformations are not included (matching upstream).
        /// </summary>
        public static void ConsistentM(double[,] m, List<Vec3> xyz, float[] r, double L, int n1, int n2,
            double Ax, double J, double Iy, double Iz, float p, double d, double EMs)
        {
            double[] t = CoordinateTransform.CoordTrans(xyz, L, n1, n2, p);

            double mt = d * Ax * L;
            double ry = d * Iy;
            double rz = d * Iz;
            double po = d * J * L;

            for (int i = 0; i < 12; i++)
                for (int j = 0; j < 12; j++)
                    m[i, j] = 0.0;

            m[0, 0] = m[6, 6] = mt / 3.0;
            m[1, 1] = m[7, 7] = 13.0 * mt / 35.0 + 6.0 * rz / (5.0 * L);
            m[2, 2] = m[8, 8] = 13.0 * mt / 35.0 + 6.0 * ry / (5.0 * L);
            m[3, 3] = m[9, 9] = po / 3.0;
            m[4, 4] = m[10, 10] = mt * L * L / 105.0 + 2.0 * L * ry / 15.0;
            m[5, 5] = m[11, 11] = mt * L * L / 105.0 + 2.0 * L * rz / 15.0;

            m[4, 2] = m[2, 4] = -11.0 * mt * L / 210.0 - ry / 10.0;
            m[5, 1] = m[1, 5] = 11.0 * mt * L / 210.0 + rz / 10.0;
            m[6, 0] = m[0, 6] = mt / 6.0;

            m[7, 5] = m[5, 7] = 13.0 * mt * L / 420.0 - rz / 10.0;
            m[8, 4] = m[4, 8] = -13.0 * mt * L / 420.0 + ry / 10.0;
            m[9, 3] = m[3, 9] = po / 6.0;
            m[10, 2] = m[2, 10] = 13.0 * mt * L / 420.0 - ry / 10.0;
            m[11, 1] = m[1, 11] = -13.0 * mt * L / 420.0 + rz / 10.0;

            m[10, 8] = m[8, 10] = 11.0 * mt * L / 210.0 + ry / 10.0;
            m[11, 7] = m[7, 11] = -11.0 * mt * L / 210.0 - rz / 10.0;

            m[7, 1] = m[1, 7] = 9.0 * mt / 70.0 - 6.0 * rz / (5.0 * L);
            m[8, 2] = m[2, 8] = 9.0 * mt / 70.0 - 6.0 * ry / (5.0 * L);
            m[10, 4] = m[4, 10] = -L * L * mt / 140.0 - ry * L / 30.0;
            m[11, 5] = m[5, 11] = -L * L * mt / 140.0 - rz * L / 30.0;

            // Add the lumped extra-mass contribution to the translational diagonal of each
            // node (rotational inertia of extra beam mass is neglected, matching upstream).
            for (int i = 0; i < 3; i++) m[i, i] += 0.5 * EMs;
            for (int i = 6; i < 9; i++) m[i, i] += 0.5 * EMs;

            double[,] mGlobal = CoordinateTransform.Atma(t, m, r[n1], r[n2]);

            // Enforce symmetry — Atma can leave tiny asymmetries from accumulated rounding.
            for (int i = 0; i < 12; i++)
                for (int j = i + 1; j < 12; j++)
                    if (mGlobal[i, j] != mGlobal[j, i])
                        mGlobal[i, j] = mGlobal[j, i] = 0.5 * (mGlobal[i, j] + mGlobal[j, i]);

            // Copy globalised values back into the caller's buffer (Atma allocates a new array).
            for (int i = 0; i < 12; i++)
                for (int j = 0; j < 12; j++)
                    m[i, j] = mGlobal[i, j];
        }

        /// <summary>
        /// Assembles the full system mass matrix. Mirrors upstream <c>assemble_M</c>.
        /// <c>lump=true</c> selects the lumped mass matrix, <c>false</c> the consistent matrix.
        /// </summary>
        public static double[,] AssembleM(
            int DoF, int nN, int nE,
            List<Vec3> xyz, float[] r, List<double> L,
            List<int> N1, List<int> N2,
            List<float> Ax, List<float> Jx, List<float> Iy, List<float> Iz, List<float> p,
            List<float> density, List<float> EMs,
            List<float> NMs, List<float> NMx, List<float> NMy, List<float> NMz,
            bool lump)
        {
            double[,] M = new double[DoF, DoF];
            double[,] m = new double[12, 12];
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
                if (lump)
                    LumpedM(m, xyz, L[i], N1[i], N2[i], Ax[i], Jx[i], Iy[i], Iz[i], p[i], density[i], EMs[i]);
                else
                    ConsistentM(m, xyz, r, L[i], N1[i], N2[i], Ax[i], Jx[i], Iy[i], Iz[i], p[i], density[i], EMs[i]);

                for (int l = 0; l < 12; l++)
                {
                    int ii = ind[l, i];
                    for (int ll = 0; ll < 12; ll++)
                    {
                        int jj = ind[ll, i];
                        M[ii, jj] += m[l, ll];
                    }
                }
            }

            // Add per-node concentrated mass and rotational inertia.
            for (int n = 0; n < nN; n++)
            {
                int b = 6 * n;
                M[b + 0, b + 0] += NMs[n];
                M[b + 1, b + 1] += NMs[n];
                M[b + 2, b + 2] += NMs[n];
                M[b + 3, b + 3] += NMx[n];
                M[b + 4, b + 4] += NMy[n];
                M[b + 5, b + 5] += NMz[n];
            }

            for (int i = 0; i < DoF; i++)
            {
                if (M[i, i] <= 0.0)
                {
                    Console.WriteLine($"  warning: Non pos-def mass matrix  M[{i}][{i}] = {M[i, i]}");
                }
            }

            return M;
        }

        public static double[,] AssembleK(
            int DoF, int nE,
            List<Vec3> xyz, float[] r, List<double> L, List<double> Le,
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
                    GeometricK(k, xyz, r, L[i], Le[i], N1[i], N2[i],
                        Ax[i], Asy[i], Asz[i], Jx[i], Iy[i], Iz[i],
                        E[i], G[i], p[i], -Q[i, 0], shear);
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
                if (!Common.IsDoubleZero(q[i]))
                {
                    errF = F[i];
                    for (j = 0; j < DoF; j++)
                    {
                        if (!Common.IsDoubleZero(q[j]))
                        {   // K_qq in upper triangle only
                            if (i <= j) errF -= K[i, j] * D[j];
                            else errF -= K[j, i] * D[j];
                        }
                    }
                    for (j = 0; j < DoF; j++)
                        if (!Common.IsDoubleZero(r[j])) errF -= K[i, j] * D[j];
                }
                dF[i] = errF;
            }

            for (i = 0; i < DoF; i++)
                if (!Common.IsDoubleZero(q[i]))
                    ss_dF += (dF[i] * dF[i]);
            for (i = 0; i < DoF; i++)
                if (!Common.IsDoubleZero(q[i]))
                    ss_F += (F[i] * F[i]);

            return (Math.Sqrt(ss_dF) / Math.Sqrt(ss_F));	// convergence criterion
        }

        public static void ElementEndForces(double[,] Q, int nE, List<Vec3> xyz, List<double> L, List<double> Le, List<int> N1, List<int> N2,
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

        private static void FrameElementForce(double[] s, List<Vec3> xyz, double L, double Le, int n1, int n2,
            float Ax, float Asy, float Asz, float J, float Iy, float Iz, float E, float G, float p, double[] fT, double[] fM,
            double[] D, bool shear, bool geom, double axialStrain)//return s
        {
            double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12,
                delta = 0.0,        /* stretch in the frame element */
                Ksy, Ksz, Dsy, Dsz, /* shear deformation coeff's	*/
                T = 0.0;        /* axial force for geometric stiffness */

            double f1 = 0, f2 = 0, f3 = 0, f4 = 0, f5 = 0, f6 = 0,
                f7 = 0, f8 = 0, f9 = 0, f10 = 0, f11 = 0, f12 = 0;

            double[] t = CoordinateTransform.CoordTrans(xyz, L, n1, n2, p);

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
                if (!Common.IsDoubleZero(q[i]))
                {
                    sdp = b[i];
                    for (j = 0; j < n; j++)
                    {
                        if (!Common.IsDoubleZero(q[j]))
                        {   // A_qq in upper triangle only
                            if (i <= j)
                            {
                                sdp -= A[i, j] * x[j];
                            }
                            else sdp -= A[j, i] * x[j];
                        }
                    }
                    for (j = 0; j < n; j++)
                        if (!Common.IsDoubleZero(r[j]))
                            sdp -= A[i, j] * x[j];
                    dx[i] = sdp;
                } // else dx[i] = 0.0; // x[i];
            }

            // solve for the residual error term, A is already factored
            LdlDcmpPm(A, n, d, dx, dx, dc, q, r, 0, 1);

            for (i = 0; i < n; i++)
                if (!Common.IsDoubleZero(q[i]))
                    rms_resid_new += dx[i] * dx[i];

            rms_resid_new = Math.Sqrt(rms_resid_new / (double)n);

            int ok = 0;
            if (rms_resid_new / rmsResid < 0.90)
            { /*  enough improvement    */
                for (i = 0; i < n; i++)
                {   /*  update the solution 2014-05-14   */
                    if (!Common.IsDoubleZero(q[i]))
                        x[i] += dx[i];
                    if (!Common.IsDoubleZero(r[i]))
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

                    if (!Common.IsDoubleZero(q[j]))
                    { /* reduce column j, except where q[i]==0	*/

                        for (m = 0, i = 0; i < j; i++)  /* scan the sky-line	*/
                            if (Common.IsDoubleZero(A[i, j]))
                                ++m;
                            else
                                break;

                        for (i = m; i < j; i++)
                        {
                            if (!Common.IsDoubleZero(q[i]))
                            {
                                A[j, i] = A[i, j];
                                for (k = m; k < i; k++)
                                    if (!Common.IsDoubleZero(q[k]))
                                        A[j, i] -= A[j, k] * A[i, k];
                            }
                        }

                        d[j] = A[j, j];
                        for (i = m; i < j; i++)
                            if (!Common.IsDoubleZero(q[i]))
                                d[j] -= A[j, i] * A[j, i] / d[i];
                        for (i = m; i < j; i++)
                            if (!Common.IsDoubleZero(q[i]))
                                A[j, i] /= d[i];

                        if (Common.IsDoubleZero(d[j]))
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
                    if (!Common.IsDoubleZero(q[i]))
                    {
                        x[i] = b[i];
                        for (j = 0; j < n; j++)
                            if (!Common.IsDoubleZero(r[j]))
                                x[i] -= A[i, j] * x[j];
                    }
                }

                /* {x} is run through the same forward reduction as was [A] */
                for (i = 0; i < n; i++)
                    if (!Common.IsDoubleZero(q[i]))
                        for (j = 0; j < i; j++)
                            if (!Common.IsDoubleZero(q[j]))
                                x[i] -= A[i, j] * x[j];

                for (i = 0; i < n; i++)
                    if (!Common.IsDoubleZero(q[i]))
                        x[i] /= d[i];

                /* now back substitution is conducted on {x};  [A] is preserved */

                for (i = n - 1; i > 0; i--)
                    if (!Common.IsDoubleZero(q[i]))
                        for (j = 0; j < i; j++)
                            if (!Common.IsDoubleZero(q[j]))
                                x[j] -= A[i, j] * x[i];

                /* finally, evaluate c_r	*/
                for (i = 0; i < n; i++)
                {
                    c[i] = 0.0;
                    if (!Common.IsDoubleZero(r[i]))
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
