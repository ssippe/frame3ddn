using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Frame3ddn
{
    public class Frame3ddIO
    {
        public static void AssembleLoads(int nN, int nE, int nL, int DoF, List<Vec3Float> xyz, List<double> L, List<double> Le,
            List<int> N1, List<int> N2,
            List<float> Ax, List<float> Asy, List<float> Asz, List<float> Iy, List<float> Iz, List<float> E,
            List<float> G, List<float> p,
            List<float> d, List<float> gX, List<float> gY, List<float> gZ, bool shear, List<int> nF, List<int> nU, List<int> nW,
            double[,] FMech, double[,,] eqFMech,
            float[,,] U, float[,,] W, IReadOnlyList<LoadCase> loadCases)
        {
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
            for (int lc = 0; lc < nL; lc++)
            {
                for (int n = 0; n < nE; n++)
                {
                    double[] t = Coordtrans.coordTrans(xyz, L[n], N1[n], N2[n], p[n]);

                    eqFMech[lc, n, 0] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0;
                    eqFMech[lc, n, 1] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0;
                    eqFMech[lc, n, 2] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0;

                    eqFMech[lc, n, 3] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((-t[3] * t[7] + t[4] * t[6]) * gY[lc] +
                                         (-t[3] * t[8] + t[5] * t[6]) * gZ[lc]);
                    eqFMech[lc, n, 4] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((-t[4] * t[6] + t[3] * t[7]) * gX[lc] +
                                         (-t[4] * t[8] + t[5] * t[7]) * gZ[lc]);
                    eqFMech[lc, n, 5] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((-t[5] * t[6] + t[3] * t[8]) * gX[lc] +
                                         (-t[5] * t[7] + t[4] * t[8]) * gY[lc]);

                    eqFMech[lc, n, 6] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0;
                    eqFMech[lc, n, 7] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0;
                    eqFMech[lc, n, 8] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0;

                    eqFMech[lc, n, 9] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                        ((t[3] * t[7] - t[4] * t[6]) * gY[lc] +
                                         (t[3] * t[8] - t[5] * t[6]) * gZ[lc]);
                    eqFMech[lc, n, 10] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                         ((t[4] * t[6] - t[3] * t[7]) * gX[lc] +
                                          (t[4] * t[8] - t[5] * t[7]) * gZ[lc]);
                    eqFMech[lc, n, 11] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
                                         ((t[5] * t[6] - t[3] * t[8]) * gX[lc] +
                                          (t[5] * t[7] - t[4] * t[8]) * gY[lc]);
                }

                ////
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
                
                for (int i = 0; i < nU[lc]; i++)
                {
                    UniformLoad uniformLoad = loadCases[lc].UniformLoads[i];
                    int n = uniformLoad.ElementIdx;
                    if (n < 0 || n > nE)
                        Console.WriteLine("\n  error in uniform distributed loads: element number %d is out of range\n",
                            n);

                    U[lc, i, 0] = (float)n;
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

                    double[] t = Coordtrans.coordTrans(xyz, L[n], N1[n], N2[n], p[n]); //

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
                if (nW[lc] < 0 || nW[lc] > 10 * nE)
                    Console.WriteLine("\n  error: valid ranges for nW is 0 ... %d \n", 10 * nE);

                for (int i = 0; i < nW[lc]; i++)
                {
                    TrapLoad trapLoad = loadCases[lc].TrapLoads[i];
                    int n = trapLoad.ElementIdx;
                    if (n < 0 || n > nE)
                        Console.WriteLine(
                            "\n  error in trapezoidally-distributed loads: element number %d is out of range\n", n);
                    W[lc, i, 0] = (float)n;
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
                }//
            }
        }

        public static (List<NodeDisplacement> nodeDisplacements, List<FrameElementEndForce> frameElementEndForces, List<ReactionOutput> reactionOutputs) GetStaticResults(int nN, int nE, int nL, int lc, int DoF, List<int> J1, List<int> J2, double[] F,
            double[] D, double[] R, float[] r, double[,] Q, double error, int ok, int axialSign)
        {
            double disp;
            int i, j, n;

            List<NodeDisplacement> nodeDisplacements = new List<NodeDisplacement>();
            //Node Displacements
            for (j = 0; j < nN; j++)
            {
                disp = 0.0;
                for (i = 0; i < 6; i++)
                    disp += Math.Abs(D[6 * j + i]);
                if (disp > 0.0)
                {
                    double[] tempSingResult = new double[6];
                    for (i = 0; i < 6; i++)
                    {
                        if (Math.Abs(D[6 * j + i]) < 1.0e-8)
                        {
                            tempSingResult[i] = 0.0;
                        }
                        else
                        {
                            tempSingResult[i] = D[6 * j + i];
                        }
                    }
                    NodeDisplacement nodeDisplacement = new NodeDisplacement(j,
                        new Vec3(tempSingResult[0], tempSingResult[1], tempSingResult[2]),
                        new Vec3(tempSingResult[3], tempSingResult[4], tempSingResult[5]));
                    nodeDisplacements.Add(nodeDisplacement);
                }
            }

            List<FrameElementEndForce> frameElementEndForces = new List<FrameElementEndForce>();
            //Frame Element End Forces
            for (n = 0; n < nE; n++)
            {
                double nx;
                string nxType = "";
                double[] temp = new double[5];

                if (Math.Abs(Q[n, 0]) < 0.0001)
                {
                    nx = 0.0;
                }
                else
                {
                    nx = Q[n, 0];
                }

                if (Q[n, 0] >= 0.0001 && axialSign != 0)
                {
                    nxType = "c";
                }

                if (Q[n, 0] <= -0.0001 && axialSign != 0)
                {
                    nxType = "t";
                }

                if (axialSign == 0)
                {
                    nxType = " ";
                }

                for (i = 1; i < 6; i++)
                {
                    if (Math.Abs(Q[n, i]) < 0.0001)
                    {
                        temp[i - 1] = 0.0;
                    }
                    else
                    {
                        temp[i - 1] = Q[n, i];
                    }
                }
                frameElementEndForces.Add(
                    new FrameElementEndForce(n, J1[n], nx, nxType, temp[0], temp[1], temp[2], temp[3], temp[4]));

                //Do not copy above code to the following.
                if (Math.Abs(Q[n, 6]) < 0.0001)
                {
                    nx = 0.0;
                }
                else
                {
                    nx = Q[n, 6];
                }

                if (Q[n, 6] >= 0.0001 && axialSign != 0)
                {
                    nxType = "t";
                }

                if (Q[n, 6] <= -0.0001 && axialSign != 0)
                {
                    nxType = "c";
                }

                if (axialSign == 0)
                {
                    nxType = " ";
                }

                for (i = 7; i < 12; i++)
                {
                    if (Math.Abs(Q[n, i]) < 0.0001)
                    {
                        temp[i - 7] = 0.0;
                    }
                    else
                    {
                        temp[i - 7] = Q[n, i];
                    }
                }
                frameElementEndForces.Add(
                    new FrameElementEndForce(n, J2[n], nx, nxType, temp[0], temp[1], temp[2], temp[3], temp[4]));
            }

            List<ReactionOutput> reactionOutputs = new List<ReactionOutput>();
            //Reactions
            for (j = 0; j < nN; j++)
            {
                if (!Common.isDoubleZero(r[6 * j + 0]) || !Common.isDoubleZero(r[6 * j + 1]) || !Common.isDoubleZero(r[6 * j + 2]) ||
                    !Common.isDoubleZero(r[6 * j + 3]) || !Common.isDoubleZero(r[6 * j + 4]) || !Common.isDoubleZero(r[6 * j + 5]))
                {
                    double[] temp = new double[6];
                    for (i = 0; i < 6; i++)
                    {
                        if (!Common.isDoubleZero(r[6 * j + i]))
                        {
                            temp[i] = R[6 * j + i];
                        }
                        else
                        {
                            temp[i] = 0.0;
                        }
                    }
                    reactionOutputs.Add(new ReactionOutput(j, new Vec3(temp[0], temp[1], temp[2]), new Vec3(temp[3], temp[4], temp[5])));
                }
            }

            return (nodeDisplacements, frameElementEndForces, reactionOutputs);
        }


        public static List<PeakFrameElementInternalForce> GetInternalForces(int lc, int nl, string title, float dx, List<Vec3Float> xyz,
            double[,] Q, int nN, int nE, List<double> L, List<int> J1, List<int> J2, List<float> Ax, List<float> Asy, List<float> Asz,
            List<float> Jx, List<float> Iy, List<float> Iz, List<float> E, List<float> G, List<float> p,
            List<float> d, float gX, float gY, float gZ, int nU, float[,] U, int nW, float[,] W, int nP, float[,] P,
            double[] D, bool shear, double error)
        {
            double u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12; /* displ. */

            double xx1, xx2, wx1, wx2,  /* trapz load data, local x dir */
                xy1, xy2, wy1, wy2, /* trapz load data, local y dir */
                xz1, xz2, wz1, wz2; /* trapz load data, local z dir */

            double wx = 0, wy = 0, wz = 0, // distributed loads in local coords at x[i] 
                wx_ = 0, wy_ = 0, wz_ = 0,// distributed loads in local coords at x[i-1]
                wxg = 0, wyg = 0, wzg = 0,// gravity loads in local x, y, z coord's
                tx = 0.0, tx_ = 0.0;  // distributed torque about local x coord 

            double xp;      /* location of internal point loads	*/

            double dx_, dxnx;   /* distance along frame element		*/





            double maxNx, maxVy, maxVz,     /*  maximum internal forces	*/
                maxTx, maxMy, maxMz,    /*  maximum internal moments	*/
                maxDx, maxDy, maxDz,    /*  maximum element displacements */
                maxRx, maxSy, maxSz;    /*  maximum element rotations	*/

            double minNx, minVy, minVz,     /*  minimum internal forces	*/
                minTx, minMy, minMz,    /*  minimum internal moments	*/
                minDx, minDy, minDz,    /*  minimum element displacements */
                minRx, minSy, minSz;    /*  minimum element rotations	*/

            int n, m,       /* frame element number			*/
                cU = 0, cW = 0, cP = 0, /* counters for U, W, and P loads	*/
                i, nx,      /* number of sections alont x axis	*/
                n1, n2, i1, i2; /* starting and stopping node no's	*/

            if (dx == -1.0)
                return null;	// skip calculation of internal forces and displ

            List<PeakFrameElementInternalForce> peakFrameElementInternalForces = new List<PeakFrameElementInternalForce>();
            for (m = 0; m < nE; m++) //m is used as index of 1 based arrays, so decrease it by 1
            {
                n1 = J1[m];
                n2 = J2[m];
                nx = (int)Math.Floor(L[m] / dx);
                if (nx < 1)
                    nx = 1;

                double[] x = new double[nx + 1]; /* distance along frame element		*/
                double[] Nx = new double[nx + 1]; /* axial force within frame el.		*/
                double[] Vy = new double[nx + 1];
                double[] Vz = new double[nx + 1]; /* shear forces within frame el.	    */
                double[] Tx = new double[nx + 1]; /* torsional moment within frame el.	*/
                double[] My = new double[nx + 1];
                double[] Mz = new double[nx + 1]; /* bending moments within frame el.	*/
                double[] Sy = new double[nx + 1];
                double[] Sz = new double[nx + 1]; /* transverse slopes of frame el.	    */
                double[] Dx = new double[nx + 1];
                double[] Dy = new double[nx + 1];
                double[] Dz = new double[nx + 1]; /* frame el. displ. in local x,y,z, dir's */
                double[] Rx = new double[nx + 1]; /* twist rotation about the local x-axis */




                for (i = 0; i < nx; i++) //i is used as index of 0 based arrays here. Remain unchanged.
                    x[i] = i * dx;
                x[nx] = L[m];
                dxnx = x[nx] - x[nx - 1];

                double[] t = Coordtrans.coordTrans(xyz, L[m], n1, n2, p[m]);

                wxg = d[m] * Ax[m] * (t[0] * gX + t[1] * gY + t[2] * gZ);
                wyg = d[m] * Ax[m] * (t[3] * gX + t[4] * gY + t[5] * gZ);
                wzg = d[m] * Ax[m] * (t[6] * gX + t[7] * gY + t[8] * gZ);

                for (n = 0; n < nE && cU < nU; n++) //n is used as index of 1 based arrays, so decrease it by 1
                {
                    if ((int)U[n, 0] == m)
                    {
                        wxg += U[n, 1];
                        wyg += U[n, 2];
                        wzg += U[n, 3];
                        ++cU;
                    }
                }

                Nx[0] = -Q[m, 0]; // positive Nx is tensile
                Vy[0] = -Q[m, 1]; // positive Vy in local y direction
                Vz[0] = -Q[m, 2]; // positive Vz in local z direction
                Tx[0] = -Q[m, 3]; // positive Tx r.h.r. about local x axis
                My[0] = Q[m, 4]; // positive My -> positive x-z curvature
                Mz[0] = -Q[m, 5]; // positive Mz -> positive x-y curvature

                dx_ = dx;

                for (i = 1; i <= nx; i++) //i is used as index of 0 based arrays here. Remain unchanged.
                {
                    wx = wxg;
                    wy = wyg;
                    wz = wzg;

                    if (i == 1)
                    {
                        wx_ = wxg;
                        wy_ = wyg;
                        wz_ = wzg;
                        tx_ = tx;
                    }

                    for (n = 0; n < 10 * nE && cW < nW; n++) //n is used as index of 1 based arrays here, so decrease it by 1
                    {
                        //here m is used as a value not a index, it should need to add 1 back,
                        //but the W[n, 0] is actually storing 0 based node index, so finally, it should remain unchanged.
                        if ((int)W[n, 0] == m)
                        {
                            if (i == nx)
                                ++cW;
                            xx1 = W[n, 1];
                            xx2 = W[n, 2];
                            wx1 = W[n, 3];
                            wx2 = W[n, 4];
                            xy1 = W[n, 5];
                            xy2 = W[n, 6];
                            wy1 = W[n, 7];
                            wy2 = W[n, 8];
                            xz1 = W[n, 9];
                            xz2 = W[n, 10];
                            wz1 = W[n, 11];
                            wz2 = W[n, 12];

                            if (x[i] > xx1 && x[i] <= xx2)
                                wx += wx1 + (wx2 - wx1) * (x[i] - xx1) / (xx2 - xx1);
                            if (x[i] > xy1 && x[i] <= xy2)
                                wy += wy1 + (wy2 - wy1) * (x[i] - xy1) / (xy2 - xy1);
                            if (x[i] > xz1 && x[i] <= xz2)
                                wz += wz1 + (wz2 - wz1) * (x[i] - xz1) / (xz2 - xz1);
                        }
                    }

                    if (i == nx)
                        dx_ = dxnx;

                    Nx[i] = Nx[i - 1] - 0.5 * (wx + wx_) * dx_;
                    Vy[i] = Vy[i - 1] - 0.5 * (wy + wy_) * dx_;
                    Vz[i] = Vz[i - 1] - 0.5 * (wz + wz_) * dx_;
                    Tx[i] = Tx[i - 1] - 0.5 * (tx + tx_) * dx_;

                    wx_ = wx;
                    wy_ = wy;
                    wz_ = wz;
                    tx_ = tx;

                    for (n = 0; n < 10 * nE && cP < nP; n++)
                    {
                        if ((int)P[n, 0] == m + 1)//here m is used as a value not a index, need to add 1 back
                        {
                            if (i == nx)
                                ++cP;
                            xp = P[n, 4];
                            if (x[i] <= xp && xp < x[i] + dx)
                            {
                                Nx[i] -= P[n, 1] * 0.5 * (1.0 - (xp - x[i]) / dx);
                                Vy[i] -= P[n, 2] * 0.5 * (1.0 - (xp - x[i]) / dx);
                                Vz[i] -= P[n, 3] * 0.5 * (1.0 - (xp - x[i]) / dx);

                            }
                            if (x[i] - dx <= xp && xp < x[i])
                            {
                                Nx[i] -= P[n, 1] * 0.5 * (1.0 - (x[i] - dx - xp) / dx);
                                Vy[i] -= P[n, 2] * 0.5 * (1.0 - (x[i] - dx - xp) / dx);
                                Vz[i] -= P[n, 3] * 0.5 * (1.0 - (x[i] - dx - xp) / dx);
                            }
                        }
                    }
                }

                // linear correction of forces for bias in trapezoidal integration
                for (i = 1; i <= nx; i++)
                {
                    Nx[i] -= (Nx[nx] - Q[m, 6]) * i / nx;
                    Vy[i] -= (Vy[nx] - Q[m, 7]) * i / nx;
                    Vz[i] -= (Vz[nx] - Q[m, 8]) * i / nx;
                    Tx[i] -= (Tx[nx] - Q[m, 9]) * i / nx;
                }
                // trapezoidal integration of shear force for bending momemnt
                dx_ = dx;
                for (i = 1; i <= nx; i++)
                {
                    if (i == nx) dx_ = dxnx;
                    My[i] = My[i - 1] - 0.5 * (Vz[i] + Vz[i - 1]) * dx_;
                    Mz[i] = Mz[i - 1] - 0.5 * (Vy[i] + Vy[i - 1]) * dx_;

                }
                // linear correction of moments for bias in trapezoidal integration
                for (i = 1; i <= nx; i++)
                {
                    My[i] -= (My[nx] + Q[m, 10]) * i / nx;
                    Mz[i] -= (Mz[nx] - Q[m, 11]) * i / nx;
                }

                // find interior transverse displacements 
                i1 = 6 * n1;
                i2 = 6 * n2;

                /* compute end deflections in local coordinates */

                u1 = t[0] * D[i1 + 0] + t[1] * D[i1 + 1] + t[2] * D[i1 + 2];
                u2 = t[3] * D[i1 + 0] + t[4] * D[i1 + 1] + t[5] * D[i1 + 2];
                u3 = t[6] * D[i1 + 0] + t[7] * D[i1 + 1] + t[8] * D[i1 + 2];

                u4 = t[0] * D[i1 + 3] + t[1] * D[i1 + 4] + t[2] * D[i1 + 5];
                u5 = t[3] * D[i1 + 3] + t[4] * D[i1 + 4] + t[5] * D[i1 + 5];
                u6 = t[6] * D[i1 + 3] + t[7] * D[i1 + 4] + t[8] * D[i1 + 5];

                u7 = t[0] * D[i2 + 0] + t[1] * D[i2 + 1] + t[2] * D[i2 + 2];
                u8 = t[3] * D[i2 + 0] + t[4] * D[i2 + 1] + t[5] * D[i2 + 2];
                u9 = t[6] * D[i2 + 0] + t[7] * D[i2 + 1] + t[8] * D[i2 + 2];

                u10 = t[0] * D[i2 + 3] + t[1] * D[i2 + 4] + t[2] * D[i2 + 5];
                u11 = t[3] * D[i2 + 3] + t[4] * D[i2 + 4] + t[5] * D[i2 + 5];
                u12 = t[6] * D[i2 + 3] + t[7] * D[i2 + 4] + t[8] * D[i2 + 5];


                // rotations and displacements for frame element "m" at (x=0)
                Dx[0] = u1; // displacement in  local x dir  at node N1
                Dy[0] = u2; // displacement in  local y dir  at node N1
                Dz[0] = u3; // displacement in  local z dir  at node N1
                Rx[0] = u4; // rotationin about local x axis at node N1
                Sy[0] = u6; // slope in  local y  direction  at node N1
                Sz[0] = -u5;    // slope in  local z  direction  at node N1

                // axial displacement along frame element "m"
                dx_ = dx;
                for (i = 1; i <= nx; i++)
                {
                    if (i == nx) dx_ = dxnx;
                    Dx[i] = Dx[i - 1] + 0.5 * (Nx[i - 1] + Nx[i]) / (E[m] * Ax[m]) * dx_;
                }
                // linear correction of axial displacement for bias in trapezoidal integration
                for (i = 1; i <= nx; i++)
                {
                    Dx[i] -= (Dx[nx] - u7) * i / nx;
                }

                // torsional rotation along frame element "m"
                dx_ = dx;
                for (i = 1; i <= nx; i++)
                {
                    if (i == nx) dx_ = dxnx;
                    Rx[i] = Rx[i - 1] + 0.5 * (Tx[i - 1] + Tx[i]) / (G[m] * Jx[m]) * dx_;
                }
                // linear correction of torsional rot'n for bias in trapezoidal integration
                for (i = 1; i <= nx; i++)
                {
                    Rx[i] -= (Rx[nx] - u10) * i / nx;
                }

                // transverse slope along frame element "m"
                dx_ = dx;
                for (i = 1; i <= nx; i++)
                {
                    if (i == nx) dx_ = dxnx;
                    Sy[i] = Sy[i - 1] + 0.5 * (Mz[i - 1] + Mz[i]) / (E[m] * Iz[m]) * dx_;
                    Sz[i] = Sz[i - 1] + 0.5 * (My[i - 1] + My[i]) / (E[m] * Iy[m]) * dx_;
                }
                // linear correction for bias in trapezoidal integration
                for (i = 1; i <= nx; i++)
                {
                    Sy[i] -= (Sy[nx] - u12) * i / nx;
                    Sz[i] -= (Sz[nx] + u11) * i / nx;
                }
                if (shear)
                {       // add-in slope due to shear deformation
                    for (i = 0; i <= nx; i++)
                    {
                        Sy[i] += Vy[i] / (G[m] * Asy[m]);
                        Sz[i] += Vz[i] / (G[m] * Asz[m]);
                    }
                }
                // displacement along frame element "m"
                dx_ = dx;
                for (i = 1; i <= nx; i++)
                {
                    if (i == nx) dx_ = dxnx;
                    Dy[i] = Dy[i - 1] + 0.5 * (Sy[i - 1] + Sy[i]) * dx_;
                    Dz[i] = Dz[i - 1] + 0.5 * (Sz[i - 1] + Sz[i]) * dx_;
                }
                // linear correction for bias in trapezoidal integration
                for (i = 1; i <= nx; i++)
                {
                    Dy[i] -= (Dy[nx] - u8) * i / nx;
                    Dz[i] -= (Dz[nx] - u9) * i / nx;
                }

                // initialize the maximum and minimum element forces and displacements 
                maxNx = minNx = Nx[0]; maxVy = minVy = Vy[0]; maxVz = minVz = Vz[0];    //  maximum internal forces
                maxTx = minTx = Tx[0]; maxMy = minMy = My[0]; maxMz = minMz = Mz[0];    //  maximum internal moments
                maxDx = minDx = Dx[0]; maxDy = minDy = Dy[0]; maxDz = minDz = Dz[0];    //  maximum element displacements
                maxRx = minRx = Rx[0]; maxSy = minSy = Sy[0]; maxSz = minSz = Sz[0];    //  maximum element rotations

                // find maximum and minimum internal element forces
                for (i = 1; i <= nx; i++)
                {
                    maxNx = (Nx[i] > maxNx) ? Nx[i] : maxNx;//i can't be 0, but max and min are default to be value of index 0.
                    minNx = (Nx[i] < minNx) ? Nx[i] : minNx;
                    maxVy = (Vy[i] > maxVy) ? Vy[i] : maxVy;
                    minVy = (Vy[i] < minVy) ? Vy[i] : minVy;
                    maxVz = (Vz[i] > maxVz) ? Vz[i] : maxVz;
                    minVz = (Vz[i] < minVz) ? Vz[i] : minVz;

                    maxTx = (Tx[i] > maxTx) ? Tx[i] : maxTx;
                    minTx = (Tx[i] < minTx) ? Tx[i] : minTx;
                    maxMy = (My[i] > maxMy) ? My[i] : maxMy;
                    minMy = (My[i] < minMy) ? My[i] : minMy;
                    maxMz = (Mz[i] > maxMz) ? Mz[i] : maxMz;
                    minMz = (Mz[i] < minMz) ? Mz[i] : minMz;
                }

                // find maximum and minimum internal element displacements
                for (i = 1; i <= nx; i++)
                {
                    maxDx = (Dx[i] > maxDx) ? Dx[i] : maxDx;
                    minDx = (Dx[i] < minDx) ? Dx[i] : minDx;
                    maxDy = (Dy[i] > maxDy) ? Dy[i] : maxDy;
                    minDy = (Dy[i] < minDy) ? Dy[i] : minDy;
                    maxDz = (Dz[i] > maxDz) ? Dz[i] : maxDz;
                    minDz = (Dz[i] < minDz) ? Dz[i] : minDz;
                    maxRx = (Rx[i] > maxRx) ? Rx[i] : maxRx;
                    minRx = (Rx[i] < minRx) ? Rx[i] : minRx;
                    maxSy = (Sy[i] > maxSy) ? Sy[i] : maxSy;
                    minSy = (Sy[i] < minSy) ? Sy[i] : minSy;
                    maxSz = (Sz[i] > maxSz) ? Sz[i] : maxSz;
                    minSz = (Sz[i] < minSz) ? Sz[i] : minSz;
                }
                peakFrameElementInternalForces.Add(new PeakFrameElementInternalForce(m, false, maxNx, maxVy, maxVz, maxTx, maxMy, maxMz));
                peakFrameElementInternalForces.Add(new PeakFrameElementInternalForce(m, true, minNx, minVy, minVz, minTx, minMy, minMz));
            }

            return peakFrameElementInternalForces;
        }

        //Simple output for testing only.
        public static void ExportOutput(Output output, string outputPath)
        {
            File.WriteAllText(outputPath, "");
            var nL = output.LoadCaseOutputs.Count;
            for (int i = 0; i < nL; i++)
            {
                LoadCaseOutput loadCaseOutput = output.LoadCaseOutputs[i];
                var csv = new StringBuilder();
                string newLine;
                csv.AppendLine("------------------Load case " + (i + 1) + "------------------");

                csv.AppendLine("* Node displacements");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                    "Node", "X-dsp", "Y-dsp", "z-dsp", "X-rot", "Y-rot", "Z-rot");
                csv.AppendLine(newLine);
                foreach (var nodeDisplacement in loadCaseOutput.NodeDisplacements)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                        nodeDisplacement.NodeIdx + 1, nodeDisplacement.Displacement.X, nodeDisplacement.Displacement.Y, nodeDisplacement.Displacement.Z,
                        nodeDisplacement.Rotation.X, nodeDisplacement.Rotation.Y, nodeDisplacement.Rotation.Z);
                    csv.AppendLine(newLine);
                }

                csv.AppendLine("* Frame Element End Force");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                    "Elmnt", "Node", "Nx", "Vy", "Vz", "Txx", "Myy", "Mzz");
                csv.AppendLine(newLine);
                foreach (var frameElementEndForce in loadCaseOutput.FrameElementEndForces)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                        frameElementEndForce.ElementIdx + 1, frameElementEndForce.NodeIdx + 1, frameElementEndForce.Nx + frameElementEndForce.NxType,
                        frameElementEndForce.Vy, frameElementEndForce.Vz, frameElementEndForce.Txx, frameElementEndForce.Myy, frameElementEndForce.Mzz);
                    csv.AppendLine(newLine);
                }

                csv.AppendLine("* Reactions");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                    "Node", "Fx", "Fy", "Fz", "Mxx", "Myy", "Mzz");
                csv.AppendLine(newLine);
                foreach (var reactionOutput in loadCaseOutput.ReactionOutputs)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                        reactionOutput.NodeIdx + 1, reactionOutput.F.X, reactionOutput.F.Y, reactionOutput.F.Z,
                        reactionOutput.M.X, reactionOutput.M.Y, reactionOutput.M.Z);
                    csv.AppendLine(newLine);
                }
                csv.AppendLine("RMS Relative Equilibrium Error: " + loadCaseOutput.RmsRelativeEquilibriumError);

                csv.AppendLine("* Peak Frame Element Internal Forces");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                    "Elmnt", ".", "Nx", "Vy", "Vz", "Txx", "Myy", "Mzz");
                csv.AppendLine(newLine);
                foreach (var peakFrameElementInternalForce in loadCaseOutput.PeakFrameElementInternalForces)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                        peakFrameElementInternalForce.ElementIdx + 1, peakFrameElementInternalForce.IsMin ? "min" : "max",
                        peakFrameElementInternalForce.Nx, peakFrameElementInternalForce.Vy, peakFrameElementInternalForce.Vz,
                        peakFrameElementInternalForce.Txx, peakFrameElementInternalForce.Myy, peakFrameElementInternalForce.Mzz);
                    csv.AppendLine(newLine);
                }
                File.AppendAllText(outputPath, csv.ToString());

            }
        }
    }
}
