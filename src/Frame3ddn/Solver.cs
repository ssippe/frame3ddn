﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Frame3ddn
{
    public class Solver
    {
        public Output Solve(Input input)
        {
            IReadOnlyList<Node> nodes = input.Nodes;
            IReadOnlyList<FrameElement> frameElements = input.FrameElements;
            int nN = input.Nodes.Count;
            List<double> rj = input.Nodes.Select(n => n.Radius).ToList();
            List<Vec3> xyz = input.Nodes.Select(n => n.Position).ToList(); //V
            int Dof = 6 * nN;
            int nR = input.ReactionInputs.Count;
            double[] q = new double[Dof];
            double[] r = new double[Dof];
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

            for (int i = 0; i < Dof; i++)
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
            List<double> Ax = input.FrameElements.Select(f => f.Ax).ToList(); //V
            List<double> Asy = input.FrameElements.Select(f => f.Asy).ToList();
            List<double> Asz = input.FrameElements.Select(f => f.Asz).ToList();
            List<double> Iy = input.FrameElements.Select(f => f.Iy).ToList();
            List<double> Iz = input.FrameElements.Select(f => f.Iz).ToList();
            List<double> E = input.FrameElements.Select(f => f.E).ToList();
            List<double> G = input.FrameElements.Select(f => f.G).ToList();
            List<double> p = input.FrameElements.Select(f => f.Roll).ToList(); //V
            List<double> d = input.FrameElements.Select(f => f.Density).ToList();
            int nL = input.LoadCases.Count; //V
            List<double> gX = input.LoadCases.Select(l => l.Gravity.X).ToList(); //V
            List<double> gY = input.LoadCases.Select(l => l.Gravity.Y).ToList(); //V
            List<double> gZ = input.LoadCases.Select(l => l.Gravity.Z).ToList(); //V
            List<int> nF = input.LoadCases.Select(l => l.NodeLoads.Count).ToList();
            List<int> nU = input.LoadCases.Select(l => l.UniformLoads.Count).ToList();
            List<int> nW = input.LoadCases.Select(l => l.TrapLoads.Count).ToList();
            double[,,] U = new double[nL, nE, 4];
            double[,,] W = new double[nL, nE*10, 13];
            bool shear = input.IncludeShearDeformation;

            IReadOnlyList<LoadCase> loadCases = input.LoadCases;

            AssembleLoads(nN, nE, nL, Dof, xyz, L, Le, N1, N2, Ax, Asy, Asz, Iy, Iz, E, G, p, d, gX, gY, gZ, shear, nF, nU, nW,
                U, W, loadCases);
            return null;
        }

        private void AssembleLoads(int nN, int nE, int nL, int Dof, List<Vec3> xyz, List<double> L, List<double> Le,
            List<int> N1, List<int> N2,
            List<double> Ax, List<double> Asy, List<double> Asz, List<double> Iy, List<double> Iz, List<double> E,
            List<double> G, List<double> p,
            List<double> d, List<double> gX, List<double> gY, List<double> gZ, bool shear, List<int> nF, List<int> nU, List<int> nW,
            double[,,] U, double[,,] W, IReadOnlyList<LoadCase> loadCases)
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
            double[,,] eqFMech = new double[nL, nE, 12];
            double[,] FMech = new double[nL, Dof];
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

                    U[lc, i, 0] = (double) n;
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
                if (nW[lc] < 0 || nW[lc] > 10 * nE)
                    Console.WriteLine("\n  error: valid ranges for nW is 0 ... %d \n", 10 * nE);

                for (int i = 0; i < nW[lc]; i++)
                {
                    TrapLoad trapLoad = loadCases[lc].TrapLoads[i];
                    int n = trapLoad.ElementIdx;
                    if (n < 0 || n > nE)
                        Console.WriteLine(
                            "\n  error in trapezoidally-distributed loads: element number %d is out of range\n", n);
                    W[lc, i, 0] = (double) n;
                    W[lc, i, 1] = trapLoad.LocationStart.X;
                    W[lc, i, 2] = trapLoad.LocationStart.Y;
                    W[lc, i, 3] = trapLoad.LocationStart.Z;
                    W[lc, i, 4] = trapLoad.LocationEnd.X;
                    W[lc, i, 5] = trapLoad.LocationEnd.Y;
                    W[lc, i, 6] = trapLoad.LocationEnd.Z;
                    W[lc, i, 7] = trapLoad.LoadStart.X;
                    W[lc, i, 8] = trapLoad.LoadStart.Y;
                    W[lc, i, 9] = trapLoad.LoadStart.Z;
                    W[lc, i, 10] = trapLoad.LoadEnd.X;
                    W[lc, i, 11] = trapLoad.LoadEnd.Y;
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
                    eqFMech[lc, n, 1] += (Nx1 * t[0] + Vy1 * t[3] + Vz1 * t[6]);
                    eqFMech[lc, n, 2] += (Nx1 * t[1] + Vy1 * t[4] + Vz1 * t[7]);
                    eqFMech[lc, n, 3] += (Nx1 * t[2] + Vy1 * t[5] + Vz1 * t[8]);
                    eqFMech[lc, n, 4] += (Mx1 * t[0] + My1 * t[3] + Mz1 * t[6]);
                    eqFMech[lc, n, 5] += (Mx1 * t[1] + My1 * t[4] + Mz1 * t[7]);
                    eqFMech[lc, n, 6] += (Mx1 * t[2] + My1 * t[5] + Mz1 * t[8]);

                    eqFMech[lc, n, 7] += (Nx2 * t[0] + Vy2 * t[3] + Vz2 * t[6]);
                    eqFMech[lc, n, 8] += (Nx2 * t[1] + Vy2 * t[4] + Vz2 * t[7]);
                    eqFMech[lc, n, 9] += (Nx2 * t[2] + Vy2 * t[5] + Vz2 * t[8]);
                    eqFMech[lc, n, 10] += (Mx2 * t[0] + My2 * t[3] + Mz2 * t[6]);
                    eqFMech[lc, n, 11] += (Mx2 * t[1] + My2 * t[4] + Mz2 * t[7]);
                    eqFMech[lc, n, 12] += (Mx2 * t[2] + My2 * t[5] + Mz2 * t[8]);
                }


            }
        }


        // ref main.c:265
        static int Dof(Input input) => input.Nodes.Count * 6;

        /// <summary>
        /// global stiffness matrix
        /// </summary>
        // ref main.c:343
        static double[,] K(Input input)
        {
            var dof = Dof(input);
            var k = new double[dof, dof];
            return k;
        }

    }
}
