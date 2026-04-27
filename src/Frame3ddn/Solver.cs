using Frame3ddn.Model;
using Frame3ddn.Writers;
using System;
using System.Collections.Generic;
using System.Linq;

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
            List<Vec3> xyz = input.Nodes.Select(n => n.Position).ToList();
            int DoF = 6 * nN;
            int nR = input.ReactionInputs.Count;
            double[] q = new double[DoF];
            float[] r = new float[DoF];
            for (int i = 0; i < nR; i++)
            {
                int j = input.ReactionInputs[i].NodeIdx;  //This index number is decreased by 1 when importing
                r[j * 6 + 0] = input.ReactionInputs[i].Force.X;
                r[j * 6 + 1] = input.ReactionInputs[i].Force.Y;
                r[j * 6 + 2] = input.ReactionInputs[i].Force.Z;
                r[j * 6 + 3] = input.ReactionInputs[i].Moment.X;
                r[j * 6 + 4] = input.ReactionInputs[i].Moment.Y;
                r[j * 6 + 5] = input.ReactionInputs[i].Moment.Z;
            }

            for (int i = 0; i < DoF; i++)
            {
                q[i] = (r[i] == (double)1) ? 0 : 1;
            }

            int nE = input.FrameElements.Count;

            List<double> L = input.FrameElements.Select(f =>
                        CoordinateTransform.CalculateSQDistance(input.Nodes[f.NodeIdx1].Position, input.Nodes[f.NodeIdx2].Position))
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
            for (int i = 0; i < nL; i++)
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
            float[,,] W = new float[nL, nE * 10, 13];
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

            int iter = 0;
            double tol = 1.0e-9;

            // Per-load-case internal concentrated point loads. Stored in P[lc, i, 0..4] =
            // (element_idx, Px, Py, Pz, x_along_element) for use by GetInternalForces. The
            // equivalent nodal forces are computed below and scattered into eqFMech/FMech.
            int[] nP = input.LoadCases.Select(l => l.InternalConcentratedLoads.Count).ToArray();
            float[,,] P = new float[nL, Math.Max(1, 10 * nE), 5];

            // Per-load-case thermal load count + per-element equivalent thermal nodal forces.
            int[] nT = input.LoadCases.Select(l => l.TemperatureLoads.Count).ToArray();
            double[,] FTemp = new double[nL, DoF];
            double[,,] eqFTemp = new double[nL, nE, 12];
            for (int lcIdx = 0; lcIdx < nL; lcIdx++)
            {
                foreach (TemperatureLoad tl in input.LoadCases[lcIdx].TemperatureLoads)
                {
                    int n = tl.ElementIdx;
                    if (n < 0 || n >= nE)
                        throw new Exception($"Temperature load references unknown element {n + 1} (load case {lcIdx + 1}).");
                    double a = tl.Alpha;
                    double hy = tl.Hy;
                    double hz = tl.Hz;
                    if (hy <= 0 || hz <= 0)
                        throw new Exception($"Temperature load element {n + 1} (LC {lcIdx + 1}) requires positive hy and hz.");

                    // Local end forces from the thermal expansion / gradient (frame3dd_io.c:1381–1388).
                    double Nx2 = a * 0.25 * (tl.Typ + tl.Tym + tl.Tzp + tl.Tzm) * E[n] * Ax[n];
                    double Nx1 = -Nx2;
                    double My1 = (a / hz) * (tl.Tzm - tl.Tzp) * E[n] * Iy[n];
                    double My2 = -My1;
                    double Mz1 = (a / hy) * (tl.Typ - tl.Tym) * E[n] * Iz[n];
                    double Mz2 = -Mz1;

                    // Globalize via the same coord transform used for elastic / geometric K.
                    double[] t = CoordinateTransform.CoordTrans(xyz, L[n], N1[n], N2[n], p[n]);
                    eqFTemp[lcIdx, n, 0] += Nx1 * t[0];
                    eqFTemp[lcIdx, n, 1] += Nx1 * t[1];
                    eqFTemp[lcIdx, n, 2] += Nx1 * t[2];
                    eqFTemp[lcIdx, n, 3] += My1 * t[3] + Mz1 * t[6];
                    eqFTemp[lcIdx, n, 4] += My1 * t[4] + Mz1 * t[7];
                    eqFTemp[lcIdx, n, 5] += My1 * t[5] + Mz1 * t[8];
                    eqFTemp[lcIdx, n, 6] += Nx2 * t[0];
                    eqFTemp[lcIdx, n, 7] += Nx2 * t[1];
                    eqFTemp[lcIdx, n, 8] += Nx2 * t[2];
                    eqFTemp[lcIdx, n, 9] += My2 * t[3] + Mz2 * t[6];
                    eqFTemp[lcIdx, n, 10] += My2 * t[4] + Mz2 * t[7];
                    eqFTemp[lcIdx, n, 11] += My2 * t[5] + Mz2 * t[8];
                }

                // Scatter element-equivalent thermal forces into the global F_temp vector.
                for (int n = 0; n < nE; n++)
                {
                    int n1 = N1[n], n2 = N2[n];
                    for (int i = 0; i < 6; i++) FTemp[lcIdx, 6 * n1 + i] += eqFTemp[lcIdx, n, i];
                    for (int i = 6; i < 12; i++) FTemp[lcIdx, 6 * n2 - 6 + i] += eqFTemp[lcIdx, n, i];
                }
            }

            int[] nD = input.LoadCases.Select(l => l.PrescribedDisplacements.Count).ToArray();
            double[,] Dp = new double[nL, DoF];
            for (int lc = 0; lc < nL; lc++)
            {
                foreach (PrescribedDisplacement pd in input.LoadCases[lc].PrescribedDisplacements)
                {
                    int j = pd.NodeIdx;
                    double[] values =
                    {
                        pd.Displacement.X, pd.Displacement.Y, pd.Displacement.Z,
                        pd.Rotation.X,     pd.Rotation.Y,     pd.Rotation.Z
                    };
                    for (int k = 0; k < 6; k++)
                    {
                        int dof = j * 6 + k;
                        if (Common.IsDoubleZero(r[dof]) && values[k] != 0.0)
                        {
                            throw new Exception(
                                $"Initial displacements can be prescribed only at restrained coordinates. Load case {lc + 1}, node {j + 1}, dof {k + 1}.");
                        }
                        Dp[lc, dof] = values[k];
                    }
                }
            }

            IReadOnlyList<LoadCase> loadCases = input.LoadCases;

            Frame3ddIO.AssembleLoads(nN, nE, nL, DoF, xyz, L, Le, N1, N2, Ax, Asy, Asz, Iy, Iz, E, G, p, d, gX, gY, gZ, shear, nF, nU, nW, FMech, eqFMech,
                U, W, loadCases);

            // Add internal concentrated point loads to eqFMech / FMech (frame3dd_io.c:1268-1340).
            for (int lcIdx = 0; lcIdx < nL; lcIdx++)
            {
                int pIdx = 0;
                foreach (InternalConcentratedLoad icl in input.LoadCases[lcIdx].InternalConcentratedLoads)
                {
                    int n = icl.ElementIdx;
                    if (n < 0 || n >= nE)
                        throw new Exception($"Internal concentrated load element {n + 1} out of range (LC {lcIdx + 1}).");
                    double Px = icl.Load.X, Py = icl.Load.Y, Pz = icl.Load.Z;
                    double a = icl.X;
                    double Ln = L[n];
                    double b = Ln - a;
                    if (a < 0 || a > Ln || b < 0)
                        throw new Exception($"Internal concentrated load location {a} out of range [0, {Ln}] (LC {lcIdx + 1}, element {n + 1}).");

                    double Ksy, Ksz;
                    if (shear)
                    {
                        Ksy = 12.0 * E[n] * Iz[n] / (G[n] * Asy[n] * Le[n] * Le[n]);
                        Ksz = 12.0 * E[n] * Iy[n] / (G[n] * Asz[n] * Le[n] * Le[n]);
                    }
                    else
                    {
                        Ksy = Ksz = 0.0;
                    }

                    // GetInternalForces expects the element id at P[n, 0] to be 1-based
                    // (mirrors upstream's `(int)P[n][1] == m+1` check); store n+1.
                    P[lcIdx, pIdx, 0] = n + 1;
                    P[lcIdx, pIdx, 1] = (float)Px;
                    P[lcIdx, pIdx, 2] = (float)Py;
                    P[lcIdx, pIdx, 3] = (float)Pz;
                    P[lcIdx, pIdx, 4] = (float)a;
                    pIdx++;

                    // Local end forces from the upstream point-load distribution formulas.
                    double Nx1 = Px * a / Ln;
                    double Nx2 = Px * b / Ln;
                    double Vy1 = (1.0 / (1.0 + Ksz)) * Py * b * b * (3.0 * a + b) / (Ln * Ln * Ln) + (Ksz / (1.0 + Ksz)) * Py * b / Ln;
                    double Vy2 = (1.0 / (1.0 + Ksz)) * Py * a * a * (3.0 * b + a) / (Ln * Ln * Ln) + (Ksz / (1.0 + Ksz)) * Py * a / Ln;
                    double Vz1 = (1.0 / (1.0 + Ksy)) * Pz * b * b * (3.0 * a + b) / (Ln * Ln * Ln) + (Ksy / (1.0 + Ksy)) * Pz * b / Ln;
                    double Vz2 = (1.0 / (1.0 + Ksy)) * Pz * a * a * (3.0 * b + a) / (Ln * Ln * Ln) + (Ksy / (1.0 + Ksy)) * Pz * a / Ln;
                    double Mx1 = 0.0, Mx2 = 0.0;
                    double My1 = -(1.0 / (1.0 + Ksy)) * Pz * a * b * b / (Ln * Ln) - (Ksy / (1.0 + Ksy)) * Pz * a * b / (2.0 * Ln);
                    double My2 = (1.0 / (1.0 + Ksy)) * Pz * a * a * b / (Ln * Ln) + (Ksy / (1.0 + Ksy)) * Pz * a * b / (2.0 * Ln);
                    double Mz1 = (1.0 / (1.0 + Ksz)) * Py * a * b * b / (Ln * Ln) + (Ksz / (1.0 + Ksz)) * Py * a * b / (2.0 * Ln);
                    double Mz2 = -(1.0 / (1.0 + Ksz)) * Py * a * a * b / (Ln * Ln) - (Ksz / (1.0 + Ksz)) * Py * a * b / (2.0 * Ln);

                    double[] tCoord = CoordinateTransform.CoordTrans(xyz, Ln, N1[n], N2[n], p[n]);
                    double[] dEq = new double[12];
                    dEq[0] = Nx1 * tCoord[0] + Vy1 * tCoord[3] + Vz1 * tCoord[6];
                    dEq[1] = Nx1 * tCoord[1] + Vy1 * tCoord[4] + Vz1 * tCoord[7];
                    dEq[2] = Nx1 * tCoord[2] + Vy1 * tCoord[5] + Vz1 * tCoord[8];
                    dEq[3] = Mx1 * tCoord[0] + My1 * tCoord[3] + Mz1 * tCoord[6];
                    dEq[4] = Mx1 * tCoord[1] + My1 * tCoord[4] + Mz1 * tCoord[7];
                    dEq[5] = Mx1 * tCoord[2] + My1 * tCoord[5] + Mz1 * tCoord[8];
                    dEq[6] = Nx2 * tCoord[0] + Vy2 * tCoord[3] + Vz2 * tCoord[6];
                    dEq[7] = Nx2 * tCoord[1] + Vy2 * tCoord[4] + Vz2 * tCoord[7];
                    dEq[8] = Nx2 * tCoord[2] + Vy2 * tCoord[5] + Vz2 * tCoord[8];
                    dEq[9] = Mx2 * tCoord[0] + My2 * tCoord[3] + Mz2 * tCoord[6];
                    dEq[10] = Mx2 * tCoord[1] + My2 * tCoord[4] + Mz2 * tCoord[7];
                    dEq[11] = Mx2 * tCoord[2] + My2 * tCoord[5] + Mz2 * tCoord[8];

                    for (int i = 0; i < 12; i++) eqFMech[lcIdx, n, i] += dEq[i];
                    int n1Idx = N1[n], n2Idx = N2[n];
                    for (int i = 0; i < 6; i++) FMech[lcIdx, 6 * n1Idx + i] += dEq[i];
                    for (int i = 6; i < 12; i++) FMech[lcIdx, 6 * n2Idx - 6 + i] += dEq[i];
                }
            }

            //ReadMassData()--Not implemented
            //ReadCondensationData()--Not implemented

            string outputText1 = OutWriter.InputDataToString(title, nN, nE, nL, nD, nR, nF, nU, nW, nP, nT,
                xyz, rj, N1, N2, Ax, Asy, Asz, Jx, Iy, Iz, E, G, p,
                d, gX, gY, gZ,
                FMech, Dp, r, U, W, P, shear, geom);

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

                // First pass: temperature loads only (solve K * D_t = F_t with no prescribed displ).
                if (nT[lc] > 0)
                {
                    double[] tempLoadTemp = Common.GetRow(FTemp, lc);
                    (int ok, double rmsResid) tResult = Frame3dd.SolveSystem(K, dD, tempLoadTemp, dR, DoF, q, r, ok, rmsResid);
                    ok = tResult.ok;
                    rmsResid = tResult.rmsResid;

                    for (int i = 0; i < DoF; i++) if (!Common.IsDoubleZero(q[i])) D[i] += dD[i];
                    for (int i = 0; i < DoF; i++) if (!Common.IsDoubleZero(r[i])) R[i] += dR[i];

                    if (geom)
                    {
                        // Thermal-induced Q feeds the geometric-stiffness contribution for the
                        // mechanical-load assembly that follows.
                        double[,] tempTArrayInit = Common.GetArray(eqFTemp, lc);
                        double[,] tempMArrayInit = Common.GetArray(eqFMech, lc);
                        Frame3dd.ElementEndForces(Q, nE, xyz, L, Le, N1, N2,
                            Ax, Asy, Asz, Jx, Iy, Iz, E, G, p,
                            tempTArrayInit, tempMArrayInit, D, shear, geom,
                            axialStrainWarning);
                        K = Frame3dd.AssembleK(DoF, nE, xyz, r, L, Le, N1, N2, Ax, Asy, Asz, Jx, Iy, Iz, E, G, p, shear, geom, Q);
                    }

                    // Reset the displacement increment before the mechanical-load solve below.
                    for (int i = 0; i < DoF; i++) dD[i] = 0.0;
                }

                if (nF[lc] > 0 || nU[lc] > 0 || nW[lc] > 0 || nP[lc] > 0 || nD[lc] > 0 ||
                    gX[lc] != 0 || gY[lc] != 0 || gZ[lc] != 0)
                {
                    for (int i = 0; i < DoF; i++)
                        if (!Common.IsDoubleZero(r[i]))
                            dD[i] = Dp[lc, i];
                }

                double[] tempLoadMech = Common.GetRow(FMech, lc);
                (int ok, double rmsResid) solveSystemResult = Frame3dd.SolveSystem(K, dD, tempLoadMech, dR, DoF, q, r, ok, rmsResid);
                ok = solveSystemResult.ok;
                rmsResid = solveSystemResult.rmsResid;
                for (int i = 0; i < DoF; i++)
                {
                    FMech[lc, i] = tempLoadMech[i];
                }

                for (int i = 0; i < DoF; i++)
                {
                    if (!Common.IsDoubleZero(q[i]))
                    {
                        D[i] += dD[i];
                    }
                    else
                    {
                        D[i] = Dp[lc, i]; dD[i] = 0.0;
                    }
                }

                for (int i = 0; i < DoF; i++)
                    if (!Common.IsDoubleZero(r[i]))
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

                /*  quasi Newton-Raphson iteration for geometric nonlinearity (mirrors main.c) */
                if (geom)
                {
                    error = 1.0;
                    ok = 0;
                    iter = 0;
                }

                while (geom && error > tol && iter < 500 && ok >= 0)
                {
                    ++iter;

                    // Reassemble K with current Q (now includes geometric stiffness contribution).
                    K = Frame3dd.AssembleK(DoF, nE, xyz, r, L, Le, N1, N2, Ax, Asy, Asz, Jx, Iy, Iz, E, G, p, shear, geom, Q);

                    // {dF}^(i) = {F} - [K({D}^(i))]*{D}^(i);  norm gives convergence error.
                    error = Frame3dd.EquilibriumError(dF, F, K, D, DoF, q, r);

                    // Solve [K] {dD} = {dF} for the next Newton increment.
                    (int ok, double rmsResid) nrResult = Frame3dd.SolveSystem(K, dD, dF, dR, DoF, q, r, ok, rmsResid);
                    ok = nrResult.ok;
                    rmsResid = nrResult.rmsResid;
                    if (ok < 0) break;  // K not positive definite — abort iteration

                    // Increment displacements at free DoFs.
                    for (int i = 0; i < DoF; i++)
                        if (!Common.IsDoubleZero(q[i])) D[i] += dD[i];

                    // Recompute element end forces for new D (feeds next iteration's geometric K).
                    tempTArray = Common.GetArray(eqFTemp, lc);
                    tempMArray = Common.GetArray(eqFMech, lc);
                    Frame3dd.ElementEndForces(Q, nE, xyz, L, Le, N1, N2,
                        Ax, Asy, Asz, Jx, Iy, Iz, E, G, p,
                        tempTArray, tempMArray, D, shear, geom,
                        axialStrainWarning);
                }

                // Recompute reactions for the converged non-linear K and D.
                if (geom) ComputeReactionForces(R, F, K, D, DoF, r);

                if (writeMatrix != 0)/* write static stiffness matrix */
                {
                    //save_ut_dmatrix("Ks", K, DoF, "w");//Not implemented
                }
                /*  display RMS equilibrium error */
                //if (verbose && ok >= 0)
                //    evaluate(error, rms_resid, tol, geom);

                (List<NodeDisplacement> nodeDisplacements, List<FrameElementEndForce> frameElementEndForces, List<ReactionOutput> reactionOutputs) staticResults = Frame3ddIO.GetStaticResults(nN, nE, nL, lc, DoF, N1, N2, F, D, R, r, Q, error, ok, axialSign);

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

            // Pass the static-loop's converged K (which carries geometric-stiffness
            // contributions from the final load case when geom=true) into modal so the
            // eigensolver sees the same K upstream uses. With geom=false the K is just
            // the linear elastic K — same in both cases.
            List<ModalResult> modalResults = ComputeModalAnalysis(
                input.DynamicAnalysis, DoF, nN, nE, xyz, rj, L, N1, N2,
                Ax, Jx, Iy, Iz, p, d, r, K);

            double modalTol = input.DynamicAnalysis.Tolerance > 0 ? input.DynamicAnalysis.Tolerance : 1e-9;
            var outText = outputText1
                          + OutWriter.OutputDataToString(loadCaseOutputs)
                          + OutWriter.ModalResultsToString(modalResults, nN, modalTol, input.DynamicAnalysis.MassType);

            Output output = new Output(outText, loadCaseOutputs, modalResults);
            return output;
        }

        /// <summary>
        /// Modal-analysis pipeline: assemble M, mask restrained DoFs in K and M (matching
        /// upstream main.c lines 657-670), call Stodola/Subspace, convert eigenvalues to Hz.
        /// <paramref name="staticK"/> is the converged stiffness from the static load-case
        /// loop — for <c>geom=true</c> inputs this carries the axial-force-induced
        /// geometric-stiffness contribution that softens columns in compression and shifts
        /// modal frequencies downward. Upstream's <c>main.c</c> passes whatever K was last
        /// assembled into <c>subspace()</c>; matching that gives 5–50% lower frequencies on
        /// gravity-loaded towers and frames.
        /// </summary>
        private static List<ModalResult> ComputeModalAnalysis(
            DynamicAnalysisInput dyn,
            int DoF, int nN, int nE,
            List<Vec3> xyz, List<float> rj, List<double> L,
            List<int> N1, List<int> N2,
            List<float> Ax, List<float> Jx, List<float> Iy, List<float> Iz, List<float> p,
            List<float> density, float[] r, double[,] staticK)
        {
            if (dyn.ModesCount < 1) return new List<ModalResult>();

            int nM = dyn.ModesCount;
            float[] rjArr = rj.ToArray();

            // Build per-node and per-element extra-mass arrays from the parsed dynamic input.
            List<float> NMs = new List<float>(new float[nN]);
            List<float> NMx = new List<float>(new float[nN]);
            List<float> NMy = new List<float>(new float[nN]);
            List<float> NMz = new List<float>(new float[nN]);
            foreach (NodeInertia ni in dyn.ExtraNodeInertia)
            {
                NMs[ni.NodeIdx] = (float)ni.Mass;
                NMx[ni.NodeIdx] = (float)ni.Ixx;
                NMy[ni.NodeIdx] = (float)ni.Iyy;
                NMz[ni.NodeIdx] = (float)ni.Izz;
            }
            List<float> EMs = new List<float>(new float[nE]);
            foreach (ElementMass em in dyn.ExtraElementMass)
                EMs[em.ElementIdx] = (float)em.Mass;

            bool lump = dyn.MassType == 1;
            double[,] M = Frame3dd.AssembleM(
                DoF, nN, nE, xyz, rjArr, L, N1, N2,
                Ax, Jx, Iy, Iz, p, density, EMs, NMs, NMx, NMy, NMz, lump);

            // Copy the static-loop K so the masking we apply below doesn't leak back into
            // the caller's matrix.
            double[,] Km = new double[DoF, DoF];
            for (int i = 0; i < DoF; i++)
                for (int j = 0; j < DoF; j++)
                    Km[i, j] = staticK[i, j];

            // Restrained-DoF mask: huge K diagonal pushes those modes well above the real ones.
            double traceK = 0.0, traceM = 0.0;
            for (int i = 0; i < DoF; i++)
            {
                if (r[i] != 1)
                {
                    traceK += Km[i, i];
                    traceM += M[i, i];
                }
            }
            for (int i = 0; i < DoF; i++)
            {
                if (r[i] == 1)
                {
                    Km[i, i] = traceK * 1.0e4;
                    M[i, i] = traceM;
                    for (int j = i + 1; j < DoF; j++)
                    {
                        Km[j, i] = Km[i, j] = 0.0;
                        M[j, i] = M[i, j] = 0.0;
                    }
                }
            }

            // Compute extra modes for convergence robustness — Bathe's heuristic.
            int nMCalc = (nM + 8) < (2 * nM) ? nM + 8 : 2 * nM;
            if (nMCalc > DoF) nMCalc = DoF;

            double tol = dyn.Tolerance > 0 ? dyn.Tolerance : 1e-9;
            double[] eigs;
            double[,] V;
            try
            {
                // Method 1 = Subspace–Jacobi (upstream default, every shipped example).
                // Method 2 = Stodola inverse iteration. Anything else falls back to subspace.
                (eigs, V, _) = dyn.Method == 2
                    ? Eigensolver.Stodola(Km, M, DoF, nMCalc, tol, dyn.Shift)
                    : Eigensolver.Subspace(Km, M, DoF, nMCalc, tol, dyn.Shift);
            }
            catch (InvalidOperationException ex)
            {
                // Either eigensolver can hit its iteration limit on a poorly-conditioned
                // system. Upstream aborts the whole run; we degrade gracefully and skip
                // modal results so the static-solve output is still returned.
                Console.WriteLine($"  warning: modal analysis did not converge — {ex.Message}");
                return new List<ModalResult>();
            }

            int kept = nM < nMCalc ? nM : nMCalc;
            List<ModalResult> results = new List<ModalResult>(kept);
            for (int k = 0; k < kept; k++)
            {
                double omegaSq = eigs[k];
                double f = omegaSq > 0 ? Math.Sqrt(omegaSq) / (2.0 * Math.PI) : 0.0;
                double[] shape = new double[DoF];
                for (int i = 0; i < DoF; i++) shape[i] = V[i, k];
                results.Add(new ModalResult(k, f, omegaSq, shape));
            }
            return results;
        }




        /// <summary>
        /// Recompute reactions <c>R[i] = -F[i] + sum_j K[i,j]*D[j]</c> at restrained DoFs.
        /// Mirrors upstream <c>compute_reaction_forces</c> (frame3dd.c:658). Used after a
        /// geometric-nonlinear solve where the reactions stored during the linear iterations
        /// no longer reflect the converged stiffness matrix.
        /// </summary>
        private static void ComputeReactionForces(double[] R, double[] F, double[,] K, double[] D, int DoF, float[] r)
        {
            for (int i = 0; i < DoF; i++)
            {
                R[i] = 0;
                if (!Common.IsDoubleZero(r[i]))
                {
                    R[i] = -F[i];
                    for (int j = 0; j < DoF; j++) R[i] += K[i, j] * D[j];
                }
            }
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
