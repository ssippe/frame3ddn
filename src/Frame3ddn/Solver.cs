using System;
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
            List<Vec3> xyz = input.Nodes.Select(n => n.Position).ToList();
            int Dof = 6 * nN;
            int nR = input.ReactionInputs.Count;
            double[] q = new double[Dof];
            double[] r = new double[Dof];
            for (int i = 0; i < nR; i++)//><
            {
                int j = input.ReactionInputs[i].Number;//>< This number is corrected wehen importing
                r[j * 6 + 0] = input.ReactionInputs[i].Position.X;
                r[j * 6 + 1] = input.ReactionInputs[i].Position.Y;
                r[j * 6 + 2] = input.ReactionInputs[i].Position.Z;
                r[j * 6 + 3] = input.ReactionInputs[i].R.X;
                r[j * 6 + 4] = input.ReactionInputs[i].R.Y;
                r[j * 6 + 5] = input.ReactionInputs[i].R.Z;
            }
            for (int i = 0; i < Dof; i++)
            {
                q[i] = (r[i] == (double)1) ? 0 : 1;
            }

            int nE = input.FrameElements.Count;

            List<double> L = input.FrameElements.Select(f =>
                    Coordtrans.CalculateSQDistance(input.Nodes[f.NodeIdx1].Position, input.Nodes[f.NodeIdx2].Position))
                .ToList();

            List<double> Le = new List<double>();
            for (int i = 0; i < L.Count; i++)//><
            {
                Le.Add(L[i] - input.Nodes[input.FrameElements[i].NodeIdx1].Radius -
                        input.Nodes[input.FrameElements[i].NodeIdx2].Radius);
            }

            List<int> N1 = input.FrameElements.Select(f => f.NodeIdx1).ToList();
            List<int> N2 = input.FrameElements.Select(f => f.NodeIdx2).ToList();
            List<double> Ax = input.FrameElements.Select(f => f.Ax).ToList();
            List<double> Asy = input.FrameElements.Select(f => f.Asy).ToList();
            List<double> Asz = input.FrameElements.Select(f => f.Asz).ToList();
            List<double> Iy = input.FrameElements.Select(f => f.Iy).ToList();
            List<double> Iz = input.FrameElements.Select(f => f.Iz).ToList();
            List<double> E = input.FrameElements.Select(f => f.E).ToList();
            List<double> G = input.FrameElements.Select(f => f.G).ToList();
            List<double> p = input.FrameElements.Select(f => f.Roll).ToList();
            List<double> d = input.FrameElements.Select(f => f.Density).ToList();
            int nL = input.LoadCases.Count;
            List<double> gX = input.LoadCases.Select(l => l.Gravity.X).ToList();
            List<double> gY = input.LoadCases.Select(l => l.Gravity.Y).ToList();
            List<double> gZ = input.LoadCases.Select(l => l.Gravity.Z).ToList();
            List<int> nF = input.LoadCases.Select(l => l.NodeLoads.Count).ToList();

            AssembleLoads(nN, nE, nL, Dof, xyz, L, Le, N1, N2, Ax, Asy, Asz, Iy, Iz, E, G, p, d, gX, gY, gZ, nF);
            return null;
        }

        private void AssembleLoads(int nN, int nE, int nL, int Dof, List<Vec3> xyz, List<double> L, List<double> Le, List<int> N1, List<int> N2,
            List<double> Ax, List<double> Asy, List<double> Asz, List<double> Iy, List<double> Iz, List<double> E, List<double> G, List<double> p,
            List<double> d, List<double> gX, List<double> gY, List<double> gZ, List<int> nF)
        {
            double[,,] eqFMech = new double[nL, nE, 12];
            for (int lc = 0; lc < nL; lc++)//><
            {
                for (int n = 0; n < nE; n++)//><
                {
                    double[] t = Coordtrans.coordTrans(xyz, L[n], N1[n], N2[n], p[n]);

                    eqFMech[lc, n, 0] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0;//><
                    eqFMech[lc, n, 1] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0;//><
                    eqFMech[lc, n, 2] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0;//><

                    eqFMech[lc, n, 3] = d[n] * Ax[n] * L[n] * L[n] / 12.0 * ((-t[3] * t[7] + t[4] * t[6]) * gY[lc] + (-t[3] * t[8] + t[5] * t[6]) * gZ[lc]);//><
                    eqFMech[lc, n, 4] = d[n] * Ax[n] * L[n] * L[n] / 12.0 * ((-t[4] * t[6] + t[3] * t[7]) * gX[lc] + (-t[4] * t[8] + t[5] * t[7]) * gZ[lc]);//><
                    eqFMech[lc, n, 5] = d[n] * Ax[n] * L[n] * L[n] / 12.0 * ((-t[5] * t[6] + t[3] * t[8]) * gX[lc] + (-t[5] * t[7] + t[4] * t[8]) * gY[lc]);//><

                    eqFMech[lc, n, 6] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0;//><
                    eqFMech[lc, n, 7] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0;//><
                    eqFMech[lc, n, 8] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0;//><

                    eqFMech[lc, n, 9] = d[n] * Ax[n] * L[n] * L[n] / 12.0 * ((t[3] * t[7] - t[4] * t[6]) * gY[lc] + (t[3] * t[8] - t[5] * t[6]) * gZ[lc]);//><
                    eqFMech[lc, n, 10] = d[n] * Ax[n] * L[n] * L[n] / 12.0 * ((t[4] * t[6] - t[3] * t[7]) * gX[lc] + (t[4] * t[8] - t[5] * t[7]) * gZ[lc]);//><
                    eqFMech[lc, n, 11] = d[n] * Ax[n] * L[n] * L[n] / 12.0 * ((t[5] * t[6] - t[3] * t[8]) * gX[lc] + (t[5] * t[7] - t[4] * t[8]) * gY[lc]);//><
                }


            }
            /* end gravity loads */


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
            var k = new double[dof,dof];
            return k;
        }
    }
}
