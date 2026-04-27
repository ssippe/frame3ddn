using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Frame3ddn.Writers
{

    public static class OutWriter
    {
        private const string Version = "20140514+";
        public static string InputDataToString(string title, int nN, int nE, int nL, int[] nD, int nR, List<int> nF, List<int> nU, List<int> nW, int[] nP, int[] nT,
            List<Vec3Float> xyz, List<float> r, List<int> J1, List<int> J2, List<float> Ax, List<float> Asy, List<float> Asz, List<float> Jx, List<float> Iy, List<float> Iz, List<float> E, List<float> G, List<float> p,
            List<float> d, List<float> gX, List<float> gY, List<float> gZ,
            double[,] Fm, double[,] Dp, float[] R, float[,,] U, float[,,] W, float[,,] P, bool shear, bool geom)
        {
            StringBuilder txt = new StringBuilder();
            txt.AppendLine("________________________________________________________________________________");
            txt.AppendLine($"Frame3DD version: {Version}               http://frame3dd.sf.net/");
            txt.AppendLine("GPL Copyright (C) 1992-2015, Henri P. Gavin ");
            txt.AppendLine("Frame3DD is distributed in the hope that it will be useful but with no warranty.");
            txt.AppendLine("For details see the GNU Public Licence: http://www.fsf.org/copyleft/gpl.html");
            txt.AppendLine("________________________________________________________________________________");
            txt.AppendLine(title);
            txt.AppendLine(DateTime.Now.ToLongDateString() + " " + DateTime.Now.ToLongTimeString());
            txt.AppendLine("________________________________________________________________________________");
            txt.AppendLine("In 2D problems the Y-axis is vertical.  In 3D problems the Z-axis is vertical.");
            txt.AppendLine("________________________________________________________________________________");
            txt.AppendLine(
                $"{nN.Format(5)} NODES {nR.Format(5)} FIXED NODES" +
                $"{nE.Format(5)} FRAME ELEMENTS {nL.Format(3)} LOAD CASES   ");
            txt.AppendLine("________________________________________________________________________________");
            txt.AppendLine("N O D E   D A T A                                           R E S T R A I N T S");
            txt.AppendLine("  Node       X              Y              Z         radius  Fx Fy Fz Mx My Mz");

            for (int i = 0; i < nN; i++)
            {
                int j = 6 * i;
                txt.AppendLine($"{(i + 1).Format(5)} {xyz[i].X.Format(14.6)} {xyz[i].Y.Format(14.6)} {xyz[i].Z.Format(14.6)} " +
                               $"{r[i].Format(8.3)} {R[j + 0].Format(2)} {R[j + 1].Format(2)} {R[j + 2].Format(2)}" +
                               $" {R[j + 3].Format(2)} {R[j + 4].Format(2)} {R[j + 5].Format(2)}");
            }
            txt.AppendLine("F R A M E   E L E M E N T   D A T A					(local)");
            txt.AppendLine("  Elmnt  J1    J2     Ax   Asy   Asz    Jxx     Iyy     Izz       E       G roll  density");
            //fprintf(fp,"%5d %5d %5d    %6.1f   %5.1f   %5.1f",
            //            i, J1[i],J2[i], Ax[i], Asy[i], Asz[i] );
            // fprintf(fp," %6.1f   %7.1f %7.1f  %8.1f %7.1f %3.0f          %8.2e\n",
            //              Jx[i], Iy[i], Iz[i], E[i], G[i], p[i]*180.0/PI, d[i] );
            for (int i = 0; i < nE; i++)
            {
                txt.AppendLine($"{(i + 1).Format(5)} {(J1[i] + 1).Format(5)} {(J2[i] + 1).Format(5)} {Ax[i].Format(6.1)} {Asy[i].Format(6.1)} {Asz[i].Format(6.1)}" +
                               $" {Jx[i].Format(6.1)} {Iy[i].Format(7.1)} {Iz[i].Format(7.1)} {E[i].Format(8.1)} {G[i].Format(7.1)}" +
                               $" {(p[i] * 180.0 / Math.PI).Format(3.0)} {d[i].EFormat(8.2)}");
            }

            if (shear)
            {
                txt.AppendLine("  Include shear deformations.");
            }
            else
            {
                txt.AppendLine("  Neglect shear deformations.");
            }

            if (geom)
            {
                txt.AppendLine("  Include geometric stiffness.");
            }
            else
            {
                txt.AppendLine("  Neglect geometric stiffness.");
            }

            for (int lc = 0; lc < nL; lc++)
            {
                txt.AppendLine("");
                txt.AppendLine($"L O A D   C A S E   {lc + 1}   O F   {nL}  ... ");
                txt.AppendLine("");
                txt.AppendLine(
                    $"   Gravity X =  {gX[lc].Format(0.3)}    Gravity Y =  {gY[lc].Format(0.3)}    Gravity Z =  {gZ[lc].Format(0.3)}");
                txt.AppendLine($" {nF[lc].Format(3)} concentrated loads");
                txt.AppendLine($" {nU[lc].Format(3)} uniformly distributed loads");
                txt.AppendLine($" {nW[lc].Format(3)} trapezoidally distributed loads");
                txt.AppendLine($" {nP[lc].Format(3)} concentrated point loads");
                txt.AppendLine($" {nT[lc].Format(3)} temperature loads");
                txt.AppendLine($" {nD[lc].Format(3)} prescribed displacements");
                if (nF[lc] > 0 || nU[lc] > 0 || nW[lc] > 0 || nP[lc] > 0 || nT[lc] > 0)
                {
                    txt.AppendLine(" N O D A L   L O A D S  +  E Q U I V A L E N T   N O D A L   L O A D S  (global)");
                    txt.AppendLine("  Node        Fx          Fy          Fz          Mxx         Myy         Mzz");
                    for (int j = 0; j < nN; j++)
                    {
                        int i = j * 6;
                        if (!Common.IsDoubleZero(Fm[lc, i + 0]) || !Common.IsDoubleZero(Fm[lc, i + 1]) || !Common.IsDoubleZero(Fm[lc, i + 2]) ||
                            !Common.IsDoubleZero(Fm[lc, i + 3]) || !Common.IsDoubleZero(Fm[lc, i + 4]) || !Common.IsDoubleZero(Fm[lc, i + 5]))
                        {
                            txt.AppendLine(
                                $" {(j + 1).Format(5)} {Fm[lc, 6 * j + 0].Format(11.3)} {Fm[lc, 6 * j + 1].Format(11.3)}" +
                                $" {Fm[lc, 6 * j + 2].Format(11.3)} {Fm[lc, 6 * j + 3].Format(11.3)} {Fm[lc, 6 * j + 4].Format(11.3)}" +
                                $" {Fm[lc, 6 * j + 5].Format(11.3)}");
                        }
                    }
                }

                if (nU[lc] > 0)
                {
                    txt.AppendLine(" U N I F O R M   L O A D S						(local)");
                    txt.AppendLine("  Elmnt       Ux               Uy               Uz");
                    for (int n = 0; n < nU[lc]; n++)
                    {
                        txt.AppendLine($" {((int)U[lc, n, 0] + 1).Format(5)} {U[lc, n, 1].Format(16.8)} {U[lc, n, 2].Format(16.8)}" +
                                       $" {U[lc, n, 3].Format(16.8)}");
                    }
                }

                if (nW[lc] > 0)
                {
                    txt.AppendLine(" T R A P E Z O I D A L   L O A D S					(local)");
                    txt.AppendLine("  Elmnt       x1               x2               W1               W2");
                    for (int n = 0; n < nW[lc]; n++)
                    {
                        txt.Append($" {((int)W[lc, n, 0] + 1).Format(5)}");
                        for (int i = 1; i < 5; i++)
                        {
                            txt.Append($" {W[lc, n, i].Format(16.8)}");
                        }
                        txt.AppendLine("  (x)");
                        txt.Append($" {((int)W[lc, n, 0] + 1).Format(5)}");
                        for (int i = 5; i < 9; i++)
                        {
                            txt.Append($" {W[lc, n, i].Format(16.8)}");
                        }
                        txt.AppendLine("  (y)");
                        txt.Append($" {((int)W[lc, n, 0] + 1).Format(5)}");
                        for (int i = 9; i < 13; i++)
                        {
                            txt.Append($" {W[lc, n, i].Format(16.8)}");
                        }
                        txt.AppendLine("  (z)");

                    }
                }
            }
            //if(anlyz)
            txt.AppendLine("");
            txt.AppendLine("E L A S T I C   S T I F F N E S S   A N A L Y S I S   via  L D L'  decomposition");
            txt.AppendLine("");
            return txt.ToString();
        }



        public static string OutputDataToString(List<LoadCaseOutput> loadCaseOutputs)
        {
            int nL = loadCaseOutputs.Count;
            StringBuilder txt = new StringBuilder();
            for (int i = 0; i < nL; i++)
            {
                LoadCaseOutput loadCaseOutput = loadCaseOutputs[i];

                txt.AppendLine($"L O A D   C A S E   {i + 1}   O F   {nL}  ... ");
                txt.AppendLine();

                txt.AppendLine("N O D E   D I S P L A C E M E N T S  					(global)");
                txt.AppendLine("  Node    X-dsp       Y-dsp       Z-dsp       X-rot       Y-rot       Z-rot");
                foreach (var nodeDisplacement in loadCaseOutput.NodeDisplacements)
                {
                    txt.AppendLine($" {(nodeDisplacement.NodeIdx + 1).Format(5)}" +
                                   $" {nodeDisplacement.Displacement.X.Format(11.6)}" +
                                   $" {nodeDisplacement.Displacement.Y.Format(11.6)}" +
                                   $" {nodeDisplacement.Displacement.Z.Format(11.6)}" +
                                   $" {nodeDisplacement.Rotation.X.Format(11.6)}" +
                                   $" {nodeDisplacement.Rotation.Y.Format(11.6)}" +
                                   $" {nodeDisplacement.Rotation.Z.Format(11.6)}");
                }

                txt.AppendLine("F R A M E   E L E M E N T   E N D   F O R C E S				(local)");
                txt.AppendLine("  Elmnt  Node       Nx          Vy         Vz        Txx        Myy        Mzz");
                foreach (var frameElementEndForce in loadCaseOutput.FrameElementEndForces)
                {
                    txt.AppendLine($" {(frameElementEndForce.ElementIdx + 1).Format(5)}" +
                                   $" {(frameElementEndForce.NodeIdx + 1).Format(5)}" +
                                   $" {frameElementEndForce.Nx.Format(10.3)}" +
                                   $"{frameElementEndForce.NxType}" +
                                   $" {frameElementEndForce.Vy.Format(10.3)}" +
                                   $" {frameElementEndForce.Vz.Format(10.3)}" +
                                   $" {frameElementEndForce.Txx.Format(10.3)}" +
                                   $" {frameElementEndForce.Myy.Format(10.3)}" +
                                   $" {frameElementEndForce.Mzz.Format(10.3)}");
                }

                txt.AppendLine("R E A C T I O N S							(global)");
                txt.AppendLine("  Node        Fx          Fy          Fz         Mxx         Myy         Mzz");
                foreach (var reactionOutput in loadCaseOutput.ReactionOutputs)
                {
                    txt.AppendLine($" {(reactionOutput.NodeIdx + 1).Format(5)}" +
                                   $" {reactionOutput.F.X.Format(11.3)}" +
                                   $" {reactionOutput.F.Y.Format(11.3)}" +
                                   $" {reactionOutput.F.Z.Format(11.3)}" +
                                   $" {reactionOutput.M.X.Format(11.3)}" +
                                   $" {reactionOutput.M.Y.Format(11.3)}" +
                                   $" {reactionOutput.M.Z.Format(11.3)}");
                }

                txt.AppendLine("R M S    R E L A T I V E    E Q U I L I B R I U M    E R R O R: " + loadCaseOutput.RmsRelativeEquilibriumError);

                txt.AppendLine("");
                txt.AppendLine("P E A K   F R A M E   E L E M E N T   I N T E R N A L   F O R C E S(local)");
                txt.AppendLine("  Elmnt   .         Nx          Vy         Vz        Txx        Myy        Mzz");
                if (loadCaseOutput.PeakFrameElementInternalForces != null)
                    foreach (var peakFrameElementInternalForce in loadCaseOutput.PeakFrameElementInternalForces.Where(f => f.IsMin.HasValue))
                    {
                        string minMax = peakFrameElementInternalForce.IsMin.Value ? "min" : "max";
                        txt.AppendLine($" {(peakFrameElementInternalForce.ElementIdx + 1).Format(5)}   {minMax} " +
                                       $" {peakFrameElementInternalForce.Nx.Format(10.3)}" +
                                       $" {peakFrameElementInternalForce.Vy.Format(10.3)}" +
                                       $" {peakFrameElementInternalForce.Vz.Format(10.3)}" +
                                       $" {peakFrameElementInternalForce.Txx.Format(10.3)}" +
                                       $" {peakFrameElementInternalForce.Myy.Format(10.3)}" +
                                       $" {peakFrameElementInternalForce.Mzz.Format(10.3)}");
                    }

                txt.AppendLine();
            }


            return txt.ToString();
        }
    }
}
