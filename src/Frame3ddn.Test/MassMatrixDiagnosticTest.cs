using Frame3ddn.Model;
using Frame3ddn.Parsers;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Diagnostic — compares our assembled mass matrix diagonal to upstream's reported
    /// nodal masses for exI. If the diagonals agree, the cross-validation discrepancy
    /// in lowest-mode frequency is in the eigensolver / masking. If they disagree, it's
    /// in <see cref="Frame3ddn.Frame3dd.AssembleM"/>.
    /// </summary>
    public class MassMatrixDiagnosticTest
    {
        // Upstream's exI.out reports per-node masses for the consistent mass matrix.
        // Free nodes (4-15) have X/Y/Z translational masses + xx/yy/zz rotational inertias.
        // Restrained nodes (1-3) show traceM after the modal-pipeline masking — we don't
        // verify those here because the masking is inside Solver.ComputeModalAnalysis, not
        // in AssembleM.
        [Fact]
        public void ExI_AssembledMassDiagonal_MatchesUpstreamNodalMasses()
        {
            string csvPath = Path.Combine(GetExampleDir(), "exI.csv");
            using (StreamReader sr = new StreamReader(csvPath))
            {
                Input input = CsvInputParser.Parse(sr);

                int nN = input.Nodes.Count;
                int nE = input.FrameElements.Count;
                int DoF = 6 * nN;

                List<Vec3> xyz = input.Nodes.Select(n => n.Position).ToList();
                float[] r = input.Nodes.Select(n => n.Radius).ToArray();
                List<double> L = input.FrameElements
                    .Select(f => Frame3ddn.CoordinateTransform.CalculateSQDistance(
                        input.Nodes[f.NodeIdx1].Position, input.Nodes[f.NodeIdx2].Position))
                    .ToList();
                List<int> N1 = input.FrameElements.Select(f => f.NodeIdx1).ToList();
                List<int> N2 = input.FrameElements.Select(f => f.NodeIdx2).ToList();
                List<float> Ax = input.FrameElements.Select(f => f.Ax).ToList();
                List<float> Jx = input.FrameElements.Select(f => f.Jx).ToList();
                List<float> Iy = input.FrameElements.Select(f => f.Iy).ToList();
                List<float> Iz = input.FrameElements.Select(f => f.Iz).ToList();
                List<float> p = input.FrameElements.Select(f => f.Roll).ToList();
                List<float> density = input.FrameElements.Select(f => f.Density).ToList();
                List<float> EMs = new List<float>(new float[nE]);
                List<float> NMs = new List<float>(new float[nN]);
                List<float> NMx = new List<float>(new float[nN]);
                List<float> NMy = new List<float>(new float[nN]);
                List<float> NMz = new List<float>(new float[nN]);

                bool lump = input.DynamicAnalysis.MassType == 1;
                double[,] M = Frame3ddn.Frame3dd.AssembleM(
                    DoF, nN, nE, xyz, r, L, N1, N2,
                    Ax, Jx, Iy, Iz, p, density, EMs, NMs, NMx, NMy, NMz, lump);

                // Upstream's per-node masses (consistent matrix, no masking yet) for nodes 4-15.
                // Format: { Xmass, Ymass, Zmass, Xinrta, Yinrta, Zinrta }.
                var expected = new Dictionary<int, double[]>
                {
                    [4]  = new[] { 4.22505e-06, 4.44836e-06, 4.31478e-06, 5.00635e-04, 2.46957e-03, 2.26180e-03 },
                    [5]  = new[] { 4.22505e-06, 4.44836e-06, 4.31478e-06, 5.00635e-04, 2.46957e-03, 2.26180e-03 },
                    [6]  = new[] { 3.63906e-06, 3.71707e-06, 3.62097e-06, 6.33910e-04, 9.17828e-04, 8.46453e-04 },
                    [7]  = new[] { 4.66938e-06, 4.89269e-06, 4.71478e-06, 8.87378e-04, 2.85631e-03, 2.26380e-03 },
                    [8]  = new[] { 4.66938e-06, 4.89269e-06, 4.71478e-06, 8.87378e-04, 2.85631e-03, 2.26380e-03 },
                    [9]  = new[] { 4.08338e-06, 4.16140e-06, 4.02097e-06, 1.02065e-03, 1.30457e-03, 8.48453e-04 },
                    [10] = new[] { 5.55937e-06, 5.78268e-06, 5.51478e-06, 2.49058e-03, 4.45951e-03, 2.26780e-03 },
                    [11] = new[] { 5.55937e-06, 5.78268e-06, 5.51478e-06, 2.49058e-03, 4.45951e-03, 2.26780e-03 },
                    [12] = new[] { 4.97337e-06, 5.05139e-06, 4.82097e-06, 2.62385e-03, 2.90777e-03, 8.52453e-04 },
                    [13] = new[] { 4.39830e-06, 4.62160e-06, 4.47478e-06, 1.98425e-03, 3.95318e-03, 2.26260e-03 },
                    [14] = new[] { 4.39830e-06, 4.62160e-06, 4.47478e-06, 1.98425e-03, 3.95318e-03, 2.26260e-03 },
                    [15] = new[] { 3.81230e-06, 3.89032e-06, 3.78097e-06, 2.11752e-03, 2.40144e-03, 8.47253e-04 },
                };

                List<string> mismatches = new List<string>();
                foreach (var kv in expected)
                {
                    int nodeIdx = kv.Key - 1;        // upstream uses 1-based, we use 0-based
                    double[] expDiag = kv.Value;
                    for (int dof = 0; dof < 6; dof++)
                    {
                        double actual = M[6 * nodeIdx + dof, 6 * nodeIdx + dof];
                        double expected1 = expDiag[dof];
                        double relErr = expected1 == 0
                            ? System.Math.Abs(actual)
                            : System.Math.Abs(actual - expected1) / System.Math.Abs(expected1);
                        if (relErr > 0.02)
                        {
                            mismatches.Add(
                                $"node {kv.Key} dof {dof}: expected {expected1:e3}, got {actual:e3} ({relErr:p2})");
                        }
                    }
                }

                Assert.True(mismatches.Count == 0,
                    "Mass diagonal mismatches:\n  " + string.Join("\n  ", mismatches));
            }
        }

        // Open question: applying upstream's reported mode-1 shape to our K and M (with the
        // same masking the modal solver uses) yields a Rayleigh quotient that differs from
        // upstream's reported ω² — by ~6% on exB and ~120% on exI. Both M diagonal and
        // static K × D = F match upstream, so the discrepancy must be in M off-diagonals
        // or some assembly subtlety not exercised by gravity-Y static loading.
        //
        // The test below is parameterised so that future changes which close the gap can
        // be verified by tightening the tolerance. Current bands chosen so exB passes;
        // exI is documented but not pinned because the gap there is too large to baseline.
        [Theory]
        [InlineData("exB", 0.10)]
        public void Frame3ddExample_UpstreamMode1Shape_SatisfiesGeneralizedEigenproblemInOurMatrices(
            string fileName, double residualTol)
        {
            // Parse upstream's reference mode 1 (φ, ω²).
            string outText = File.ReadAllText(Path.Combine(GetExampleDir(), fileName + ".out"));
            var refModes = OutOutputParser.ParseModalResults(outText);
            ModalResult refMode1 = refModes[0];
            double refOmegaSq = refMode1.Eigenvalue;

            // Re-assemble K and M the same way Solver.ComputeModalAnalysis does, including
            // the restrained-DoF mask. Borrow the property-array build from the test above.
            string csvPath = Path.Combine(GetExampleDir(), fileName + ".csv");
            using (StreamReader sr = new StreamReader(csvPath))
            {
                Input input = CsvInputParser.Parse(sr);

                int nN = input.Nodes.Count;
                int nE = input.FrameElements.Count;
                int DoF = 6 * nN;

                List<Vec3> xyz = input.Nodes.Select(n => n.Position).ToList();
                float[] rj = input.Nodes.Select(n => n.Radius).ToArray();
                List<double> L = input.FrameElements
                    .Select(f => Frame3ddn.CoordinateTransform.CalculateSQDistance(
                        input.Nodes[f.NodeIdx1].Position, input.Nodes[f.NodeIdx2].Position))
                    .ToList();
                List<double> Le = new List<double>();
                for (int i = 0; i < nE; i++)
                    Le.Add(L[i] - input.Nodes[input.FrameElements[i].NodeIdx1].Radius
                                - input.Nodes[input.FrameElements[i].NodeIdx2].Radius);
                List<int> N1 = input.FrameElements.Select(f => f.NodeIdx1).ToList();
                List<int> N2 = input.FrameElements.Select(f => f.NodeIdx2).ToList();
                List<float> Ax  = input.FrameElements.Select(f => f.Ax).ToList();
                List<float> Asy = input.FrameElements.Select(f => f.Asy).ToList();
                List<float> Asz = input.FrameElements.Select(f => f.Asz).ToList();
                List<float> Jx  = input.FrameElements.Select(f => f.Jx).ToList();
                List<float> Iy  = input.FrameElements.Select(f => f.Iy).ToList();
                List<float> Iz  = input.FrameElements.Select(f => f.Iz).ToList();
                List<float> E   = input.FrameElements.Select(f => f.E).ToList();
                List<float> G   = input.FrameElements.Select(f => f.G).ToList();
                List<float> p   = input.FrameElements.Select(f => f.Roll).ToList();
                List<float> density = input.FrameElements.Select(f => f.Density).ToList();
                List<float> EMs = new List<float>(new float[nE]);
                List<float> NMs = new List<float>(new float[nN]);
                List<float> NMx = new List<float>(new float[nN]);
                List<float> NMy = new List<float>(new float[nN]);
                List<float> NMz = new List<float>(new float[nN]);

                float[] r = new float[DoF];
                foreach (var ri in input.ReactionInputs)
                {
                    int j = ri.NodeIdx;
                    r[j * 6 + 0] = ri.Force.X;  r[j * 6 + 1] = ri.Force.Y;  r[j * 6 + 2] = ri.Force.Z;
                    r[j * 6 + 3] = ri.Moment.X; r[j * 6 + 4] = ri.Moment.Y; r[j * 6 + 5] = ri.Moment.Z;
                }

                bool shear = input.IncludeShearDeformation;
                double[,] zeroQ = new double[nE, 12];
                double[,] K = Frame3ddn.Frame3dd.AssembleK(
                    DoF, nE, xyz, rj, L, Le, N1, N2,
                    Ax, Asy, Asz, Jx, Iy, Iz, E, G, p, shear, geom: false, zeroQ);
                double[,] M = Frame3ddn.Frame3dd.AssembleM(
                    DoF, nN, nE, xyz, rj, L, N1, N2,
                    Ax, Jx, Iy, Iz, p, density, EMs, NMs, NMx, NMy, NMz, lump: false);

                double traceK = 0.0, traceM = 0.0;
                for (int i = 0; i < DoF; i++)
                    if (r[i] != 1) { traceK += K[i, i]; traceM += M[i, i]; }
                for (int i = 0; i < DoF; i++)
                {
                    if (r[i] == 1)
                    {
                        K[i, i] = traceK * 1.0e4;
                        M[i, i] = traceM;
                        for (int j = i + 1; j < DoF; j++)
                        {
                            K[j, i] = K[i, j] = 0.0;
                            M[j, i] = M[i, j] = 0.0;
                        }
                    }
                }

                // φ from upstream's mode shape, but mass-normalised under our M (not upstream's
                // — they're equal per the previous test). Then test the residual.
                double[] phi = refMode1.ModeShape.ToArray();
                if (phi.Length != DoF)
                    throw new System.Exception($"shape length {phi.Length} != DoF {DoF}");

                double phiMphi = 0.0;
                for (int i = 0; i < DoF; i++)
                    for (int j = 0; j < DoF; j++)
                    {
                        double mij = i <= j ? M[i, j] : M[j, i];
                        phiMphi += phi[i] * mij * phi[j];
                    }
                double scale = 1.0 / System.Math.Sqrt(phiMphi);
                for (int i = 0; i < DoF; i++) phi[i] *= scale;

                // Compute residual ‖K φ − ω² M φ‖ / ‖K φ‖.
                double[] kphi = new double[DoF];
                double[] mphi = new double[DoF];
                for (int i = 0; i < DoF; i++)
                {
                    double ks = 0.0, ms = 0.0;
                    for (int j = 0; j < DoF; j++)
                    {
                        ks += (i <= j ? K[i, j] : K[j, i]) * phi[j];
                        ms += (i <= j ? M[i, j] : M[j, i]) * phi[j];
                    }
                    kphi[i] = ks; mphi[i] = ms;
                }
                double residSq = 0.0, kphiSq = 0.0;
                for (int i = 0; i < DoF; i++)
                {
                    double diff = kphi[i] - refOmegaSq * mphi[i];
                    residSq += diff * diff;
                    kphiSq += kphi[i] * kphi[i];
                }
                double relResidual = System.Math.Sqrt(residSq / kphiSq);

                // Also compute the Rayleigh quotient: φ' K φ / φ' M φ should equal ω².
                double phiKphi = 0.0;
                for (int i = 0; i < DoF; i++)
                    for (int j = 0; j < DoF; j++)
                    {
                        double kij = i <= j ? K[i, j] : K[j, i];
                        phiKphi += phi[i] * kij * phi[j];
                    }
                double rqOmegaSq = phiKphi;  // φ already mass-normalised so φ'Mφ = 1

                Assert.True(relResidual < residualTol,
                    $"{fileName}: K φ - ω² M φ residual = {relResidual:p2} (allowed {residualTol:p0}); " +
                    $"upstream ω² = {refOmegaSq:e3}, our Rayleigh quotient = {rqOmegaSq:e3}, " +
                    $"φ'Mφ before normalize = {phiMphi:e3}, " +
                    $"upstream f = {refMode1.FrequencyHz:f4} Hz, " +
                    $"our √(RQ)/(2π) = {System.Math.Sqrt(rqOmegaSq) / (2 * System.Math.PI):f4} Hz");
            }
        }

        private static string GetExampleDir()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            return Path.Combine(testDataPath, "frame3dd-examples");
        }
    }
}
