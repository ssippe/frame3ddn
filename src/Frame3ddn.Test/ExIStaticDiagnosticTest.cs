using Frame3ddn.Model;
using Frame3ddn.Parsers;
using System.IO;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Diagnostic — confirm our static Y-displacements for exI match upstream's reported
    /// values. Mode 1 of exI is a Y-direction mode and static gravity is also in Y, so any
    /// K mismatch in the Y-stiffness would show up here too.
    /// </summary>
    public class ExIStaticDiagnosticTest
    {
        // Upstream exI.out reports node 4 Y-dsp = 2.648229 under gravity. If our K is 2.23×
        // stiffer in Y (which the modal residual suggests), we'd see ≈1.19 in. Pin to the
        // upstream value with 0.5% tolerance.
        [Fact]
        public void ExI_StaticYDisplacement_MatchesUpstreamWithinHalfPercent()
        {
            using (StreamReader sr = new StreamReader(GetExamplePath("exI.csv")))
            {
                Input input = CsvInputParser.Parse(sr);
                Output output = new Frame3ddn.Solver().Solve(input);

                // Upstream reports node 4 (1-based) Y-dsp = 2.648229 in load case 1 (gravity).
                LoadCaseOutput lc = output.LoadCaseOutputs[0];
                NodeDisplacement node4 = lc.NodeDisplacements.First(d => d.NodeIdx == 3);  // 0-based

                const double upstream_node4_Ydsp = 2.648229;
                Assert.Equal(upstream_node4_Ydsp, node4.Displacement.Y, 4);

                // And node 13 (top) has Y-dsp = 56.796981
                NodeDisplacement node13 = lc.NodeDisplacements.First(d => d.NodeIdx == 12);
                const double upstream_node13_Ydsp = 56.796981;
                Assert.Equal(upstream_node13_Ydsp, node13.Displacement.Y, 4);
            }
        }

        // Pin a few mode-shape values against the .out file we parsed so we can confirm the
        // parser places X/Y/Z/xx/yy/zz in the right DoF slots.
        [Fact]
        public void ExI_ParsedMode1Shape_MatchesOutFileValues()
        {
            string outText = File.ReadAllText(GetExamplePath("exI.out"));
            var modes = OutOutputParser.ParseModalResults(outText);
            ModalResult mode1 = modes[0];

            // From upstream exI.out node 4: " 4   8.081e-03   1.162e+01   3.342e-01  -2.399e-01  -3.187e-02   2.348e-04"
            // Order is X-dsp, Y-dsp, Z-dsp, X-rot, Y-rot, Z-rot.
            // Node 4 (1-based) = node index 3 (0-based) → DoFs 18..23.
            Assert.Equal(8.081e-03, mode1.ModeShape[18], 5);
            Assert.Equal(1.162e+01, mode1.ModeShape[19], 5);
            Assert.Equal(3.342e-01, mode1.ModeShape[20], 5);
            Assert.Equal(-2.399e-01, mode1.ModeShape[21], 5);
            Assert.Equal(-3.187e-02, mode1.ModeShape[22], 5);
            Assert.Equal(2.348e-04, mode1.ModeShape[23], 5);

            // Node 13 (1-based) = node index 12 → DoFs 72..77.
            // " 13  -1.601e-02   1.984e+02   1.072e+00  -3.178e-01  -8.800e-02   6.068e-03"
            Assert.Equal(-1.601e-02, mode1.ModeShape[72], 5);
            Assert.Equal(1.984e+02, mode1.ModeShape[73], 4);  // larger value, looser precision
        }

        private static string GetExamplePath(string fileName)
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            return Path.Combine(testDataPath, "frame3dd-examples", fileName);
        }
    }
}
