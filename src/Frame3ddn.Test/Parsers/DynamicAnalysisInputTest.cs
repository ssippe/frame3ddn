using Frame3ddn.Model;
using Frame3ddn.Parsers;
using System.IO;
using Xunit;

namespace Frame3ddn.Test.Parsers
{
    public class DynamicAnalysisInputTest
    {
        // Expected dynamic-section shape for each upstream example (frame3dd repo
        // examples/exX.csv). Pulled by reading the trailing dynamic-analysis section
        // of each file. ModesCount==0 means "no modal analysis"; the parser returns
        // DynamicAnalysisInput.None in that case (Method==0, etc.).
        [Theory]
        [InlineData("exA",  0, 0, 0, 0, 0, 0, 0)]
        [InlineData("exB",  6, 1, 1, 0, 6, 0, 0)]
        [InlineData("exC", 10, 1, 0, 0, 5, 0, 0)]
        [InlineData("exD", 14, 1, 0, 0, 5, 0, 0)]
        [InlineData("exE",  4, 1, 1, 0, 4, 2, 1)]
        [InlineData("exF", 15, 1, 4, 0, 5, 2, 4)]
        [InlineData("exG",  4, 1, 0, 0, 4, 0, 0)]
        [InlineData("exH",  5, 1, 0, 163, 5, 0, 0)]
        [InlineData("exI",  4, 1, 0, 0, 4, 0, 0)]
        [InlineData("exJ", 22, 1, 0, 0, 8, 0, 0)]
        public void Frame3ddExamplesDynamicAnalysis(
            string fileName,
            int expectedModesCount,
            int expectedMethod,
            int expectedExtraInertiaCount,
            int expectedExtraElementMassCount,
            int expectedAnimatedModesCount,
            int expectedCondensationMethod,
            int expectedCondensedNodesCount)
        {
            string inputPath = GetExamplePath(fileName + ".csv");
            using (StreamReader sr = new StreamReader(inputPath))
            {
                Input input = CsvInputParser.Parse(sr);
                DynamicAnalysisInput dyn = input.DynamicAnalysis;

                Assert.NotNull(dyn);
                Assert.Equal(expectedModesCount, dyn.ModesCount);
                Assert.Equal(expectedMethod, dyn.Method);
                Assert.Equal(expectedExtraInertiaCount, dyn.ExtraNodeInertia.Count);
                Assert.Equal(expectedExtraElementMassCount, dyn.ExtraElementMass.Count);
                Assert.Equal(expectedAnimatedModesCount, dyn.AnimatedModes.Count);
                Assert.Equal(expectedCondensationMethod, dyn.CondensationMethod);
                Assert.Equal(expectedCondensedNodesCount, dyn.CondensedNodes.Count);
            }
        }

        // exF is the most interesting case — a full Guyan condensation with extra
        // node inertia, animated modes and a condensed-modes list. Verify the
        // contents (not just the counts) so a parser regression on any field is
        // caught.
        [Fact]
        public void ExFFullDynamicSectionContents()
        {
            string inputPath = GetExamplePath("exF.csv");
            using (StreamReader sr = new StreamReader(inputPath))
            {
                Input input = CsvInputParser.Parse(sr);
                DynamicAnalysisInput dyn = input.DynamicAnalysis;

                Assert.Equal(15, dyn.ModesCount);
                Assert.Equal(1, dyn.Method);                     // subspace-Jacobi
                Assert.Equal(0, dyn.MassType);                   // consistent
                Assert.Equal(1e-6, dyn.Tolerance, 12);
                Assert.Equal(0.0, dyn.Shift);

                // Four nodes (6, 19, 26, 33) each carry the same extra inertia.
                Assert.Equal(4, dyn.ExtraNodeInertia.Count);
                int[] expectedNodes = { 5, 18, 25, 32 }; // 0-based
                for (int i = 0; i < 4; i++)
                {
                    NodeInertia n = dyn.ExtraNodeInertia[i];
                    Assert.Equal(expectedNodes[i], n.NodeIdx);
                    Assert.Equal(4.0, n.Mass);
                    Assert.Equal(4750000.0, n.Ixx);
                    Assert.Equal(3620000.0, n.Iyy);
                    Assert.Equal(26530000.0, n.Izz);
                }

                Assert.Empty(dyn.ExtraElementMass);

                Assert.Equal(new[] { 1, 2, 5, 9, 12 }, dyn.AnimatedModes);
                Assert.Equal(1.5, dyn.PanRate, 6);

                // Condensation: Guyan, four nodes each retaining x, y, zz (3 DoFs/node).
                Assert.Equal(2, dyn.CondensationMethod);
                Assert.Equal(4, dyn.CondensedNodes.Count);
                for (int i = 0; i < 4; i++)
                {
                    CondensedNode cn = dyn.CondensedNodes[i];
                    Assert.Equal(expectedNodes[i], cn.NodeIdx);
                    Assert.Equal(new[] { true, true, false, false, false, true }, cn.Dof);
                }

                // 4 nodes × 3 retained DoFs = 12 mode tokens
                Assert.Equal(new[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }, dyn.CondensedModes);
            }
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
