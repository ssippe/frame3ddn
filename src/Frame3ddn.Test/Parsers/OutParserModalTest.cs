using Frame3ddn.Model;
using Frame3ddn.Parsers;
using System.IO;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test.Parsers
{
    /// <summary>
    /// Tests for OutOutputParser.ParseModalResults — both standalone (parses upstream's
    /// reference .out files correctly) and as a round-trip with OutWriter.
    /// </summary>
    public class OutParserModalTest
    {
        // Confirms the parser reads upstream's exB.out modal section: 6 modes with the
        // documented frequencies (18.81, 19.11, 19.69, 31.71, 35.16, 42.25 Hz).
        [Fact]
        public void ParseModalResults_UpstreamExBOut_ExtractsAllSixModes()
        {
            string text = File.ReadAllText(GetExamplePath("exB.out"));
            var modes = OutOutputParser.ParseModalResults(text);

            Assert.Equal(6, modes.Count);
            double[] expectedHz = { 18.807942, 19.105451, 19.690439, 31.711570, 35.159165, 42.248953 };
            for (int i = 0; i < expectedHz.Length; i++)
                Assert.Equal(expectedHz[i], modes[i].FrequencyHz, 6);

            // Each upstream MODE block has a per-node row for all 5 nodes × 6 DoFs = 30 entries.
            foreach (ModalResult m in modes) Assert.Equal(30, m.ModeShape.Count);
        }

        // Round-trip: render the parsed upstream modes via OutWriter, parse again, verify
        // the second parse matches the first. This pins both ends of the writer/parser pair.
        [Fact]
        public void ParseModalResults_RoundTripsThroughOutWriter()
        {
            string text = File.ReadAllText(GetExamplePath("exB.out"));
            var modes = OutOutputParser.ParseModalResults(text);

            string rendered = Frame3ddn.Writers.OutWriter.ModalResultsToString(
                modes, nodeCount: 5, convergenceTolerance: 1e-9, massType: 0);

            var roundTripped = OutOutputParser.ParseModalResults(rendered);

            Assert.Equal(modes.Count, roundTripped.Count);
            for (int i = 0; i < modes.Count; i++)
            {
                Assert.Equal(modes[i].ModeIndex, roundTripped[i].ModeIndex);
                Assert.Equal(modes[i].FrequencyHz, roundTripped[i].FrequencyHz, 6);
                Assert.Equal(modes[i].ModeShape.Count, roundTripped[i].ModeShape.Count);
                // Mode-shape values are emitted with 3 significant digits, so round-trip
                // precision is bounded at ~1e-3 of the value's magnitude.
                for (int k = 0; k < modes[i].ModeShape.Count; k++)
                {
                    double expected = modes[i].ModeShape[k];
                    double actual = roundTripped[i].ModeShape[k];
                    Assert.True(System.Math.Abs(expected - actual) < 1e-3 * (System.Math.Abs(expected) + 1e-9),
                        $"mode {i} dof {k}: expected {expected:e3}, got {actual:e3}");
                }
            }
        }

        // Cross-validation: solve through C# (Subspace iteration, the upstream default for
        // every shipped example) and compare the lowest mode frequency to upstream's
        // reported value within a 1% band. Modal now uses the static loop's converged K,
        // matching upstream's main.c — this carries geometric-stiffness softening for
        // gravity-loaded structures (geom=true on every example except exD, exJ).
        //
        // Excluded:
        //   • exD, exJ — unrestrained structures with rigid-body modes near 0 Hz; relative
        //     comparison is meaningless when both values are at the numerical noise floor.
        [Theory]
        [InlineData("exB")]
        [InlineData("exC")]
        [InlineData("exE")]
        [InlineData("exF")]
        [InlineData("exG")]
        [InlineData("exH")]
        [InlineData("exI")]
        public void Frame3ddExamples_LowestModeMatchesUpstream(string fileName)
        {
            string upstreamText = File.ReadAllText(GetExamplePath(fileName + ".out"));
            var upstreamModes = OutOutputParser.ParseModalResults(upstreamText);
            Assert.NotEmpty(upstreamModes);
            double upstreamLowestHz = upstreamModes.Min(m => m.FrequencyHz);

            using (StreamReader sr = new StreamReader(GetExamplePath(fileName + ".csv")))
            {
                Input input = CsvInputParser.Parse(sr);
                Output output = new Frame3ddn.Solver().Solve(input);

                Assert.NotEmpty(output.ModalResults);
                double csharpLowestHz = output.ModalResults.Min(m => m.FrequencyHz);

                double relErr = System.Math.Abs(csharpLowestHz - upstreamLowestHz) / upstreamLowestHz;
                string csharpAll = string.Join(", ", output.ModalResults.Select(m => m.FrequencyHz.ToString("f4")));
                string upstreamAll = string.Join(", ", upstreamModes.Select(m => m.FrequencyHz.ToString("f4")));
                Assert.True(relErr < 0.01,
                    $"{fileName}: C#={csharpAll} Hz | upstream={upstreamAll} Hz | rel.err={relErr:p2} (allowed 1%)");
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
