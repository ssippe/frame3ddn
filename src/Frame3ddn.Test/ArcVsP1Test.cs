using Frame3ddn.Model;
using Frame3ddn.Parsers;
using Frame3ddn.Writers;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Solves each .arc fixture and compares the result against the corresponding
    /// Microstran .p1 reference report (parsed with <see cref="P1Parser"/>) using
    /// <see cref="OutputAsserts.OutputAssertEqual"/>.
    /// </summary>
    public class ArcVsP1Test
    {
        // Per-fixture absolute tolerance. lateral-column.p1 is non-linear (P-Δ) so the moment
        // residual can be ~0.2 kNm; the others are linear-elastic and much tighter.
        // lateral-column.p1 is from a Microstran *large-displacement* non-linear analysis;
        // the others are linear-elastic. Our P-Δ geometric-stiffness path doesn't help on
        // lateral-column because the load is purely transverse (T=0 → Kg=0); the residual
        // there is flexural shortening which needs large-displacement analysis we don't
        // implement, hence the looser tolerance.
        [Theory]
        [InlineData("lateral-column",     0.30, false)]
        [InlineData("lateral-column-rev", 0.05, false)]
        [InlineData("lateral-column-x",   0.05, false)]
        [InlineData("lateral-column-y",   0.05, false)]
        public void OutputMatchesMicrostranP1Reference(string name, double absoluteTolerance, bool geometricStiffness)
        {
            using StreamReader sr = new StreamReader(GetArcPath(name));
            Input input = ArcParser.Parse(sr, includeGeometricStiffness: geometricStiffness);
            Solver solver = new Solver();
            Output actual = solver.Solve(input);

            Dictionary<int, int> nodeIdToIdx = ReadArcNodeOrder(GetArcPath(name));
            Output expected = BuildExpectedFromP1(
                P1Parser.Parse(File.ReadAllText(GetP1Path(name))),
                nodeIdToIdx);

            // Write both sides to TestResults/ so they can be diffed by hand. The solver's
            // TextOutput is `OutWriter.InputDataToString + OutWriter.OutputDataToString` — strip
            // the output portion off the end to recover the shared input preamble, then re-render
            // each side's output section through the same formatter so any visible diff is real.
            string actualOutputSection = OutWriter.OutputDataToString(actual.LoadCaseOutputs.ToList());
            int outputStart = actual.TextOutput.IndexOf(actualOutputSection, StringComparison.Ordinal);
            string preamble = outputStart >= 0 ? actual.TextOutput.Substring(0, outputStart) : "";
            WriteOutPair(name,
                preamble + OutWriter.OutputDataToString(expected.LoadCaseOutputs.ToList()),
                preamble + actualOutputSection);

            OutputAsserts.OutputAssertEqual(expected, actual, new OutputAssertOptions
            {
                RelativeTolerance = 0.05,
                AbsoluteTolerance = absoluteTolerance,
                IgnoreFrameElementEndForces = true, // .p1 member-force section not parsed
                IgnorePeakForces = true,            // .p1 has no peak-force section
            });
        }

        private static void WriteOutPair(string name, string expected, string actual)
        {
            string dir = GetTestResultsDir();
            Directory.CreateDirectory(dir);
            File.WriteAllText(Path.Combine(dir, name + ".expected.out"), expected);
            File.WriteAllText(Path.Combine(dir, name + ".actual.out"), actual);
        }

        private static string GetTestResultsDir()
        {
            string projectDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory()).ToString()).ToString()).ToString();
            return Path.Combine(projectDir, "TestResults");
        }

        private static Output BuildExpectedFromP1(List<P1LoadCase> p1Cases, Dictionary<int, int> nodeIdToIdx)
        {
            List<LoadCaseOutput> lcs = new List<LoadCaseOutput>();
            for (int lc = 0; lc < p1Cases.Count; lc++)
            {
                P1LoadCase pc = p1Cases[lc];

                // Solver only emits NodeDisplacement rows for nodes with non-zero motion;
                // filter the .p1 list to match.
                List<NodeDisplacement> displacements = pc.Displacements
                    .Where(r => HasAnyValue(r.Linear) || HasAnyValue(r.Angular))
                    .Select(r => new NodeDisplacement(lc, nodeIdToIdx[r.NodeId], r.Linear, r.Angular))
                    .OrderBy(d => d.NodeIdx)
                    .ToList();

                List<ReactionOutput> reactions = pc.Reactions
                    .Select(r => new ReactionOutput(lc, nodeIdToIdx[r.NodeId], r.Linear, r.Angular))
                    .OrderBy(r => r.NodeIdx)
                    .ToList();

                lcs.Add(new LoadCaseOutput(0,
                    displacements,
                    new List<FrameElementEndForce>(),
                    reactions,
                    new List<PeakFrameElementInternalForce>()));
            }
            return new Output("", lcs);
        }

        private static bool HasAnyValue(Vec3 v) => v.X != 0 || v.Y != 0 || v.Z != 0;

        // Walks the .arc text in NODE-statement order to recover the same .arc-id -> 0-based
        // index mapping that ArcParser uses internally.
        private static Dictionary<int, int> ReadArcNodeOrder(string arcPath)
        {
            Dictionary<int, int> map = new Dictionary<int, int>();
            int idx = 0;
            foreach (string line in File.ReadAllLines(arcPath))
            {
                string trimmed = line.TrimStart();
                if (!trimmed.StartsWith("NODE")) continue;
                string[] tok = trimmed.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (tok.Length < 2) continue;
                if (int.TryParse(tok[1], out int id))
                {
                    map[id] = idx++;
                }
            }
            return map;
        }

        private static string GetArcPath(string name) => Path.Combine(GetArcExamplesDir(), name + ".arc");
        private static string GetP1Path(string name) => Path.Combine(GetArcExamplesDir(), name + ".p1");

        private static string GetArcExamplesDir()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            return Path.Combine(testDataPath, "ArcExamples");
        }
    }
}
