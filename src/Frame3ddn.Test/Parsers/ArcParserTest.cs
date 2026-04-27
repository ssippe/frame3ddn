using Frame3ddn.Model;
using Frame3ddn.Parsers;
using System;
using System.IO;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test.Parsers
{
    public class ArcParserTest
    {
        [Theory]
        [InlineData("lateral-column")]
        [InlineData("lateral-column-rev")]
        [InlineData("lateral-column-x")]
        [InlineData("lateral-column-y")]
        [InlineData("bent-cantilever")]
        [InlineData("truss")]
        [InlineData("testcase3")]
        public void ParsesUpstreamArcFixture(string name)
        {
            using StreamReader sr = new StreamReader(GetArcPath(name));
            Input input = ArcInputParser.Parse(sr);

            Assert.NotNull(input.Title);
            Assert.NotEmpty(input.Nodes);
            Assert.NotEmpty(input.FrameElements);
            Assert.NotEmpty(input.LoadCases);
        }

        [Fact]
        public void ParsesLateralColumn()
        {
            using StreamReader sr = new StreamReader(GetArcPath("lateral-column"));
            Input input = ArcInputParser.Parse(sr);

            Assert.Equal(2, input.Nodes.Count);
            Assert.Equal(0f, input.Nodes[0].Position.X);
            Assert.Equal(0f, input.Nodes[0].Position.Y);
            Assert.Equal(0f, input.Nodes[0].Position.Z);
            Assert.Equal(1f, input.Nodes[1].Position.Z);

            // Node 1 has flags 111111 -> a fully-fixed reaction; Node 2 is free.
            Assert.Single(input.ReactionInputs);
            Assert.Equal(0, input.ReactionInputs[0].NodeIdx);
            Assert.Equal(1f, input.ReactionInputs[0].Force.X);
            Assert.Equal(1f, input.ReactionInputs[0].Force.Y);
            Assert.Equal(1f, input.ReactionInputs[0].Force.Z);
            Assert.Equal(1f, input.ReactionInputs[0].Moment.X);
            Assert.Equal(1f, input.ReactionInputs[0].Moment.Y);
            Assert.Equal(1f, input.ReactionInputs[0].Moment.Z);

            Assert.Single(input.FrameElements);
            FrameElement e0 = input.FrameElements[0];
            Assert.Equal(0, e0.NodeIdx1);
            Assert.Equal(1, e0.NodeIdx2);
            Assert.Equal(3.74e-04f, e0.Ax, 6);
            Assert.Equal(1.84e-07f, e0.Jx, 9);
            Assert.Equal(7.78e-08f, e0.Iy, 9);
            Assert.Equal(2.04e-07f, e0.Iz, 9);
            Assert.Equal(2.0e+08f, e0.E);
            // G = E / (2 * (1 + nu)) = 2.0e8 / (2 * 1.25) = 8.0e7
            Assert.Equal(8.0e7f, e0.G, 0);
            Assert.Equal(7.85f, e0.Density);

            Assert.Single(input.LoadCases);
            LoadCase lc = input.LoadCases[0];
            Assert.Single(lc.NodeLoads);
            NodeLoad nl = lc.NodeLoads[0];
            Assert.Equal(1, nl.NodeIdx);    // NDLD references node ID 2 -> 0-based idx 1
            Assert.Equal(5.0, nl.Load.X);
            Assert.Equal(3.0, nl.Load.Y);
        }

        [Fact]
        public void HandlesSparseNodeIds()
        {
            // truss.arc nodes 1..12 are dense, but member IDs are sparse (101, 102, 201, ...).
            // Verify the parser maps via .arc IDs correctly.
            using StreamReader sr = new StreamReader(GetArcPath("truss"));
            Input input = ArcInputParser.Parse(sr);

            Assert.Equal(12, input.Nodes.Count);
            Assert.Equal(22, input.FrameElements.Count);
            // Node 1 has flags 111100 (X,Y,Z,Mx fixed; My,Mz free), Node 11 has 011000 (Y,Z fixed).
            Assert.Equal(2, input.ReactionInputs.Count);
        }

        [Fact]
        public void HandlesGravityLoadCase()
        {
            using StreamReader sr = new StreamReader(GetArcPath("testcase3"));
            Input input = ArcInputParser.Parse(sr);

            // testcase3 has CASE 1 Gravity, CASE 2 Nodal load, CASE 3 COMB (combination, parsed as empty).
            Assert.Equal(3, input.LoadCases.Count);
            LoadCase gravityCase = input.LoadCases[0];
            Assert.Equal(0f, gravityCase.Gravity.X);
            Assert.Equal(0f, gravityCase.Gravity.Y);
            Assert.Equal(-9.81f, gravityCase.Gravity.Z);

            LoadCase nodalCase = input.LoadCases[1];
            Assert.Single(nodalCase.NodeLoads);
            Assert.Equal(10.0, nodalCase.NodeLoads[0].Load.X);
            Assert.Equal(40.0, nodalCase.NodeLoads[0].Load.Y);
            Assert.Equal(20.0, nodalCase.NodeLoads[0].Load.Z);
        }

        [Theory]
        [InlineData("lateral-column")]
        [InlineData("lateral-column-rev")]
        [InlineData("lateral-column-x")]
        [InlineData("lateral-column-y")]
        [InlineData("bent-cantilever")]
        [InlineData("truss")]
        [InlineData("testcase3")]
        public void SolvesArcFixture(string name)
        {
            using StreamReader sr = new StreamReader(GetArcPath(name));
            Input input = ArcInputParser.Parse(sr);

            Solver solver = new Solver();
            Output output = solver.Solve(input);

            Assert.Equal(input.LoadCases.Count, output.LoadCaseOutputs.Count);
            foreach (LoadCaseOutput lc in output.LoadCaseOutputs)
            {
                foreach (NodeDisplacement nd in lc.NodeDisplacements)
                {
                    AssertFinite(nd.Displacement.X, $"{name} disp X node {nd.NodeIdx}");
                    AssertFinite(nd.Displacement.Y, $"{name} disp Y node {nd.NodeIdx}");
                    AssertFinite(nd.Displacement.Z, $"{name} disp Z node {nd.NodeIdx}");
                    AssertFinite(nd.Rotation.X, $"{name} rot X node {nd.NodeIdx}");
                    AssertFinite(nd.Rotation.Y, $"{name} rot Y node {nd.NodeIdx}");
                    AssertFinite(nd.Rotation.Z, $"{name} rot Z node {nd.NodeIdx}");
                }
                foreach (ReactionOutput r in lc.ReactionOutputs)
                {
                    AssertFinite(r.F.X, $"{name} react Fx node {r.NodeIdx}");
                    AssertFinite(r.F.Y, $"{name} react Fy node {r.NodeIdx}");
                    AssertFinite(r.F.Z, $"{name} react Fz node {r.NodeIdx}");
                }
            }
        }

        [Theory]
        [InlineData("lateral-column")]
        [InlineData("lateral-column-rev")]
        [InlineData("lateral-column-x")]
        [InlineData("lateral-column-y")]
        public void ReactionsBalanceAppliedNodalLoads(string name)
        {
            // For statically determinate cantilevers, ΣF_react = -ΣF_applied (Newton's 3rd law).
            // Applied loads come from the parsed .arc itself rather than being hard-coded.
            using StreamReader sr = new StreamReader(GetArcPath(name));
            Input input = ArcInputParser.Parse(sr);

            Solver solver = new Solver();
            Output output = solver.Solve(input);

            LoadCase lc = input.LoadCases[0];
            double appliedFx = lc.NodeLoads.Sum(n => n.Load.X);
            double appliedFy = lc.NodeLoads.Sum(n => n.Load.Y);
            double appliedFz = lc.NodeLoads.Sum(n => n.Load.Z);

            double sumFx = output.LoadCaseOutputs[0].ReactionOutputs.Sum(r => r.F.X);
            double sumFy = output.LoadCaseOutputs[0].ReactionOutputs.Sum(r => r.F.Y);
            double sumFz = output.LoadCaseOutputs[0].ReactionOutputs.Sum(r => r.F.Z);

            AssertClose(-appliedFx, sumFx, 0.05, $"{name} ΣFx");
            AssertClose(-appliedFy, sumFy, 0.05, $"{name} ΣFy");
            AssertClose(-appliedFz, sumFz, 0.05, $"{name} ΣFz");
        }

        [Fact]
        public void RoundTripsThroughSolver()
        {
            // Smoke test: the parsed Input is solver-compatible end-to-end.
            using StreamReader sr = new StreamReader(GetArcPath("lateral-column"));
            Input input = ArcInputParser.Parse(sr);

            Solver solver = new Solver();
            Output output = solver.Solve(input);
            Assert.Single(output.LoadCaseOutputs);
            Assert.NotEmpty(output.LoadCaseOutputs[0].NodeDisplacements);
            Assert.NotEmpty(output.LoadCaseOutputs[0].ReactionOutputs);
        }

        private static void AssertFinite(double value, string label)
        {
            Assert.False(double.IsNaN(value) || double.IsInfinity(value),
                $"{label} = {value} is not finite");
        }

        private static void AssertClose(double expected, double actual, double tolerance, string label)
        {
            Assert.True(Math.Abs(expected - actual) <= tolerance,
                $"{label}: expected {expected}, got {actual} (tolerance {tolerance})");
        }

        private static string GetArcPath(string name) => Path.Combine(GetArcExamplesDir(), name + ".arc");

        private static string GetArcExamplesDir()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            return Path.Combine(testDataPath, "ArcExamples");
        }
    }
}
