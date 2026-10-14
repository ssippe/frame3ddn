using Frame3ddn.Model;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Axial (local x) component of interior concentrated loads.
    /// </summary>
    /// <remarks>
    /// These cases are asserted against closed-form results rather than against an upstream
    /// reference output, because upstream frame3dd_io.c distributes the axial component the wrong
    /// way round and its output cannot serve as an oracle here. The transverse components are
    /// unaffected and stay covered by the exB fixture.
    ///
    /// For a load P at distance a from node 1 (b = L - a) the fixed-end axial forces are
    /// P*b/L at node 1 and P*a/L at node 2 — the share grows as the load moves *closer*.
    /// Equivalent statements: the two segments act as springs k1 = EA/a and k2 = EA/b, giving
    /// R1 = P*b/L; and the consistent load vector for the linear axial shape function is
    /// P*N1(a) = P*(1 - a/L) = P*b/L.
    ///
    /// All values are asserted with their sign: the load acts in +x, so both reactions act in -x,
    /// the segment between node 1 and the load is in tension and the segment beyond it is in
    /// compression.
    /// </remarks>
    public class InternalConcentratedAxialLoadTest
    {
        private const double L = 4000.0;   // mm
        private const double Px = 12000.0; // N
        private const double Ax = 1000.0;  // mm^2
        private const double E = 200000.0; // N/mm^2
        private const double Dx = 100.0;   // mm, internal force diagram increment

        /// <summary>
        /// Load at a quarter point: node 1 must take three quarters, node 2 one quarter.
        /// </summary>
        [Fact]
        public void AxialInteriorLoad_AtQuarterPoint_SplitsByDistanceToFarEnd()
        {
            var (r1, r2) = SolveAxialReactions(1000.0);

            // a = L/4, b = 3L/4 -> node 1 takes P*b/L = 9000 N, node 2 takes P*a/L = 3000 N.
            Assert.Equal(-0.75 * Px, r1, 6);
            Assert.Equal(-0.25 * Px, r2, 6);
        }

        /// <summary>
        /// Degenerate case: the load sits exactly on node 1, so node 1 must take all of it.
        /// </summary>
        /// <remarks>
        /// This is the case that needs no mechanics at all, and the one that exposes the reversed
        /// split most bluntly: with the shares swapped the whole axial load is handed to the far
        /// node, while the transverse formulas in the same block correctly give everything to
        /// node 1.
        /// </remarks>
        [Fact]
        public void AxialInteriorLoad_AtStartNode_GoesEntirelyToThatNode()
        {
            var (r1, r2) = SolveAxialReactions(0.0);

            Assert.Equal(-Px, r1, 6);
            Assert.Equal(0.0, r2, 6);
        }

        /// <summary>
        /// A position that is not a simple fraction of L, so a split that happens to be right at
        /// a symmetric or quarter point cannot pass by coincidence.
        /// </summary>
        [Fact]
        public void AxialInteriorLoad_AtArbitraryPosition_SplitsByDistanceToFarEnd()
        {
            var (r1, r2) = SolveAxialReactions(700.0);

            Assert.Equal(-Px * 3300.0 / L, r1, 6);
            Assert.Equal(-Px * 700.0 / L, r2, 6);
        }

        /// <summary>
        /// Element end forces (Q = k·d − eqF): node 1 end in tension at P*b/L, node 2 end in
        /// compression at P*a/L.
        /// </summary>
        [Fact]
        public void AxialInteriorLoad_AtQuarterPoint_EndForcesMatchSegmentForces()
        {
            var endForces = Solve(BuildInput(1000.0, node2FreeInX: false)).FrameElementEndForces
                .ToDictionary(f => f.NodeIdx);

            // At node 1 a tensile element pulls the node in +x, so the end force is negative.
            Assert.Equal(-0.75 * Px, endForces[0].Nx, 6);
            Assert.Equal("t", endForces[0].NxType);
            Assert.Equal(-0.25 * Px, endForces[1].Nx, 6);
            Assert.Equal("c", endForces[1].NxType);
        }

        /// <summary>
        /// Internal axial force diagram: constant tension P*b/L up to the load, constant
        /// compression P*a/L beyond it.
        /// </summary>
        [Fact]
        public void AxialInteriorLoad_AtQuarterPoint_InternalDiagramSteps()
        {
            var diagram = Solve(BuildInput(1000.0, node2FreeInX: false)).PeakFrameElementInternalForces
                .Where(f => f.IsMin == null)
                .ToDictionary(f => f.XOffset.Value, f => f.Nx);

            // Positive internal Nx is tensile.
            Assert.Equal(0.75 * Px, diagram[500.0], 3);
            Assert.Equal(-0.25 * Px, diagram[3000.0], 3);
        }

        /// <summary>
        /// Node 2 free to slide in X: the bar is axially determinate, so node 1 reacts the whole
        /// load and only the segment 0..a stretches, giving u2 = P*a/(E*A).
        /// </summary>
        /// <remarks>
        /// Unlike the fully fixed cases, the solved displacement here depends on how eqF is split:
        /// with the shares swapped u2 would be P*b/(E*A).
        /// </remarks>
        [Fact]
        public void AxialInteriorLoad_Node2FreeInX_OnlyLoadedSegmentStretches()
        {
            var output = Solve(BuildInput(1000.0, node2FreeInX: true));

            var u2 = output.NodeDisplacements.Single(d => d.NodeIdx == 1).Displacement.X;
            Assert.Equal(Px * 1000.0 / (E * Ax), u2, 9);

            var r1 = output.ReactionOutputs.Single(r => r.NodeIdx == 0).F.X;
            Assert.Equal(-Px, r1, 6);
        }

        /// <summary>
        /// Solves the fully fixed fixture and returns the signed global X reactions at node 1 and
        /// node 2.
        /// </summary>
        private static (double node1, double node2) SolveAxialReactions(double a)
        {
            var byNode = Solve(BuildInput(a, node2FreeInX: false)).ReactionOutputs
                .ToDictionary(r => r.NodeIdx, r => r.F.X);
            return (byNode[0], byNode[1]);
        }

        private static LoadCaseOutput Solve(Input input) =>
            new Solver().Solve(input).LoadCaseOutputs.Single();

        /// <summary>
        /// A single bar along global X with both ends restrained, carrying an axial interior
        /// point load at <paramref name="a"/> from node 1.
        /// </summary>
        /// <remarks>
        /// With both ends fully fixed the axial problem is statically indeterminate, so the
        /// reactions are exactly the fixed-end forces under test. <paramref name="node2FreeInX"/>
        /// releases node 2 in global X only, making the bar axially determinate.
        /// </remarks>
        private static Input BuildInput(double a, bool node2FreeInX)
        {
            var nodes = new List<Node>
            {
                new Node(new Vec3(0.0, 0.0, 0.0), 0.0f),
                new Node(new Vec3(L, 0.0, 0.0), 0.0f),
            };

            // The remaining section properties only matter for bending and torsion, which this
            // fixture does not exercise.
            var elements = new List<FrameElement>
            {
                new FrameElement(0, 1, (float)Ax, 800.0f, 800.0f, 1.0e6f, 1.0e6f, 1.0e6f,
                    (float)E, 79300.0f, 0.0f, 7.85e-9f),
            };

            var restrained = new Vec3Float(1, 1, 1);
            var node2Translation = node2FreeInX ? new Vec3Float(0, 1, 1) : restrained;
            var reactionInputs = new List<ReactionInput>
            {
                new ReactionInput(0, restrained, restrained),
                new ReactionInput(1, node2Translation, restrained),
            };

            var loadCase = new LoadCase(
                new Vec3Float(0, 0, 0),
                new List<NodeLoad>(),
                new List<UniformLoad>(),
                new List<TrapLoad>(),
                new List<PrescribedDisplacement>(),
                new List<TemperatureLoad>(),
                new List<InternalConcentratedLoad> { new InternalConcentratedLoad(0, new Vec3(Px, 0.0, 0.0), a) });

            return new Input("Axial interior concentrated load", nodes, elements, reactionInputs,
                new List<LoadCase> { loadCase },
                includeShearDeformation: false, includeGeometricStiffness: false,
                exaggerateMeshDeformations: 1.0f, zoomScale: 1.0f,
                xAxisIncrementForInternalForces: (float)Dx);
        }
    }
}
