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
    /// </remarks>
    public class InternalConcentratedAxialLoadTest
    {
        private const double L = 4000.0;   // mm
        private const double Px = 12000.0; // N

        /// <summary>
        /// Load at a quarter point: node 1 must take three quarters, node 2 one quarter.
        /// </summary>
        [Fact]
        public void AxialInteriorLoad_AtQuarterPoint_SplitsByDistanceToFarEnd()
        {
            var (r1, r2) = SolveAxialReactions(1000.0);

            // a = L/4, b = 3L/4 -> node 1 takes P*b/L = 9000 N, node 2 takes P*a/L = 3000 N.
            Assert.Equal(0.75 * Px, r1, 6);
            Assert.Equal(0.25 * Px, r2, 6);
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

            Assert.Equal(Px, r1, 6);
            Assert.Equal(0.0, r2, 6);
        }

        /// <summary>
        /// Equilibrium holds regardless of how the shares are distributed — this is why the
        /// reversed split stayed silent: the sum is always P.
        /// </summary>
        [Fact]
        public void AxialInteriorLoad_AnyPosition_PreservesTotal()
        {
            var (r1, r2) = SolveAxialReactions(700.0);

            Assert.Equal(Px, r1 + r2, 6);
        }

        /// <summary>
        /// Solves the fixture and returns the magnitudes of the axial reactions at node 1 and
        /// node 2.
        /// </summary>
        private static (double node1, double node2) SolveAxialReactions(double a)
        {
            var reactions = new Solver().Solve(BuildInput(a)).LoadCaseOutputs.Single().ReactionOutputs;
            var byNode = reactions.ToDictionary(r => r.NodeIdx, r => System.Math.Abs(r.F.X));
            return (byNode[0], byNode[1]);
        }

        /// <summary>
        /// A single bar along global X with both ends fully restrained, carrying an axial interior
        /// point load at <paramref name="a"/> from node 1.
        /// </summary>
        /// <remarks>
        /// Both ends fixed makes the axial problem statically indeterminate, so the reactions are
        /// exactly the fixed-end forces under test.
        /// </remarks>
        private static Input BuildInput(double a)
        {
            var nodes = new List<Node>
            {
                new Node(new Vec3(0.0, 0.0, 0.0), 0.0f),
                new Node(new Vec3(L, 0.0, 0.0), 0.0f),
            };

            // Ax = 1000 mm^2, E = 200000 N/mm^2; the remaining section properties only matter for
            // bending and torsion, which this fixture does not exercise.
            var elements = new List<FrameElement>
            {
                new FrameElement(0, 1, 1000.0f, 800.0f, 800.0f, 1.0e6f, 1.0e6f, 1.0e6f,
                    200000.0f, 79300.0f, 0.0f, 7.85e-9f),
            };

            var restrained = new Vec3Float(1, 1, 1);
            var reactionInputs = new List<ReactionInput>
            {
                new ReactionInput(0, restrained, restrained),
                new ReactionInput(1, restrained, restrained),
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
                xAxisIncrementForInternalForces: -1.0f);
        }
    }
}
