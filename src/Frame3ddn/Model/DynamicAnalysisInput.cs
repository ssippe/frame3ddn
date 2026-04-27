using System.Collections.Generic;

namespace Frame3ddn.Model
{
    /// <summary>
    /// Per-input dynamic-modal-analysis configuration. Mirrors the trailing section of the
    /// upstream input files parsed by <c>read_mass_data</c> + <c>read_condensation_data</c>
    /// in <c>frame3dd_io.c</c>.
    ///
    /// The C# solver does not (yet) consume these values — they are parsed and surfaced so
    /// that downstream tools can inspect the same data the upstream binary would use, and so
    /// that the parser fails loudly rather than silently when the format drifts.
    /// </summary>
    public class DynamicAnalysisInput
    {
        /// <summary>Number of modes requested. <c>0</c> means "no modal analysis".</summary>
        public int ModesCount { get; }

        /// <summary>1 = subspace–Jacobi iteration, 2 = Stodola.</summary>
        public int Method { get; }

        /// <summary>0 = consistent mass matrix, 1 = lumped mass matrix.</summary>
        public int MassType { get; }

        /// <summary>Mode-shape convergence tolerance.</summary>
        public double Tolerance { get; }

        /// <summary>Frequency shift used for unrestrained structures.</summary>
        public double Shift { get; }

        /// <summary>Exaggeration factor applied when plotting modal mesh deformations.</summary>
        public double ExaggerateModalDeformations { get; }

        /// <summary>Nodes carrying additional concentrated mass / inertia.</summary>
        public IReadOnlyList<NodeInertia> ExtraNodeInertia { get; }

        /// <summary>Frame elements carrying additional distributed mass.</summary>
        public IReadOnlyList<ElementMass> ExtraElementMass { get; }

        /// <summary>Mode numbers (1-based) the upstream viewer animates.</summary>
        public IReadOnlyList<int> AnimatedModes { get; }

        /// <summary>Pan rate during mode animation.</summary>
        public double PanRate { get; }

        /// <summary>0 = none, 1 = static, 2 = Guyan, 3 = dynamic. <c>0</c> when no condensation requested.</summary>
        public int CondensationMethod { get; }

        /// <summary>Per-node retained-DoF flags for matrix condensation.</summary>
        public IReadOnlyList<CondensedNode> CondensedNodes { get; }

        /// <summary>
        /// Mode numbers (1-based) to match for dynamic condensation. Length equals the total
        /// number of "1" flags across <see cref="CondensedNodes"/>. Empty unless
        /// <see cref="CondensationMethod"/> > 0.
        /// </summary>
        public IReadOnlyList<int> CondensedModes { get; }

        public DynamicAnalysisInput(
            int modesCount,
            int method,
            int massType,
            double tolerance,
            double shift,
            double exaggerateModalDeformations,
            IReadOnlyList<NodeInertia> extraNodeInertia,
            IReadOnlyList<ElementMass> extraElementMass,
            IReadOnlyList<int> animatedModes,
            double panRate,
            int condensationMethod,
            IReadOnlyList<CondensedNode> condensedNodes,
            IReadOnlyList<int> condensedModes)
        {
            ModesCount = modesCount;
            Method = method;
            MassType = massType;
            Tolerance = tolerance;
            Shift = shift;
            ExaggerateModalDeformations = exaggerateModalDeformations;
            ExtraNodeInertia = extraNodeInertia;
            ExtraElementMass = extraElementMass;
            AnimatedModes = animatedModes;
            PanRate = panRate;
            CondensationMethod = condensationMethod;
            CondensedNodes = condensedNodes;
            CondensedModes = condensedModes;
        }

        /// <summary>Sentinel value used when no dynamic-analysis section is present.</summary>
        public static DynamicAnalysisInput None { get; } = new DynamicAnalysisInput(
            modesCount: 0,
            method: 0,
            massType: 0,
            tolerance: 0.0,
            shift: 0.0,
            exaggerateModalDeformations: 0.0,
            extraNodeInertia: new List<NodeInertia>(),
            extraElementMass: new List<ElementMass>(),
            animatedModes: new List<int>(),
            panRate: 0.0,
            condensationMethod: 0,
            condensedNodes: new List<CondensedNode>(),
            condensedModes: new List<int>());
    }
}
