using System.Collections.Generic;

namespace Frame3ddn.Model
{
    /// <summary>
    /// One row of matrix-condensation data: which DoFs at a node are retained in the
    /// condensed system. Mirrors a row of upstream's <c>cm[i][1..7]</c> in
    /// <c>frame3dd_io.c read_condensation_data</c>.
    /// </summary>
    public class CondensedNode
    {
        /// <summary>Node index, 0-based.</summary>
        public int NodeIdx { get; }

        /// <summary>
        /// Six flags (x, y, z, xx, yy, zz). True means "condense this DoF" (retain it as a master DoF).
        /// </summary>
        public IReadOnlyList<bool> Dof { get; }

        public CondensedNode(int nodeIdx, IReadOnlyList<bool> dof)
        {
            NodeIdx = nodeIdx;
            Dof = dof;
        }
    }
}
