namespace Frame3ddn.Model
{
    /// <summary>
    /// Extra concentrated mass and rotational inertia at a node, used for modal analysis.
    /// Mirrors upstream <c>NMs/NMx/NMy/NMz</c> arrays in <c>frame3dd_io.c read_mass_data</c>.
    /// </summary>
    public class NodeInertia
    {
        /// <summary>Node index, 0-based.</summary>
        public int NodeIdx { get; }
        /// <summary>Translational mass at the node.</summary>
        public double Mass { get; }
        /// <summary>Rotational inertia about local x.</summary>
        public double Ixx { get; }
        /// <summary>Rotational inertia about local y.</summary>
        public double Iyy { get; }
        /// <summary>Rotational inertia about local z.</summary>
        public double Izz { get; }

        public NodeInertia(int nodeIdx, double mass, double ixx, double iyy, double izz)
        {
            NodeIdx = nodeIdx;
            Mass = mass;
            Ixx = ixx;
            Iyy = iyy;
            Izz = izz;
        }
    }
}
