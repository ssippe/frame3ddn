namespace Frame3ddn.Model
{
    /// <summary>
    /// Extra distributed mass on a frame element, used for modal analysis.
    /// Mirrors upstream <c>EMs[]</c> in <c>frame3dd_io.c read_mass_data</c>.
    /// </summary>
    public class ElementMass
    {
        /// <summary>Element index, 0-based.</summary>
        public int ElementIdx { get; }
        /// <summary>Extra mass on the element (in addition to that derived from density × area × length).</summary>
        public double Mass { get; }

        public ElementMass(int elementIdx, double mass)
        {
            ElementIdx = elementIdx;
            Mass = mass;
        }
    }
}
