namespace Frame3ddn.Model
{
    /// <summary>
    /// Concentrated point load applied at a location along the local x-axis of a frame
    /// element (frame3dd_io.c around line 1268). Equivalent nodal forces are derived via
    /// standard cantilever-load distribution; see upstream <c>P[lc][i][1..5]</c>.
    /// </summary>
    public class InternalConcentratedLoad
    {
        /// <summary>Element index, 0-based.</summary>
        public int ElementIdx { get; }
        /// <summary>Load components in element-local coordinates (Px along the axis).</summary>
        public Vec3 Load { get; }
        /// <summary>Position along the element from end 1, 0 ≤ x ≤ L.</summary>
        public double X { get; }

        public InternalConcentratedLoad(int elementIdx, Vec3 load, double x)
        {
            ElementIdx = elementIdx;
            Load = load;
            X = x;
        }

        public static InternalConcentratedLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new InternalConcentratedLoad(
                int.Parse(data[0]) - 1,
                new Vec3(double.Parse(data[1]), double.Parse(data[2]), double.Parse(data[3])),
                double.Parse(data[4]));
        }
    }
}
