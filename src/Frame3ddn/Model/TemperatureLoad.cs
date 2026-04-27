namespace Frame3ddn.Model
{
    /// <summary>
    /// Element thermal load. Mirrors upstream frame3dd's per-element thermal entry
    /// (frame3dd_io.c around line 1358). Equivalent nodal forces are derived as:
    ///   Nx = (alpha / 4) * (Typ + Tym + Tzp + Tzm) * E * Ax
    ///   My = (alpha / hz) * (Tzm - Tzp) * E * Iy
    ///   Mz = (alpha / hy) * (Typ - Tym) * E * Iz
    /// </summary>
    public class TemperatureLoad
    {
        /// <summary>Element index, 0-based.</summary>
        public int ElementIdx { get; }
        /// <summary>Thermal expansion coefficient [1/temperature].</summary>
        public double Alpha { get; }
        /// <summary>Cross-section dimension along local y (used for Mz from Ty gradient).</summary>
        public double Hy { get; }
        /// <summary>Cross-section dimension along local z (used for My from Tz gradient).</summary>
        public double Hz { get; }
        /// <summary>Temperature change on the +y face.</summary>
        public double Typ { get; }
        /// <summary>Temperature change on the -y face.</summary>
        public double Tym { get; }
        /// <summary>Temperature change on the +z face.</summary>
        public double Tzp { get; }
        /// <summary>Temperature change on the -z face.</summary>
        public double Tzm { get; }

        public TemperatureLoad(int elementIdx, double alpha, double hy, double hz,
            double typ, double tym, double tzp, double tzm)
        {
            ElementIdx = elementIdx;
            Alpha = alpha;
            Hy = hy;
            Hz = hz;
            Typ = typ;
            Tym = tym;
            Tzp = tzp;
            Tzm = tzm;
        }

        public static TemperatureLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new TemperatureLoad(
                int.Parse(data[0]) - 1,
                double.Parse(data[1]),
                double.Parse(data[2]),
                double.Parse(data[3]),
                double.Parse(data[4]),
                double.Parse(data[5]),
                double.Parse(data[6]),
                double.Parse(data[7]));
        }
    }
}
