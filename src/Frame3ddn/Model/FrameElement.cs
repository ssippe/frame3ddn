namespace Frame3ddn
{
    public class FrameElement
    {
        /// <summary>
        /// Node Index Element Start base-0
        /// </summary>
        public int NodeIdx1 { get; }
        /// <summary>
        /// Node Index Element End base-0
        /// </summary>
        public int NodeIdx2 { get; }
        /// <summary>
        /// Cross-sectional area of a prismatic frame element
        /// (The x-axis is along the element length, in local coordinates)
        /// [mm^2]
        /// </summary>
        public float Ax { get; }
        /// <summary>
        /// Shear area in the local y-axis of a prismatic frame element [mm^2]
        /// </summary>
        public float Asy { get; }
        /// <summary>
        /// Shear area in the local z-axis of a prismatic frame element [mm^2]
        /// </summary>
        public float Asz { get; }
        /// <summary>
        /// Torsional moment of inertia of a frame element [mm^4]
        /// </summary>
        public float Jx { get; }
        /// <summary>
        /// Moment of inertia for bending about the local y axis [mm^4]
        /// </summary>
        public float Iy { get; }
        /// <summary>
        /// Moment of inertia for bending about the local z axis [mm^4]
        /// </summary>
        public float Iz { get; }
        /// <summary>
        /// Modulus of elasticity of a frame element [MPa]
        /// </summary>
        public float E { get; }
        /// <summary>
        /// Shear modulus of elasticity of frame element i [MPa]
        /// </summary>
        public float G { get; }
        /// <summary>
        /// The roll angle of the frame element [degrees]
        /// </summary>
        public float Roll { get; }
        /// <summary>
        /// Mass density of a frame element [tonne/mm^3]
        /// </summary>
        public float Density { get; }

        public FrameElement(int nodeNum1, int nodeNum2, float ax, float asy, float asz, float jx, float iy, float iz, float e, float g, float roll, float density)
        {
            NodeIdx1 = nodeNum1;
            NodeIdx2 = nodeNum2;
            Ax = ax;
            Asy = asy;
            Asz = asz;
            Jx = jx;
            Iy = iy;
            Iz = iz;
            E = e;
            G = g;
            Roll = roll;
            Density = density;
        }

        public static FrameElement Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new FrameElement(
                int.Parse(data[1]) - 1,//Convert the nodes number to be 0 based.
                int.Parse(data[2]) - 1,
                float.Parse(data[3]),
                float.Parse(data[4]),
                float.Parse(data[5]),
                float.Parse(data[6]),
                float.Parse(data[7]),
                float.Parse(data[8]),
                float.Parse(data[9]),
                float.Parse(data[10]),
                float.Parse(data[11]),
                float.Parse(data[12]));
        }
    }
}