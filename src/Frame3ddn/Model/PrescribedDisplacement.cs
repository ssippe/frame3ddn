namespace Frame3ddn.Model
{
    public class PrescribedDisplacement
    {
        /// <summary>
        /// Node Index base-0
        /// </summary>
        public int NodeIdx { get; }

        /// <summary>
        /// Prescribed translation [length units of the input]
        /// </summary>
        public Vec3 Displacement { get; }

        /// <summary>
        /// Prescribed rotation [rad]
        /// </summary>
        public Vec3 Rotation { get; }

        public PrescribedDisplacement(int nodeIdx, Vec3 displacement, Vec3 rotation)
        {
            NodeIdx = nodeIdx;
            Displacement = displacement;
            Rotation = rotation;
        }

        public static PrescribedDisplacement Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new PrescribedDisplacement(
                int.Parse(data[0]) - 1,
                new Vec3(double.Parse(data[1]), double.Parse(data[2]), double.Parse(data[3])),
                new Vec3(double.Parse(data[4]), double.Parse(data[5]), double.Parse(data[6])));
        }

        public override string ToString() => $"NodeIdx={NodeIdx} Displacement={Displacement} Rotation={Rotation}";
    }
}
