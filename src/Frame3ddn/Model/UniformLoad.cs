namespace Frame3ddn.Model
{
    public class UniformLoad
    {
        /// <summary>
        /// Element Index base-0
        /// </summary>
        public int ElementIdx { get; }
        /// <summary>
        /// Load [N/mm]
        /// </summary>
        public Vec3Float Load { get; }

        public UniformLoad(int elementIdx, Vec3Float load)
        {
            ElementIdx = elementIdx;
            Load = load;
        }

        public static UniformLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new UniformLoad(int.Parse(data[0]) - 1,
                new Vec3Float(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3])));
        }

        public override string ToString() => $"ElementIdx={ElementIdx}, Load={Load}";        
    }
}