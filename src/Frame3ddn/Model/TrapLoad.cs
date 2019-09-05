namespace Frame3ddn.Model
{
    public class TrapLoad
    {
        public int ElementIdx { get; }
        public Vec3Float LocationStart { get; }
        public Vec3Float LocationEnd { get; }
        public Vec3Float LoadStart { get; }
        public Vec3Float LoadEnd { get; }
        public TrapLoad(int elementIdx, Vec3Float locationStart, Vec3Float locationEnd, Vec3Float loadStart, Vec3Float loadEnd)
        {
            ElementIdx = elementIdx;
            LocationStart = locationStart;
            LocationEnd = locationEnd;
            LoadStart = loadStart;
            LoadEnd = loadEnd;
        }

        public static TrapLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new TrapLoad(int.Parse(data[0]) - 1,
                new Vec3Float(float.Parse(data[1]), float.Parse(data[5]), float.Parse(data[9])),
                new Vec3Float(float.Parse(data[2]), float.Parse(data[6]), float.Parse(data[10])),
                new Vec3Float(float.Parse(data[3]), float.Parse(data[7]), float.Parse(data[11])),
                new Vec3Float(float.Parse(data[4]), float.Parse(data[8]), float.Parse(data[12]))
            );
        }
    }
}