namespace Frame3ddn.Model
{
    public class NodeLoad
    {
        public int NodeIdx { get; }
        public Vec3 Load { get; }
        public Vec3 Moment { get; }

        public NodeLoad(int nodeIdx, Vec3 load, Vec3 moment)
        {
            NodeIdx = nodeIdx;
            Load = load;
            Moment = moment;
        }

        public static NodeLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new NodeLoad(int.Parse(data[0]) - 1, 
                new Vec3(double.Parse(data[1]), double.Parse(data[2]), double.Parse(data[3])),
                new Vec3(double.Parse(data[4]), double.Parse(data[5]), double.Parse(data[6])));
        }
    }
}