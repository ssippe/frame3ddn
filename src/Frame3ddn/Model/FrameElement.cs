namespace Frame3ddn
{
    public class FrameElement
    {
        public int NodeIdx1 { get; }
        public int NodeIdx2 { get; }
        public float Ax { get; }
        public float Asy { get; }
        public float Asz { get; set; }
        public float Jx { get; }
        public float Iy { get; }
        public float Iz { get; }
        public float E { get; }
        public float G { get; }
        public float Roll { get; }
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