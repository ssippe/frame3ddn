namespace Frame3ddn.Model
{
    public struct Vec3
    {
        public double X { get; }
        public double Y { get;}
        public double Z { get; }

        public Vec3(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public override string ToString() => $"{X:G3},{Y:G3},{Z:G3}";        
    }
}