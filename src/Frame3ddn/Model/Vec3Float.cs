namespace Frame3ddn.Model
{
    public struct Vec3Float
    {
        public float X { get; }
        public float Y { get; }
        public float Z { get; }

        public Vec3Float(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public override string ToString() => $"{X:G3},{Y:G3},{Z:G3}";
    }
}