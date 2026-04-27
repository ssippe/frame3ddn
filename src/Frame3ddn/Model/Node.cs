using System.Linq;

namespace Frame3ddn.Model
{
    public class Node
    {
        public Node(Vec3 position, float radius)
        {
            Position = position;
            Radius = radius;
        }

        /// <summary>
        /// Position [mm]. Stored as <see cref="Vec3"/> (double) to match upstream
        /// frame3dd's vec3 type and avoid catastrophic cancellation in coordTrans.
        /// </summary>
        public Vec3 Position { get; }

        /// <summary>
        /// Radius [mm]
        /// </summary>
        public float Radius { get; }

        public static Node Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new Node(
                new Vec3(double.Parse(data[1]), double.Parse(data[2]), double.Parse(data[3])),
                float.Parse(data[4]));
        }
    }
}