using System.Linq;

namespace Frame3ddn.Model
{
    public class Node
    {
        public Node(Vec3Float position, float radius)
        {
            Position = position;
            Radius = radius;
        }

        /// <summary>
        /// Position [mm]
        /// </summary>
        public Vec3Float Position { get; }

        /// <summary>
        /// Radius [mm]
        /// </summary>
        public float Radius { get; }

        public static Node Parse(string inputString)
        {
            float[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}").Select(float.Parse).ToArray();
            return new Node(new Vec3Float(data[1], data[2], data[3]), data[4]);
        }
    }
}