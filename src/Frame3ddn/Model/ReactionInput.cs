using System.Linq;

namespace Frame3ddn.Model
{
    public class ReactionInput
    {
        public ReactionInput(int nodeIdx, Vec3Float force, Vec3Float moment)
        {            
            NodeIdx = nodeIdx;
            Force = force;
            Moment = moment;
        }

        /// <summary>
        /// Node Index base-0
        /// </summary>
        public int NodeIdx { get; }

        /// <summary>
        /// Rx - 1: reaction force in the global X direction,  0: free
        /// Ry - 1: reaction force in the global Y direction,  0: free
        /// Rz - 1: reaction force in the global Z direction,  0: free
        /// </summary>
        public Vec3Float Force { get; }

        /// <summary>
        /// Rxx - 1: reaction moment about the global X axis, 0: free
        /// Ryy - 1: reaction moment about the global Y axis, 0: free
        /// Rzz - 1: reaction moment about the global Z axis, 0: free
        /// </summary>
        public Vec3Float Moment { get; }

        public static ReactionInput Parse(string inputString)
        {
            int[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}").Select(int.Parse).ToArray();
            return new ReactionInput(data[0] - 1, new Vec3Float(data[1], data[2], data[3]), new Vec3Float(data[4], data[5], data[6]));
        }
    }
}