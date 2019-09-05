using System.Collections.Generic;

namespace Frame3ddn.Model
{
    public class LoadCase
    {
        public Vec3Float Gravity { get; }
        public IReadOnlyList<NodeLoad> NodeLoads { get; }
        public IReadOnlyList<UniformLoad> UniformLoads { get; }
        public IReadOnlyList<TrapLoad> TrapLoads { get; }

        public LoadCase(Vec3Float gravity,
            IReadOnlyList<NodeLoad> nodeLoads,
            IReadOnlyList<UniformLoad> uniformLoads,
            IReadOnlyList<TrapLoad> trapLoads)
        {
            Gravity = gravity;
            NodeLoads = nodeLoads;
            UniformLoads = uniformLoads;
            TrapLoads = trapLoads;
        }

        public static LoadCase Parse(string inputGravityString, List<NodeLoad> nodeLoads, List<UniformLoad> uniformLoads, List<TrapLoad> trapLoads)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputGravityString, @"\s{1,}");
            return new LoadCase(
                new Vec3Float(float.Parse(data[0]), float.Parse(data[1]), float.Parse(data[2])),
                nodeLoads,
                uniformLoads,
                trapLoads
            );
        }


    }
}