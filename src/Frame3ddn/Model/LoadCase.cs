using System;
using System.Collections.Generic;
using System.Linq;
using Frame3ddn.Core;

namespace Frame3ddn.Model
{
    public class LoadCase
    {
        /// <summary>
        /// Gravitational acceleration for self-weight loading, (global) [mm/s^2] 
        /// </summary>
        public Vec3Float Gravity { get; }
        /// <summary>
        /// Loaded nodes (global)
        /// </summary>
        public IReadOnlyList<NodeLoad> NodeLoads { get; }
        /// <summary>
        /// Uniformly-distributed element loads (local)
        /// </summary>
        public IReadOnlyList<UniformLoad> UniformLoads { get; }
        /// <summary>
        /// Trapezoidally-distributed element loads (local)
        /// </summary>
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

        public override string ToString()
        {
            var uniformLoads =
                (UniformLoads?.Any() == true)
                    ? $"UniformLoads ({UniformLoads.Count})"
                      + Environment.NewLine
                      + UniformLoads.Select(f => f.ToString()).JoinString(Environment.NewLine)
                    : null;
            var nodeLoadText =
                (NodeLoads?.Any() == true)
                    ? $"NodeLoads ({NodeLoads.Count})"
                      + Environment.NewLine
                      + NodeLoads
                          .Select(f=>f.ToString())
                          .JoinString(Environment.NewLine)
                    : null;

            var trapLoadsText =
                (TrapLoads?.Any() == true)
                    ? $"TrapLoads ({TrapLoads.Count})"
                      + Environment.NewLine
                      + TrapLoads
                          .Select(f => f.ToString())
                          .JoinString(Environment.NewLine)
                    : null;

            var str = new[] { uniformLoads, nodeLoadText, trapLoadsText }.Where(f => f != null)
                .JoinString(Environment.NewLine + Environment.NewLine);
            return str;
        }
    }
}