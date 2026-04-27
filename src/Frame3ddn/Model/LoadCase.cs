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
        /// <summary>
        /// Prescribed nodal displacements applied at restrained DoFs (global)
        /// </summary>
        public IReadOnlyList<PrescribedDisplacement> PrescribedDisplacements { get; }
        /// <summary>
        /// Per-element thermal loads (local)
        /// </summary>
        public IReadOnlyList<TemperatureLoad> TemperatureLoads { get; }
        /// <summary>
        /// Concentrated point loads applied along the local x-axis of frame elements.
        /// </summary>
        public IReadOnlyList<InternalConcentratedLoad> InternalConcentratedLoads { get; }

        public LoadCase(Vec3Float gravity,
            IReadOnlyList<NodeLoad> nodeLoads,
            IReadOnlyList<UniformLoad> uniformLoads,
            IReadOnlyList<TrapLoad> trapLoads,
            IReadOnlyList<PrescribedDisplacement> prescribedDisplacements,
            IReadOnlyList<TemperatureLoad> temperatureLoads = null,
            IReadOnlyList<InternalConcentratedLoad> internalConcentratedLoads = null)
        {
            Gravity = gravity;
            NodeLoads = nodeLoads;
            UniformLoads = uniformLoads;
            TrapLoads = trapLoads;
            PrescribedDisplacements = prescribedDisplacements;
            TemperatureLoads = temperatureLoads ?? new List<TemperatureLoad>();
            InternalConcentratedLoads = internalConcentratedLoads ?? new List<InternalConcentratedLoad>();
        }

        public static LoadCase Parse(string inputGravityString, List<NodeLoad> nodeLoads, List<UniformLoad> uniformLoads, List<TrapLoad> trapLoads, List<PrescribedDisplacement> prescribedDisplacements, List<TemperatureLoad> temperatureLoads = null, List<InternalConcentratedLoad> internalConcentratedLoads = null)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputGravityString, @"\s{1,}");
            return new LoadCase(
                new Vec3Float(float.Parse(data[0]), float.Parse(data[1]), float.Parse(data[2])),
                nodeLoads,
                uniformLoads,
                trapLoads,
                prescribedDisplacements,
                temperatureLoads,
                internalConcentratedLoads
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