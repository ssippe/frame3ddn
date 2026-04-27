using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses Microstran .arc files into <see cref="Input"/>. Format reference:
    /// upstream-frame3dd/src/microstran/FORMAT.txt and the test fixtures in
    /// upstream-frame3dd/test (lateral-column.arc, truss.arc, etc.).
    /// </summary>
    public static class ArcParser
    {
        public static Input Parse(StreamReader sr)
        {
            string title = null;

            List<int> nodeIds = new List<int>();
            Dictionary<int, int> nodeIdToIdx = new Dictionary<int, int>();
            List<Node> nodes = new List<Node>();
            List<ReactionInput> reactionInputs = new List<ReactionInput>();
            List<MemberRow> members = new List<MemberRow>();
            Dictionary<int, PropRow> props = new Dictionary<int, PropRow>();
            Dictionary<int, MatlRow> matls = new Dictionary<int, MatlRow>();
            List<LoadCaseAccum> loadCases = new List<LoadCaseAccum>();
            LoadCaseAccum currentCase = null;
            bool endSeen = false;

            string raw;
            while ((raw = sr.ReadLine()) != null)
            {
                string line = raw.Trim();
                if (line.Length == 0) continue;
                if (line.StartsWith("*"))
                {
                    if (title == null)
                    {
                        string text = line.TrimStart('*').Trim();
                        if (text.Length > 0) title = text;
                    }
                    continue;
                }
                if (line.StartsWith("$")) continue;
                if (endSeen) continue;

                string[] tok = SplitWs(line);
                switch (tok[0])
                {
                    case "VERS":
                    case "TYPE":
                    case "VERT":
                    case "UNIT":
                        break;

                    case "END":
                        endSeen = true;
                        break;

                    case "NODE":
                        {
                            int id = int.Parse(tok[1], CultureInfo.InvariantCulture);
                            float x = float.Parse(tok[2], CultureInfo.InvariantCulture);
                            float y = float.Parse(tok[3], CultureInfo.InvariantCulture);
                            float z = float.Parse(tok[4], CultureInfo.InvariantCulture);
                            string flags = tok.Length > 5 ? tok[5] : "000000";

                            int idx = nodes.Count;
                            nodeIdToIdx[id] = idx;
                            nodeIds.Add(id);
                            nodes.Add(new Node(new Vec3Float(x, y, z), 0f));

                            if (flags.Length >= 6 && flags.IndexOf('1') >= 0)
                            {
                                reactionInputs.Add(new ReactionInput(idx,
                                    new Vec3Float(flags[0] - '0', flags[1] - '0', flags[2] - '0'),
                                    new Vec3Float(flags[3] - '0', flags[4] - '0', flags[5] - '0')));
                            }
                            break;
                        }

                    case "MEMB":
                        {
                            int memberId = int.Parse(tok[1], CultureInfo.InvariantCulture);
                            int n1Id = int.Parse(tok[2], CultureInfo.InvariantCulture);
                            int n2Id = int.Parse(tok[3], CultureInfo.InvariantCulture);
                            string axis = tok[4];
                            int propId = int.Parse(tok[5], CultureInfo.InvariantCulture);
                            int matlId = int.Parse(tok[6], CultureInfo.InvariantCulture);
                            members.Add(new MemberRow(memberId, n1Id, n2Id, axis, propId, matlId));
                            break;
                        }

                    case "PROP":
                        {
                            int propId = int.Parse(tok[1], CultureInfo.InvariantCulture);
                            string propLine = ReadDataLine(sr)
                                ?? throw new Exception($"PROP {propId} missing property values line");
                            string[] pTok = SplitWs(propLine);
                            props[propId] = new PropRow(
                                propId,
                                float.Parse(pTok[0], CultureInfo.InvariantCulture),
                                float.Parse(pTok[1], CultureInfo.InvariantCulture),
                                float.Parse(pTok[2], CultureInfo.InvariantCulture),
                                float.Parse(pTok[3], CultureInfo.InvariantCulture),
                                float.Parse(pTok[4], CultureInfo.InvariantCulture),
                                float.Parse(pTok[5], CultureInfo.InvariantCulture));
                            break;
                        }

                    case "MATL":
                        {
                            int matlId = int.Parse(tok[1], CultureInfo.InvariantCulture);
                            float E = float.Parse(tok[2], CultureInfo.InvariantCulture);
                            float nu = float.Parse(tok[3], CultureInfo.InvariantCulture);
                            float density = float.Parse(tok[4], CultureInfo.InvariantCulture);
                            float alpha = tok.Length > 5
                                ? float.Parse(tok[5], CultureInfo.InvariantCulture)
                                : 0f;
                            matls[matlId] = new MatlRow(matlId, E, nu, density, alpha);
                            break;
                        }

                    case "CASE":
                        currentCase = new LoadCaseAccum();
                        loadCases.Add(currentCase);
                        break;

                    case "NDLD":
                        {
                            if (currentCase == null)
                                throw new Exception("NDLD encountered before any CASE statement");
                            int nodeArcId = int.Parse(tok[1], CultureInfo.InvariantCulture);
                            if (!nodeIdToIdx.TryGetValue(nodeArcId, out int nodeIdx))
                                throw new Exception($"NDLD references unknown NODE {nodeArcId}");
                            currentCase.NodeLoads.Add(new NodeLoad(
                                nodeIdx,
                                new Vec3(
                                    double.Parse(tok[2], CultureInfo.InvariantCulture),
                                    double.Parse(tok[3], CultureInfo.InvariantCulture),
                                    double.Parse(tok[4], CultureInfo.InvariantCulture)),
                                new Vec3(
                                    double.Parse(tok[5], CultureInfo.InvariantCulture),
                                    double.Parse(tok[6], CultureInfo.InvariantCulture),
                                    double.Parse(tok[7], CultureInfo.InvariantCulture))));
                            break;
                        }

                    case "GRAV":
                        {
                            if (currentCase == null)
                                throw new Exception("GRAV encountered before any CASE statement");
                            currentCase.Gravity = new Vec3Float(
                                float.Parse(tok[1], CultureInfo.InvariantCulture),
                                float.Parse(tok[2], CultureInfo.InvariantCulture),
                                float.Parse(tok[3], CultureInfo.InvariantCulture));
                            break;
                        }

                    case "COMB":
                        // Load case combinations are not yet supported; the case still exists but stays empty.
                        break;

                    default:
                        // Forward-compat: ignore unknown directives (MLDx, design data, etc.)
                        break;
                }
            }

            List<FrameElement> frameElements = new List<FrameElement>();
            foreach (MemberRow m in members)
            {
                if (!props.TryGetValue(m.PropId, out PropRow p))
                    throw new Exception($"MEMB {m.Id} references unknown PROP {m.PropId}");
                if (!matls.TryGetValue(m.MatlId, out MatlRow ma))
                    throw new Exception($"MEMB {m.Id} references unknown MATL {m.MatlId}");
                if (!nodeIdToIdx.TryGetValue(m.N1Id, out int n1))
                    throw new Exception($"MEMB {m.Id} references unknown NODE {m.N1Id}");
                if (!nodeIdToIdx.TryGetValue(m.N2Id, out int n2))
                    throw new Exception($"MEMB {m.Id} references unknown NODE {m.N2Id}");

                float G = ma.E / (2f * (1f + ma.Nu));
                float roll = ComputeRoll(nodes[n1].Position, nodes[n2].Position, m.Axis, nodes, nodeIdToIdx);
                frameElements.Add(new FrameElement(
                    n1, n2,
                    p.Ax, p.Ay, p.Az,
                    p.J, p.Iy, p.Iz,
                    ma.E, G,
                    roll,
                    ma.Density));
            }

            List<LoadCase> finalLoadCases = loadCases.Select(lc => new LoadCase(
                lc.Gravity,
                lc.NodeLoads,
                new List<UniformLoad>(),
                new List<TrapLoad>(),
                new List<PrescribedDisplacement>())).ToList();

            return new Input(
                title ?? "",
                nodes,
                frameElements,
                reactionInputs,
                finalLoadCases,
                includeShearDeformation: false,
                includeGeometricStiffness: false,
                exaggerateMeshDeformations: 0f,
                zoomScale: 0f,
                xAxisIncrementForInternalForces: -1f);   // skip internal-force sampling; .arc has no analogue
        }

        /// <summary>
        /// Convert a Microstran MEMB axis-alignment flag (X, Y, Z, -X, -Y, -Z, or a node ID)
        /// into a frame3dd roll angle (radians, rotation about local x). Microstran's flag
        /// names the global axis (or third-node direction) the local Y axis aligns with;
        /// frame3dd's coordinate transform places local Y at a default orientation when
        /// roll = 0, so we compute the signed angle around local x that takes the default
        /// local Y to Microstran's desired local Y.
        /// </summary>
        private static float ComputeRoll(Vec3Float p1, Vec3Float p2, string axisFlag,
            List<Node> nodes, Dictionary<int, int> nodeIdToIdx)
        {
            (double dx, double dy, double dz) = (p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z);
            double L = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            if (L == 0) return 0f;
            double Cx = dx / L, Cy = dy / L, Cz = dz / L;

            // Desired local-y direction in global coordinates from the axis flag.
            (double yx, double yy, double yz) desired;
            switch (axisFlag)
            {
                case "X":  desired = (1, 0, 0); break;
                case "-X": desired = (-1, 0, 0); break;
                case "Y":  desired = (0, 1, 0); break;
                case "-Y": desired = (0, -1, 0); break;
                case "Z":  desired = (0, 0, 1); break;
                case "-Z": desired = (0, 0, -1); break;
                default:   desired = ResolveThirdNode(axisFlag, p1, p2, nodes, nodeIdToIdx); break;
            }

            // Project desired_y onto the plane perpendicular to local x, then re-normalize.
            double dot = desired.yx * Cx + desired.yy * Cy + desired.yz * Cz;
            double dxC = desired.yx - dot * Cx;
            double dyC = desired.yy - dot * Cy;
            double dzC = desired.yz - dot * Cz;
            double dLen = Math.Sqrt(dxC * dxC + dyC * dyC + dzC * dzC);
            if (dLen < 1e-9) return 0f;     // axis flag was parallel to local x -- ignore
            dxC /= dLen; dyC /= dLen; dzC /= dLen;

            // frame3dd's default local-y at p = 0 (mirrors Coordtrans.coordTrans).
            double defx, defy, defz;
            if (Math.Abs(Math.Abs(Cz) - 1.0) < 1e-12)
            {
                defx = 0; defy = 1; defz = 0;
            }
            else
            {
                double den = Math.Sqrt(1.0 - Cz * Cz);
                defx = -Cy / den; defy = Cx / den; defz = 0;
            }

            // Signed angle from default-y to desired-y, measured around local-x.
            double cos = defx * dxC + defy * dyC + defz * dzC;
            double crossX = defy * dzC - defz * dyC;
            double crossY = defz * dxC - defx * dzC;
            double crossZ = defx * dyC - defy * dxC;
            double sin = crossX * Cx + crossY * Cy + crossZ * Cz;
            return (float)Math.Atan2(sin, cos);
        }

        private static (double, double, double) ResolveThirdNode(string flag, Vec3Float p1, Vec3Float p2,
            List<Node> nodes, Dictionary<int, int> nodeIdToIdx)
        {
            if (!int.TryParse(flag, NumberStyles.Integer, CultureInfo.InvariantCulture, out int id))
                return (0, 0, 1);   // unknown -- fall back to +Z; the projection step will sort out degeneracy
            if (!nodeIdToIdx.TryGetValue(id, out int idx)) return (0, 0, 1);
            Vec3Float third = nodes[idx].Position;
            double mx = 0.5 * (p1.X + p2.X);
            double my = 0.5 * (p1.Y + p2.Y);
            double mz = 0.5 * (p1.Z + p2.Z);
            return (third.X - mx, third.Y - my, third.Z - mz);
        }

        private static string ReadDataLine(StreamReader sr)
        {
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.Length == 0) continue;
                if (line.StartsWith("*")) continue;
                if (line.StartsWith("$")) continue;
                return line;
            }
            return null;
        }

        private static string[] SplitWs(string line) =>
            line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);

        private sealed class MemberRow
        {
            public int Id { get; }
            public int N1Id { get; }
            public int N2Id { get; }
            public string Axis { get; }
            public int PropId { get; }
            public int MatlId { get; }

            public MemberRow(int id, int n1Id, int n2Id, string axis, int propId, int matlId)
            {
                Id = id; N1Id = n1Id; N2Id = n2Id; Axis = axis; PropId = propId; MatlId = matlId;
            }
        }

        private sealed class PropRow
        {
            public int Id { get; }
            public float Ax { get; }
            public float Ay { get; }
            public float Az { get; }
            public float J { get; }
            public float Iy { get; }
            public float Iz { get; }

            public PropRow(int id, float ax, float ay, float az, float j, float iy, float iz)
            {
                Id = id; Ax = ax; Ay = ay; Az = az; J = j; Iy = iy; Iz = iz;
            }
        }

        private sealed class MatlRow
        {
            public int Id { get; }
            public float E { get; }
            public float Nu { get; }
            public float Density { get; }
            public float Alpha { get; }

            public MatlRow(int id, float e, float nu, float density, float alpha)
            {
                Id = id; E = e; Nu = nu; Density = density; Alpha = alpha;
            }
        }

        private sealed class LoadCaseAccum
        {
            public Vec3Float Gravity { get; set; } = new Vec3Float(0f, 0f, 0f);
            public List<NodeLoad> NodeLoads { get; } = new List<NodeLoad>();
        }
    }
}
