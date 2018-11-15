using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace Frame3ddn
{
    public struct Vec3
    {
        public double X;
        public double Y;
        public double Z;

        public Vec3(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }
    }

    public struct Vec3Float
    {
        public float X;
        public float Y;
        public float Z;

        public Vec3Float(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }
    }

    public class Node
    {
        public Node(Vec3Float position, float radius)
        {
            Position = position;
            Radius = radius;
        }

        public Vec3Float Position { get; }
        public float Radius { get; }

        public static Node Parse(string inputString)
        {
            float[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}").Select(float.Parse).ToArray();
            return new Node(new Vec3Float(data[1], data[2], data[3]), data[4]);
        }
    }

    public class ReactionInput
    {
        public ReactionInput(int num, Vec3Float position, Vec3Float r)
        {
            Number = num;
            Position = position;
            R = r;
        }

        public Vec3Float Position { get; }
        public Vec3Float R { get; }
        public int Number { get; set; }//This can't be replaced by index

        public static ReactionInput Parse(string inputString)
        {
            int[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}").Select(int.Parse).ToArray();
            return new ReactionInput(data[0] - 1, new Vec3Float(data[1], data[2], data[3]), new Vec3Float(data[4], data[5], data[6]));
        }
    }

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

    public class NodeLoad
    {
        public int NodeIdx { get; }
        public Vec3 Load { get; }
        public Vec3 Moment { get; }

        public NodeLoad(int nodeIdx, Vec3 load, Vec3 moment)
        {
            NodeIdx = nodeIdx;
            Load = load;
            Moment = moment;
        }

        public static NodeLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new NodeLoad(int.Parse(data[0]) - 1, 
                new Vec3(double.Parse(data[1]), double.Parse(data[2]), double.Parse(data[3])),
                new Vec3(double.Parse(data[4]), double.Parse(data[5]), double.Parse(data[6])));
        }
    }

    public class UniformLoad
    {
        public int ElementIdx { get; }
        public Vec3Float Load { get; }

        public UniformLoad(int elementIdx, Vec3Float load)
        {
            ElementIdx = elementIdx;
            Load = load;
        }

        public static UniformLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new UniformLoad(int.Parse(data[0]) - 1,
                new Vec3Float(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3])));
        }
    }

    public class TrapLoad
    {
        public int ElementIdx { get; }
        public Vec3Float LocationStart { get; }
        public Vec3Float LocationEnd { get; }
        public Vec3Float LoadStart { get; }
        public Vec3Float LoadEnd { get; }
        public TrapLoad(int elementIdx, Vec3Float locationStart, Vec3Float locationEnd, Vec3Float loadStart, Vec3Float loadEnd)
        {
            ElementIdx = elementIdx;
            LocationStart = locationStart;
            LocationEnd = locationEnd;
            LoadStart = loadStart;
            LoadEnd = loadEnd;
        }

        public static TrapLoad Parse(string inputString)
        {
            string[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}");
            return new TrapLoad(int.Parse(data[0]) - 1,
                new Vec3Float(float.Parse(data[1]), float.Parse(data[5]), float.Parse(data[9])),
                new Vec3Float(float.Parse(data[2]), float.Parse(data[6]), float.Parse(data[10])),
                new Vec3Float(float.Parse(data[3]), float.Parse(data[7]), float.Parse(data[11])),
                new Vec3Float(float.Parse(data[4]), float.Parse(data[8]), float.Parse(data[12]))
                );
        }
    }

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

    public class Input
    {
        public string Title { get; }
        public bool IncludeShearDeformation { get; }
        public bool IncludeGeometricStiffness { get; }
        public float ExaggerateMeshDeformations { get; }
        public float ZoomScale { get; }
        public float XAxisIncrementForInternalForces { get; set; }

        public IReadOnlyList<Node> Nodes { get; }
        public IReadOnlyList<ReactionInput> ReactionInputs { get; }
        public IReadOnlyList<FrameElement> FrameElements { get; }
        public IReadOnlyList<LoadCase> LoadCases { get; }

        public Input(string title, IReadOnlyList<Node> nodes, IReadOnlyList<FrameElement> frameElements,
            IReadOnlyList<ReactionInput> reactionInputs, IReadOnlyList<LoadCase> loadCases,
            bool includeShearDeformation, bool includeGeometricStiffness, float exaggerateMeshDeformations,
            float zoomScale, float xAxisIncrementForInternalForces)
        {
            Title = title;
            Nodes = nodes;
            FrameElements = frameElements;
            ReactionInputs = reactionInputs;
            LoadCases = loadCases;
            IncludeShearDeformation = includeShearDeformation;
            IncludeGeometricStiffness = includeGeometricStiffness;
            ExaggerateMeshDeformations = exaggerateMeshDeformations;
            ZoomScale = zoomScale;
            XAxisIncrementForInternalForces = xAxisIncrementForInternalForces;
        }

        /**
         * Parse an input textfile to get Input object. The parsing logic is similar to the parsing methods in C program.
         */
        public static Input Parse(StreamReader sr)
        {
            List<Node> nodes = new List<Node>();
            List<FrameElement> frameElements = new List<FrameElement>();
            List<ReactionInput> reactionInputs = new List<ReactionInput>();
            List<LoadCase> loadCases = new List<LoadCase>();
            List<string> noComentInput = GetNoCommentInputCsv(sr);
            int currentLine = 0;

            string title = noComentInput[currentLine++];

            int nodeNum = int.Parse(noComentInput[currentLine++]);
            for (int i = currentLine; currentLine < i + nodeNum; currentLine++)
            {
                nodes.Add(Node.Parse(noComentInput[currentLine]));
            }

            int reactionNodeNum = int.Parse(noComentInput[currentLine++]);
            for (int i = currentLine; currentLine < i + reactionNodeNum; currentLine++)
            {
                reactionInputs.Add(ReactionInput.Parse(noComentInput[currentLine]));
            }

            int frameElementNum = int.Parse(noComentInput[currentLine++]);
            for (int i = currentLine; currentLine < i + frameElementNum; currentLine++)
            {
                frameElements.Add(FrameElement.Parse(noComentInput[currentLine]));
            }

            bool includeShearDeformation = int.Parse(noComentInput[currentLine++]) != 0;
            bool includeGeometricStiffness = int.Parse(noComentInput[currentLine++]) != 0;
            if (includeGeometricStiffness)
            {
                throw new Exception("Geometric stiffness is not supported.");
            }
            float exaggerateMeshDeformations = float.Parse(noComentInput[currentLine++]);
            float zoomScale = float.Parse(noComentInput[currentLine++]);
            float xAxisIncrementForInternalForces = float.Parse(noComentInput[currentLine++]);

            int LoadCaseNum = int.Parse(noComentInput[currentLine++]);
            for (int i = 0; i < LoadCaseNum; i++)
            {
                string loadCaseGravityString = noComentInput[currentLine++];
                int loadNodeNum = int.Parse(noComentInput[currentLine++]);
                List<NodeLoad> nodeLoads = new List<NodeLoad>();
                for (int j = currentLine; currentLine < j + loadNodeNum; currentLine++)
                {
                    nodeLoads.Add(NodeLoad.Parse(noComentInput[currentLine]));
                }

                int uniformLoadNum = int.Parse(noComentInput[currentLine++]);
                List<UniformLoad> uniformLoads = new List<UniformLoad>();
                for (int j = currentLine; currentLine < j + uniformLoadNum; currentLine++)
                {
                    uniformLoads.Add(UniformLoad.Parse(noComentInput[currentLine]));
                }

                int trapLoadNum = int.Parse(noComentInput[currentLine++]);
                List<TrapLoad> trapLoads = new List<TrapLoad>();
                for (int j = currentLine; currentLine < j + trapLoadNum * 3; currentLine = currentLine + 3)
                {
                    string combinedData = noComentInput[currentLine] + " " + 
                                          noComentInput[currentLine + 1] + " " +
                                          noComentInput[currentLine + 2];
                    trapLoads.Add(TrapLoad.Parse(combinedData));
                }

                int internalConcentratedLoadNum = int.Parse(noComentInput[currentLine++]);
                if (internalConcentratedLoadNum > 0)
                {
                    throw new Exception("Internal concentrated load is not supported.");
                }
                for (int j = currentLine; currentLine < j + internalConcentratedLoadNum; currentLine++)
                {
                }

                int temperatureLoadNum = int.Parse(noComentInput[currentLine++]);
                if (temperatureLoadNum > 0)
                {
                    throw new Exception("Temperature load is not supported.");
                }
                for (int j = currentLine; currentLine < j + temperatureLoadNum; currentLine++)
                {
                }

                int prescribedDisplacementNum = int.Parse(noComentInput[currentLine++]);
                if (prescribedDisplacementNum > 0)
                {
                    throw new Exception("Prescribed displacement is not supported.");
                }
                for (int j = currentLine; currentLine < j + prescribedDisplacementNum; currentLine++)
                {
                }

                LoadCase loadCase = LoadCase.Parse(loadCaseGravityString, nodeLoads, uniformLoads, trapLoads);
                loadCases.Add(loadCase);
            }

            try
            {
                if (int.Parse(noComentInput[currentLine]) > 0)
                {
                    throw new Exception("Dynamic analysis data is not supported.");
                }

                currentLine += 6;
                if (int.Parse(noComentInput[currentLine++]) > 0)
                {
                    throw new Exception("Extra node inertia data is not supported.");
                }

                if (int.Parse(noComentInput[currentLine++]) > 0)
                {
                    throw new Exception("Element with extra mass is not supported.");
                }

                if (int.Parse(noComentInput[currentLine]) > 0)
                {
                    throw new Exception("Mode shape animation data is not supported.");
                }

                currentLine += 3;
                if (int.Parse(noComentInput[currentLine]) > 0)
                {
                    throw new Exception("Condensed node is not supported.");
                }
            }
            catch (ArgumentOutOfRangeException)
            {
                //Incomplete data will be ignored
            }
            catch (Exception e)
            {
                Console.WriteLine("A");
                throw e;
            }

            return new Input(title, nodes, frameElements, reactionInputs, loadCases, includeShearDeformation, includeGeometricStiffness,
                exaggerateMeshDeformations, zoomScale, xAxisIncrementForInternalForces);
        }

        private static List<string> GetNoCommentInputCsv(StreamReader sr)
        {
            List<string> noComentInput = new List<string>();
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                line = line.Replace('\t', ' ');
                line = line.Replace("\"", "");
                line = line.Replace("\\", "");
                line = line.Replace(" ", "");
                line = line.Replace(",", " ");
                line = line.Trim();
                if (string.IsNullOrEmpty(line)) //eliminate empty line
                {
                    continue;
                }
                else if (!line.Contains("#")) //save unchanged if there's no comment
                {
                    noComentInput.Add(line.Trim());
                }
                else //check if the line only contains comment
                {
                    string[] data = line.Split('#');
                    if (string.IsNullOrEmpty(data[0])) //if it does, eliminate it
                    {
                        continue;
                    }
                    else
                    {
                        noComentInput.Add(data[0].Trim()); //otherwise, save the non-comment text only
                    }
                }
            }
            return noComentInput;
        }

        private static List<string> GetNoCommentInput(StreamReader sr)
        {
            List<string> noComentInput = new List<string>();
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                line = line.Replace('\t', ' ');
                if (string.IsNullOrEmpty(line)) //eliminate empty line
                {
                    continue;
                }
                else if (!line.Contains("#")) //save unchanged if there's no comment
                {
                    noComentInput.Add(line.Trim());
                }
                else //check if the line only contains comment
                {
                    string[] data = line.Split('#');
                    if (string.IsNullOrEmpty(data[0])) //if it does, eliminate it
                    {
                        continue;
                    }
                    else
                    {
                        noComentInput.Add(data[0].Trim()); //otherwise, save the non-comment text only
                    }
                }
            }
            return noComentInput;
        }
    }
}
