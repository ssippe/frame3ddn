using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses the upstream frame3dd CSV input format into <see cref="Input"/>.
    /// The CSV layout matches the <c>examples/exX.csv</c> files in the upstream repo.
    /// </summary>
    public static class CsvInputParser
    {
        public static Input Parse(StreamReader sr) => ParseLines(GetNoCommentInputCsv(sr));

        /// <summary>
        /// Shared parse body — once the input has been preprocessed to one-record-per-line
        /// whitespace-delimited form, both <c>.csv</c> and <c>.3dd</c> inputs follow the same
        /// section sequence.
        /// </summary>
        internal static Input ParseLines(List<string> noComentInput)
        {
            List<Node> nodes = new List<Node>();
            List<FrameElement> frameElements = new List<FrameElement>();
            List<ReactionInput> reactionInputs = new List<ReactionInput>();
            List<LoadCase> loadCases = new List<LoadCase>();
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
                List<InternalConcentratedLoad> internalConcentratedLoads = new List<InternalConcentratedLoad>();
                for (int j = currentLine; currentLine < j + internalConcentratedLoadNum; currentLine++)
                {
                    internalConcentratedLoads.Add(InternalConcentratedLoad.Parse(noComentInput[currentLine]));
                }

                int temperatureLoadNum = int.Parse(noComentInput[currentLine++]);
                List<TemperatureLoad> temperatureLoads = new List<TemperatureLoad>();
                for (int j = currentLine; currentLine < j + temperatureLoadNum; currentLine++)
                {
                    temperatureLoads.Add(TemperatureLoad.Parse(noComentInput[currentLine]));
                }

                int prescribedDisplacementNum = int.Parse(noComentInput[currentLine++]);
                List<PrescribedDisplacement> prescribedDisplacements = new List<PrescribedDisplacement>();
                for (int j = currentLine; currentLine < j + prescribedDisplacementNum; currentLine++)
                {
                    prescribedDisplacements.Add(PrescribedDisplacement.Parse(noComentInput[currentLine]));
                }

                LoadCase loadCase = LoadCase.Parse(loadCaseGravityString, nodeLoads, uniformLoads, trapLoads, prescribedDisplacements, temperatureLoads, internalConcentratedLoads);
                loadCases.Add(loadCase);
            }

            DynamicAnalysisInput dynamicAnalysis = ParseDynamicAnalysis(noComentInput, currentLine);

            return new Input(title, nodes, frameElements, reactionInputs, loadCases, includeShearDeformation, includeGeometricStiffness,
                exaggerateMeshDeformations, zoomScale, xAxisIncrementForInternalForces, dynamicAnalysis);
        }

        /// <summary>
        /// Parses the trailing modal-analysis + matrix-condensation section. Mirrors upstream
        /// <c>read_mass_data</c> + <c>read_condensation_data</c> in <c>frame3dd_io.c</c>.
        /// Reads tokens across line boundaries (matching fscanf semantics) so that records that
        /// span multiple CSV rows — e.g. animated-mode lists — parse correctly.
        /// </summary>
        /// <remarks>
        /// Falls back to <see cref="DynamicAnalysisInput.None"/> if the section is missing or
        /// truncated. Any malformed mid-record content surfaces as a <see cref="FormatException"/>
        /// rather than being silently swallowed — the upstream files in the repo all parse
        /// cleanly, so a parse failure indicates a genuine problem worth reporting.
        /// </remarks>
        private static DynamicAnalysisInput ParseDynamicAnalysis(List<string> lines, int startLine)
        {
            TokenReader tokens = new TokenReader(lines, startLine);

            // No dynamic section, or modes count is zero: nothing to do.
            if (!tokens.TryReadInt(out int modesCount) || modesCount < 1)
            {
                return DynamicAnalysisInput.None;
            }

            int method = tokens.ReadInt();
            int massType = tokens.ReadInt();
            double tolerance = tokens.ReadDouble();
            double shift = tokens.ReadDouble();
            double exagg = tokens.ReadDouble();

            int nI = tokens.ReadInt();
            List<NodeInertia> extraInertia = new List<NodeInertia>(nI);
            for (int i = 0; i < nI; i++)
            {
                int nodeIdx = tokens.ReadInt() - 1;
                double mass = tokens.ReadDouble();
                double ixx = tokens.ReadDouble();
                double iyy = tokens.ReadDouble();
                double izz = tokens.ReadDouble();
                extraInertia.Add(new NodeInertia(nodeIdx, mass, ixx, iyy, izz));
            }

            int nX = tokens.ReadInt();
            List<ElementMass> extraMass = new List<ElementMass>(nX);
            for (int i = 0; i < nX; i++)
            {
                int elemIdx = tokens.ReadInt() - 1;
                double mass = tokens.ReadDouble();
                extraMass.Add(new ElementMass(elemIdx, mass));
            }

            int nA = tokens.ReadInt();
            List<int> animatedModes = new List<int>(nA);
            for (int i = 0; i < nA; i++)
            {
                animatedModes.Add(tokens.ReadInt());
            }

            double panRate = tokens.ReadDouble();

            int condensationMethod = 0;
            List<CondensedNode> condensedNodes = new List<CondensedNode>();
            List<int> condensedModes = new List<int>();

            // Condensation block is optional — many files end after the pan rate.
            if (tokens.TryReadInt(out int cm))
            {
                condensationMethod = cm;
                if (cm > 0)
                {
                    int nC = tokens.ReadInt();
                    int totalDofs = 0;
                    for (int i = 0; i < nC; i++)
                    {
                        int nodeIdx = tokens.ReadInt() - 1;
                        bool[] dof = new bool[6];
                        for (int j = 0; j < 6; j++)
                        {
                            int flag = tokens.ReadInt();
                            dof[j] = flag != 0;
                            if (dof[j]) totalDofs++;
                        }
                        condensedNodes.Add(new CondensedNode(nodeIdx, dof));
                    }
                    // Mode list (only meaningful for Cmethod == 3 / dynamic) — but upstream
                    // always reads it when Cdof > 0. Tolerate truncation.
                    for (int i = 0; i < totalDofs; i++)
                    {
                        if (!tokens.TryReadInt(out int m)) break;
                        condensedModes.Add(m);
                    }
                }
            }

            return new DynamicAnalysisInput(modesCount, method, massType, tolerance, shift, exagg,
                extraInertia, extraMass, animatedModes, panRate,
                condensationMethod, condensedNodes, condensedModes);
        }

        /// <summary>
        /// Reads whitespace-separated tokens across multiple preprocessed input lines.
        /// Matches the line-agnostic semantics of upstream's <c>fscanf</c> calls.
        /// </summary>
        private sealed class TokenReader
        {
            private readonly List<string> _lines;
            private int _lineIdx;
            private string[] _currentTokens;
            private int _tokenIdx;

            public TokenReader(List<string> lines, int startLineIdx)
            {
                _lines = lines;
                _lineIdx = startLineIdx;
                _currentTokens = Array.Empty<string>();
                _tokenIdx = 0;
            }

            public bool TryReadToken(out string token)
            {
                while (_tokenIdx >= _currentTokens.Length)
                {
                    if (_lineIdx >= _lines.Count)
                    {
                        token = null;
                        return false;
                    }
                    _currentTokens = _lines[_lineIdx++].Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    _tokenIdx = 0;
                }
                token = _currentTokens[_tokenIdx++];
                return true;
            }

            public bool TryReadInt(out int value)
            {
                if (TryReadToken(out string t) && int.TryParse(t, NumberStyles.Integer, CultureInfo.InvariantCulture, out value))
                    return true;
                value = 0;
                return false;
            }

            public int ReadInt()
            {
                if (!TryReadToken(out string t))
                    throw new FormatException("Unexpected end of input in dynamic-analysis section");
                if (!int.TryParse(t, NumberStyles.Integer, CultureInfo.InvariantCulture, out int v))
                    throw new FormatException($"Expected integer in dynamic-analysis section, got '{t}'");
                return v;
            }

            public double ReadDouble()
            {
                if (!TryReadToken(out string t))
                    throw new FormatException("Unexpected end of input in dynamic-analysis section");
                if (!double.TryParse(t, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
                    throw new FormatException($"Expected number in dynamic-analysis section, got '{t}'");
                return v;
            }
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
    }
}
