using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.IO;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses the upstream frame3dd CSV input format into <see cref="Input"/>.
    /// The CSV layout matches the <c>examples/exX.csv</c> files in the upstream repo.
    /// </summary>
    public static class CsvParser
    {
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

            // Dynamic-analysis section: not used by the static solver, but consume the rows so
            // parsing reaches end-of-file. Layout (frame3dd_io.c):
            //   nM (#modes), method, mass-type, tolerance, shift, exaggerate    [nM>0 → 5 params]
            //   nI (#nodes with extra inertia)   followed by nI rows
            //   nX (#elements with extra mass)   followed by nX rows
            //   nA (#modes to animate)           followed by 1 row of mode IDs (only if nA>0)
            //   pan rate
            //   nC (#condensed nodes)            followed by nC rows
            // Any of these may be missing if the file ends early; ArgumentOutOfRangeException
            // and FormatException are both treated as end-of-data.
            try
            {
                int nM = int.Parse(noComentInput[currentLine++]);
                if (nM > 0) currentLine += 5;       // method, mass-type, tolerance, shift, exaggerate

                int nI = int.Parse(noComentInput[currentLine++]);
                currentLine += nI;                  // extra node inertia rows

                int nX = int.Parse(noComentInput[currentLine++]);
                currentLine += nX;                  // extra element mass rows

                int nA = int.Parse(noComentInput[currentLine++]);
                if (nA > 0) currentLine += 1;       // mode list line
                currentLine += 1;                   // pan rate

                int nC = int.Parse(noComentInput[currentLine++]);
                currentLine += nC;                  // condensed-node rows
            }
            catch (ArgumentOutOfRangeException) { /* file ended early — ignore */ }
            catch (FormatException)              { /* hit a non-numeric row — ignore */ }

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
    }
}
