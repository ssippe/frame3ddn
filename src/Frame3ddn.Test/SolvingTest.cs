using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using Xunit;

namespace Frame3ddn.Test
{
    public class SolvingTest
    {
        [Fact]
        public void GetInputFromFile()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string inputPath = Directory.GetDirectories(testDataPath, "Input")[0];
            StreamReader sr = new StreamReader(Directory.GetFiles(inputPath, "B.csv")[0]);
            Input input = Input.Parse(sr);
            Assert.Equal(18, input.Nodes.Count);
            Assert.Equal(17, input.ReactionInputs[3].Number);
            Assert.Equal(10, input.FrameElements[3].Ax);
            Assert.True(input.LoadCases[0].UniformLoads[1].Load.Y + 1.1 < 0.0001);
        }

        [Fact]
        public void Run()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string inputPath = Directory.GetDirectories(testDataPath, "Input")[0];
            string outputPath = Directory.GetDirectories(testDataPath, "Output")[0];
            StreamReader sr = new StreamReader(Directory.GetFiles(inputPath, "B.csv")[0]);
            Input input = Input.Parse(sr);
            Solver solver = new Solver();
            Output output = solver.Solve(input);
            File.WriteAllText(Directory.GetFiles(outputPath, "B.csv")[0], output.TextOutput);
        }

        /// <summary>
        /// Put input file in Input folder and output file from C Program in output folder.
        /// </summary>
        [Fact]
        public void BatchCompare()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string inputPath = Directory.GetDirectories(testDataPath, "Input")[0];
            string outputPath = Directory.GetDirectories(testDataPath, "Output")[0];
            string outputFromCProgramPath = Directory.GetDirectories(testDataPath, "OutputFromCProgram")[0];

            DirectoryInfo inputDirectoryInfo = new DirectoryInfo(inputPath);
            FileInfo[] Files = inputDirectoryInfo.GetFiles("*.csv"); 
            List<string> fileNameList = new List<string>();
            foreach (FileInfo file in Files)
            {
                string filename = file.Name.Substring(0, file.Name.Length - 4);
                fileNameList.Add(filename);
            }

            for(int i = 0; i < fileNameList.Count; i ++)
            {
                string fileName = fileNameList[i];
                Compare(
                    Directory.GetFiles(inputPath, fileName + ".csv")[0],
                    Directory.GetFiles(outputFromCProgramPath, fileName +".txt")[0]
                );
                System.Diagnostics.Debug.WriteLine("finished comparing file: " + i + " " + fileName);
            }
        }

        private void Compare(string inputFilePath, string outputFilePath)
        {
            StreamReader sr = new StreamReader(inputFilePath);
            Input input = Input.Parse(sr);
            Solver solver = new Solver();
            Output output = solver.Solve(input);

            string file = File.ReadAllText(outputFilePath);
            var result = ParseLines(file);

            for (int i = 0; i < output.LoadCaseOutputs.Count; i++)
            {
                LoadCaseOutput loadCaseOutput = output.LoadCaseOutputs[i];
                OutputLine loadCaseOutputToCompare = result[i];
                for (int j = 0; j < loadCaseOutput.NodeDisplacements.Count; j++)
                {
                    NodeDisplacement nodeDisplacement = loadCaseOutput.NodeDisplacements[j];
                    NodeDeflectionLine nodeDisplacementToCompare = loadCaseOutputToCompare.NodeDeflectionLines[j];
                    GetCompareResult(nodeDisplacement.NodeIdx, nodeDisplacementToCompare.NodeIdx);
                    GetCompareResult(nodeDisplacement.Displacement.X, nodeDisplacementToCompare.DisplacementX);
                    GetCompareResult(nodeDisplacement.Displacement.Y, nodeDisplacementToCompare.DisplacementY);
                    GetCompareResult(nodeDisplacement.Displacement.Z, nodeDisplacementToCompare.Fz);
                    GetCompareResult(nodeDisplacement.Rotation.X, nodeDisplacementToCompare.Mxx);
                    GetCompareResult(nodeDisplacement.Rotation.Y, nodeDisplacementToCompare.Myy);
                    GetCompareResult(nodeDisplacement.Rotation.Z, nodeDisplacementToCompare.Mzz);
                }

                for (int j = 0; j < loadCaseOutput.FrameElementEndForces.Count; j++)
                {
                    FrameElementEndForce frameElementEndForce = loadCaseOutput.FrameElementEndForces[j];
                    FrameElementEndForcesLine frameElementEndForcesToCompare =
                        loadCaseOutputToCompare.FrameElementEndForcesLines[j];
                    GetCompareResult(frameElementEndForce.ElementIdx, frameElementEndForcesToCompare.Elmnt);
                    GetCompareResult(frameElementEndForce.NodeIdx, frameElementEndForcesToCompare.Node);
                    GetCompareResult(frameElementEndForce.Nx, frameElementEndForcesToCompare.Nx);
                    GetCompareResult(frameElementEndForce.Vy, frameElementEndForcesToCompare.Vy);
                    GetCompareResult(frameElementEndForce.Vz, frameElementEndForcesToCompare.Vz);
                    GetCompareResult(frameElementEndForce.Txx, frameElementEndForcesToCompare.Txx);
                    GetCompareResult(frameElementEndForce.Myy, frameElementEndForcesToCompare.Myy);
                    GetCompareResult(frameElementEndForce.Mzz, frameElementEndForcesToCompare.Mzz);
                }

                for (int j = 0; j < loadCaseOutput.ReactionOutputs.Count; j++)
                {
                    ReactionOutput reactionOutput = loadCaseOutput.ReactionOutputs[j];
                    ReactionLine reactionOutputToCompare = loadCaseOutputToCompare.ReactionLines[j];
                    GetCompareResult(reactionOutput.NodeIdx, reactionOutputToCompare.NodeIndex);
                    GetCompareResult(reactionOutput.M.X, reactionOutputToCompare.Mxx);
                    GetCompareResult(reactionOutput.M.Y, reactionOutputToCompare.Myy);
                    GetCompareResult(reactionOutput.M.Z, reactionOutputToCompare.Mzz);
                    GetCompareResult(reactionOutput.F.X, reactionOutputToCompare.Fx);
                    GetCompareResult(reactionOutput.F.Y, reactionOutputToCompare.Fy);
                    GetCompareResult(reactionOutput.F.Z, reactionOutputToCompare.Fz);
                }

                for (int j = 0; j < loadCaseOutput.PeakFrameElementInternalForces.Count; j++)
                {
                    PeakFrameElementInternalForce peakFrameElementInternalForce =
                        loadCaseOutput.PeakFrameElementInternalForces[j];
                    PeakFrameElementInternalForceLine peakFrameElementInternalForceToCompare =
                        loadCaseOutputToCompare.PeakFrameElementInternalForceLines[j];
                    GetCompareResult(peakFrameElementInternalForce.ElementIdx, peakFrameElementInternalForceToCompare.MemberIndex);
                    GetCompareResult(peakFrameElementInternalForce.Nx, peakFrameElementInternalForceToCompare.Nx);
                    GetCompareResult(peakFrameElementInternalForce.Vy, peakFrameElementInternalForceToCompare.Vy);
                    GetCompareResult(peakFrameElementInternalForce.Vz, peakFrameElementInternalForceToCompare.Vz);
                    GetCompareResult(peakFrameElementInternalForce.Txx, peakFrameElementInternalForceToCompare.Txx);
                    GetCompareResult(peakFrameElementInternalForce.Myy, peakFrameElementInternalForceToCompare.Myy);
                    GetCompareResult(peakFrameElementInternalForce.Mzz, peakFrameElementInternalForceToCompare.Mzz);
                    if (peakFrameElementInternalForce.IsMin == peakFrameElementInternalForceToCompare.IsMax)
                        throw new Exception(peakFrameElementInternalForce.IsMin + " " + !peakFrameElementInternalForceToCompare.IsMax);
                }
            }
        }

        private void GetCompareResult(double num1, double num2)
        {
            if (!CompareDouble(num1, num2))
                throw new Exception(num1 + " " + num2);
        }

        private bool CompareDouble(double num1, double num2)
        {
            if (Math.Abs(num1 - num2) < Math.Abs(num1) * 0.01 || Math.Abs(num1 - num2) < 0.01)
                return true;
            return false;
        }

        private string ReadUntil(StringReader reader, Func<string, bool> stop)
        {
            string currentLine;
            while (true)
            {
                currentLine = reader.ReadLine();
                if (currentLine == null)
                    return null;
                if (stop(currentLine))
                    return currentLine;
            }
        }

        private List<OutputLine> ParseLines(string frame3DDoutput)
        {
            
            List<OutputLine> outputLines = new List<OutputLine>();
            var reader = new StringReader(frame3DDoutput);
            while (true)
            {
                string currentLine = ReadUntil(reader, (s) => s.StartsWith("L O A D   C A S E"));
                var forceLines = new List<PeakFrameElementInternalForceLine>();
                var displacementLines = new List<NodeDeflectionLine>();
                var reactionLines = new List<ReactionLine>();
                var frameElementEndForceLines = new List<FrameElementEndForcesLine>();

                if (currentLine == null)
                    return outputLines;
                int loadCaseIdx =
                    int.Parse(Regex.Match(currentLine, @"L O A D   C A S E\s*(\d+)\s*O F\s*(\d+)").Groups[1].Value) - 1;
                
                // node displacements
                if (ReadUntil(reader,
                    s => s.StartsWith("N O D E   D I S P L A C E M E N T S  					(global)")) == null)
                    break;
                if (reader.ReadLine() == null) //skip headers
                    break;
                while (true)
                {
                    var resultLine = NodeDeflectionLine.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    displacementLines.Add(resultLine);
                }
                
                // frame element end forces
                if (ReadUntil(reader,
                        s => s.Contains("Elmnt")) == null)
                    break;
                while (true)
                {
                    var resultLine = FrameElementEndForcesLine.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    frameElementEndForceLines.Add(resultLine);
                }
                
                // reactions                
                if (ReadUntil(reader,
                    s => s.Contains("Node")) == null)
                    break;
                while (true)
                {
                    var resultLine = ReactionLine.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    reactionLines.Add(resultLine);
                }
                
                // internal forces
                if (ReadUntil(reader,
                    s => s.StartsWith("P E A K   F R A M E   E L E M E N T   I N T E R N A L   F O R C E S")) == null)
                    break;
                if (reader.ReadLine() == null) //skip headers
                    break;
                while (true)
                {
                    var resultLine = PeakFrameElementInternalForceLine.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    forceLines.Add(resultLine);
                }
                outputLines.Add(new OutputLine(frameElementEndForceLines, displacementLines, reactionLines, forceLines));
            }
            return outputLines;
        }
    }
}
