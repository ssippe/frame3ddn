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

        [Fact]
        public void BatchCompareTwoFiles()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string ComparingOutputFiles = Directory.GetDirectories(testDataPath, "ComparingOutputFiles")[0];
            string outputAPath = Directory.GetDirectories(ComparingOutputFiles, "A")[0];
            string outputBPath = Directory.GetDirectories(ComparingOutputFiles, "B")[0];

            DirectoryInfo inputDirectoryInfo = new DirectoryInfo(outputAPath);
            FileInfo[] Files = inputDirectoryInfo.GetFiles("*.txt");
            List<string> fileNameList = new List<string>();
            foreach (FileInfo file in Files)
            {
                string filename = file.Name;
                fileNameList.Add(filename);
            }

            for (int i = 0; i < fileNameList.Count; i++)
            {
                string fileName = fileNameList[i];
                CompareTwoFiles(
                    Directory.GetFiles(outputAPath, fileName)[0],
                    Directory.GetFiles(outputBPath, fileName)[0]
                );
                System.Diagnostics.Debug.WriteLine("finished comparing file: " + i + " " + fileName);
            }
        }

        private void CompareTwoFiles(string outputFilePath1, string outputFilePath2)
        {
            string file1 = File.ReadAllText(outputFilePath1);
            string file2 = File.ReadAllText(outputFilePath2);
            var result1 = ParseLines(file1);
            var result2 = ParseLines(file2);

            for (int i = 0; i < result1.Count; i++)
            {
                OutputLine loadCaseOutput1 = result1[i];
                OutputLine loadCaseOutput2 = result2[i];
                for (int j = 0; j < loadCaseOutput1.NodeDeflectionLines.Count; j++)
                {
                    NodeDeflectionLine nodeDisplacement1 = loadCaseOutput1.NodeDeflectionLines[j];
                    NodeDeflectionLine nodeDisplacement2 = loadCaseOutput2.NodeDeflectionLines[j];
                    GetCompareResult(nodeDisplacement1.NodeIdx, nodeDisplacement2.NodeIdx);
                    GetCompareResult(nodeDisplacement1.DisplacementX, nodeDisplacement2.DisplacementX);
                    GetCompareResult(nodeDisplacement1.DisplacementY, nodeDisplacement2.DisplacementY);
                    GetCompareResult(nodeDisplacement1.Fz, nodeDisplacement2.Fz);
                    GetCompareResult(nodeDisplacement1.Mxx, nodeDisplacement2.Mxx);
                    GetCompareResult(nodeDisplacement1.Myy, nodeDisplacement2.Myy);
                    GetCompareResult(nodeDisplacement1.Mzz, nodeDisplacement2.Mzz);
                }

                for (int j = 0; j < loadCaseOutput1.FrameElementEndForcesLines.Count; j++)
                {
                    FrameElementEndForcesLine frameElementEndForces1 = loadCaseOutput1.FrameElementEndForcesLines[j];
                    FrameElementEndForcesLine frameElementEndForces2 = loadCaseOutput2.FrameElementEndForcesLines[j];
                    GetCompareResult(frameElementEndForces1.Elmnt, frameElementEndForces2.Elmnt);
                    GetCompareResult(frameElementEndForces1.Node, frameElementEndForces2.Node);
                    GetCompareResult(frameElementEndForces1.Nx, frameElementEndForces2.Nx);
                    GetCompareResult(frameElementEndForces1.Vy, frameElementEndForces2.Vy);
                    GetCompareResult(frameElementEndForces1.Vz, frameElementEndForces2.Vz);
                    GetCompareResult(frameElementEndForces1.Txx, frameElementEndForces2.Txx);
                    GetCompareResult(frameElementEndForces1.Myy, frameElementEndForces2.Myy);
                    GetCompareResult(frameElementEndForces1.Mzz / 100000, frameElementEndForces2.Mzz / 100000);
                }

                for (int j = 0; j < loadCaseOutput1.ReactionLines.Count; j++)
                {
                    ReactionLine reactionOutput1 = loadCaseOutput1.ReactionLines[j];
                    ReactionLine reactionOutput2 = loadCaseOutput2.ReactionLines[j];
                    GetCompareResult(reactionOutput1.NodeIndex, reactionOutput2.NodeIndex);
                    GetCompareResult(reactionOutput1.Mxx, reactionOutput2.Mxx);
                    GetCompareResult(reactionOutput1.Myy, reactionOutput2.Myy);
                    GetCompareResult(reactionOutput1.Mzz, reactionOutput2.Mzz);
                    GetCompareResult(reactionOutput1.Fx, reactionOutput2.Fx);
                    GetCompareResult(reactionOutput1.Fy, reactionOutput2.Fy);
                    GetCompareResult(reactionOutput1.Fz, reactionOutput2.Fz);
                }

                for (int j = 0; j < loadCaseOutput1.PeakFrameElementInternalForceLines.Count; j++)
                {
                    PeakFrameElementInternalForceLine peakFrameElementInternalForce1 =
                        loadCaseOutput1.PeakFrameElementInternalForceLines[j];
                    PeakFrameElementInternalForceLine peakFrameElementInternalForce2 =
                        loadCaseOutput2.PeakFrameElementInternalForceLines[j];
                    GetCompareResult(peakFrameElementInternalForce1.MemberIndex, peakFrameElementInternalForce2.MemberIndex);
                    GetCompareResult(peakFrameElementInternalForce1.Nx, peakFrameElementInternalForce2.Nx);
                    GetCompareResult(peakFrameElementInternalForce1.Vy, peakFrameElementInternalForce2.Vy);
                    GetCompareResult(peakFrameElementInternalForce1.Vz, peakFrameElementInternalForce2.Vz);
                    GetCompareResult(peakFrameElementInternalForce1.Txx, peakFrameElementInternalForce2.Txx);
                    GetCompareResult(peakFrameElementInternalForce1.Myy, peakFrameElementInternalForce2.Myy);
                    GetCompareResult(peakFrameElementInternalForce1.Mzz/100000, peakFrameElementInternalForce2.Mzz/100000);
                    if (peakFrameElementInternalForce1.IsMax != peakFrameElementInternalForce2.IsMax)
                        throw new Exception(peakFrameElementInternalForce1.IsMax + " " + peakFrameElementInternalForce2.IsMax);
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
