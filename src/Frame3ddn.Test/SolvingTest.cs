using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using Frame3ddn.Model;
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
            Model.Input input = Input.Parse(sr);
            Assert.Equal(18, input.Nodes.Count);
            Assert.Equal(17, input.ReactionInputs[3].NodeIdx);
            Assert.Equal(10, input.FrameElements[3].Ax);
            Assert.True(input.LoadCases[0].UniformLoads[1].Load.Y + 1.1 < 0.0001);
        }

        [Fact]
        public void Run()
        {
            string fileName = "NTI";
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string inputPath = Directory.GetDirectories(testDataPath, "Input")[0];
            string outputPath = Directory.GetDirectories(testDataPath, "Output")[0];
            StreamReader sr = new StreamReader(Directory.GetFiles(inputPath, fileName + ".csv")[0]);
            Input input = Input.Parse(sr);
            Solver solver = new Solver();
            Output output = solver.Solve(input);
            File.WriteAllText(Directory.GetFiles(outputPath, fileName + ".txt")[0], output.TextOutput);
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
            Output outputCalculated = solver.Solve(input);

            string file = File.ReadAllText(outputFilePath);
            var result = ParseLines(file);

            for (int i = 0; i < outputCalculated.LoadCaseOutputs.Count; i++)
            {
                var loadCaseOutput = outputCalculated.LoadCaseOutputs[i];
                var loadCaseOutputToCompare = result[i];
                if (!IsEqual(loadCaseOutput, loadCaseOutputToCompare))
                    throw new Exception("?");                
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
                var loadCaseOutput1 = result1[i];
                var loadCaseOutput2 = result2[i];
                if (!IsEqual(loadCaseOutput1, loadCaseOutput2))
                    throw new Exception("?");                
            }
        }

        static bool IsMinMaxForceEqual(IEnumerable<PeakFrameElementInternalForce> o1,
            IEnumerable<PeakFrameElementInternalForce> o2)
        {
            var o1Filt = o1.Where(f => f.IsMin.HasValue).ToList();
            var o2Filt = o2.Where(f => f.IsMin.HasValue).ToList();
            if (o1Filt.Count != o2Filt.Count)
                return false;
            for (int i = 0; i < o1Filt.Count; i++)
            {
                if (!IsEqual(o1Filt[i], o2Filt[i]))
                    return false;
            }

            return true;
        }

        static bool IsEqual(LoadCaseOutput o1, LoadCaseOutput o2)
        {
            if (o1.FrameElementEndForces.Count != o2.FrameElementEndForces.Count)
                return false;
            for (int i = 0; i < o1.FrameElementEndForces.Count; i++)
            {
                if (!IsEqual(o1.FrameElementEndForces[i], o2.FrameElementEndForces[i]))
                    return false;                
            }

            if (!IsMinMaxForceEqual(o1.PeakFrameElementInternalForces, o2.PeakFrameElementInternalForces))
                return false;            

            if (o1.ReactionOutputs.Count != o2.ReactionOutputs.Count)
                return false;
            for (int i = 0; i < o1.ReactionOutputs.Count; i++)
            {
                if (!IsEqual(o1.ReactionOutputs[i], o2.ReactionOutputs[i]))
                    return false;
            }

            if (o1.NodeDisplacements.Count != o2.NodeDisplacements.Count)
                return false;
            for (int i = 0; i < o1.NodeDisplacements.Count; i++)
            {
                if (!IsEqual(o1.NodeDisplacements[i], o2.NodeDisplacements[i]))
                    return false;
            }


            return true;
        }

        static bool IsEqual(NodeDisplacement nodeDisplacement1, NodeDisplacement nodeDisplacement2)
        {
            return nodeDisplacement1.NodeIdx == nodeDisplacement2.NodeIdx &&
                   CompareDouble(nodeDisplacement1.Displacement.X, nodeDisplacement2.Displacement.X) &&
                   CompareDouble(nodeDisplacement1.Displacement.Y, nodeDisplacement2.Displacement.Y) &&
                   CompareDouble(nodeDisplacement1.Displacement.Z, nodeDisplacement2.Displacement.Z) &&
                   CompareDouble(nodeDisplacement1.Rotation.X, nodeDisplacement2.Rotation.X) &&
                   CompareDouble(nodeDisplacement1.Rotation.Y, nodeDisplacement2.Rotation.Y) &&
                   CompareDouble(nodeDisplacement1.Rotation.Z, nodeDisplacement2.Rotation.Z);
        }

        static bool IsEqual(ReactionOutput o1, ReactionOutput o2)
        {
            return CompareDouble(o1.NodeIdx, o2.NodeIdx) &&
                   CompareDouble(o1.LoadcaseIdx, o2.LoadcaseIdx) &&
                   IsEqual(o1.F, o2.F) &&
                   IsEqual(o1.M, o2.M);
            
        }

        static bool IsEqual(Vec3 o1, Vec3 o2)
        {
            return CompareDouble(o1.X, o2.X) &&
                   CompareDouble(o1.Y, o2.Y) &&
                   CompareDouble(o1.Z, o2.Z);
        }

        static bool IsEqual(FrameElementEndForce o1, FrameElementEndForce o2)
        {
            return
                CompareDouble(o1.ElementIdx, o2.ElementIdx) &&
                CompareDouble(o1.NodeIdx, o2.NodeIdx) &&
                CompareDouble(o1.Nx, o2.Nx) &&
                CompareDouble(o1.Vy, o2.Vy) &&
                CompareDouble(o1.Vz, o2.Vz) &&
                CompareDouble(o1.Txx, o2.Txx) &&
                CompareDouble(o1.Myy, o2.Myy) &&
                CompareDouble(o1.Mzz / 100000, o2.Mzz / 100000);
        }

        static bool IsEqual(PeakFrameElementInternalForce o1, PeakFrameElementInternalForce o2)
        {
            return CompareDouble(o1.ElementIdx, o2.ElementIdx) &&
                   CompareDouble(o1.Nx, o2.Nx) &&
                   CompareDouble(o1.Vy, o2.Vy) &&
                   CompareDouble(o1.Vz, o2.Vz) &&
                   CompareDouble(o1.Txx, o2.Txx) &&
                   CompareDouble(o1.Myy, o2.Myy) &&
                   CompareDouble(o1.Mzz / 100000, o2.Mzz / 100000) &&
                   o1.IsMin == o2.IsMin;
        }

        private void GetCompareResult(double num1, double num2)
        {
            if (!CompareDouble(num1, num2))
                throw new Exception(num1 + " " + num2);
        }

        static bool CompareDouble(double num1, double num2)
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

        private List<LoadCaseOutput> ParseLines(string frame3DDoutput)
        {
            
            List<LoadCaseOutput> outputLines = new List<LoadCaseOutput>();
            var reader = new StringReader(frame3DDoutput);
            while (true)
            {
                string currentLine = ReadUntil(reader, (s) => s.StartsWith("L O A D   C A S E"));
                var forceLines = new List<PeakFrameElementInternalForce>();
                var displacementLines = new List<NodeDisplacement>();
                var reactionLines = new List<ReactionOutput>();
                var frameElementEndForceLines = new List<FrameElementEndForce>();

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
                    var resultLine = NodeDisplacement.FromLine(reader.ReadLine(), loadCaseIdx);
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
                    var resultLine = FrameElementEndForce.FromLine(reader.ReadLine(), loadCaseIdx);
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
                    var resultLine = ReactionOutput.FromLine(reader.ReadLine(), loadCaseIdx);
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
                    var resultLine = PeakFrameElementInternalForce.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    forceLines.Add(resultLine);
                }

                outputLines.Add(new LoadCaseOutput(0, displacementLines, frameElementEndForceLines, reactionLines,
                    forceLines));
            }
            return outputLines;
        }
    }
}
