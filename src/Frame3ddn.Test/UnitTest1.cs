using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using Xunit;

namespace Frame3ddn.Test
{
    public class UnitTest1
    {
        [Fact]
        public void GetInputFromFile()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            StreamReader sr = new StreamReader(workspaceDir + "\\TestData\\TEST.csv");
            Input input = Input.Parse(sr);
        }

        [Fact]
        public void Run()
        {
            const string inputFileName = "TEST3.csv";
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            StreamReader sr = new StreamReader(workspaceDir + "\\TestData\\" + inputFileName);
            Input input = Input.Parse(sr);
            Solver solver = new Solver();
            Output output = solver.Solve(input);
            ExportOutput(output, workspaceDir + "\\TestData\\testResult.csv");
        }

        [Fact]
        public void BatchCompare()
        {

        }

        private void Compare()
        {
            const string inputFileName = "TEST3.csv";
            string workspaceDir1 = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            StreamReader sr = new StreamReader(workspaceDir1 + "\\TestData\\" + inputFileName);
            Input input = Input.Parse(sr);
            Solver solver = new Solver();
            Output output = solver.Solve(input);

            const string outputFileName = "TEST3.txt";
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string file = File.ReadAllText(workspaceDir + "\\TestData\\" + outputFileName);
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
            if (Math.Abs(num1 - num2) < Math.Abs(num1) * 0.01 || Math.Abs(num1 - num2) < 0.00001)
                return true;
            return false;
        }

        private void ExportOutput(Output output, string outputPath)
        {
            File.WriteAllText(outputPath, "");
            var nL = output.LoadCaseOutputs.Count;
            for(int i = 0; i < nL; i ++)
            {
                LoadCaseOutput loadCaseOutput = output.LoadCaseOutputs[i];
                var csv = new StringBuilder();
                string newLine;
                csv.AppendLine("------------------Load case " + (i + 1) + "------------------");

                csv.AppendLine("* Node displacements");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                    "Node", "X-dsp", "Y-dsp", "z-dsp", "X-rot", "Y-rot", "Z-rot");
                csv.AppendLine(newLine);
                foreach (var nodeDisplacement in loadCaseOutput.NodeDisplacements)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}", 
                        nodeDisplacement.NodeIdx + 1, nodeDisplacement.Displacement.X, nodeDisplacement.Displacement.Y, nodeDisplacement.Displacement.Z,
                        nodeDisplacement.Rotation.X, nodeDisplacement.Rotation.Y, nodeDisplacement.Rotation.Z);
                    csv.AppendLine(newLine);
                }

                csv.AppendLine("* Frame Element End Force");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                    "Elmnt", "Node", "Nx", "Vy", "Vz", "Txx", "Myy", "Mzz");
                csv.AppendLine(newLine);
                foreach (var frameElementEndForce in loadCaseOutput.FrameElementEndForces)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                        frameElementEndForce.ElementIdx + 1, frameElementEndForce.NodeIdx + 1, frameElementEndForce.Nx + frameElementEndForce.NxType,
                        frameElementEndForce.Vy, frameElementEndForce.Vz, frameElementEndForce.Txx, frameElementEndForce.Myy, frameElementEndForce.Mzz);
                    csv.AppendLine(newLine);
                }

                csv.AppendLine("* Reactions");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                    "Node", "Fx", "Fy", "Fz", "Mxx", "Myy", "Mzz");
                csv.AppendLine(newLine);
                foreach (var reactionOutput in loadCaseOutput.ReactionOutputs)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6}",
                        reactionOutput.NodeIdx + 1, reactionOutput.F.X, reactionOutput.F.Y, reactionOutput.F.Z,
                        reactionOutput.M.X, reactionOutput.M.Y, reactionOutput.M.Z);
                    csv.AppendLine(newLine);
                }
                csv.AppendLine("RMS Relative Equilibrium Error: " + loadCaseOutput.RmsRelativeEquilibriumError);

                csv.AppendLine("* Peak Frame Element Internal Forces");
                newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                    "Elmnt", ".", "Nx", "Vy", "Vz", "Txx", "Myy", "Mzz");
                csv.AppendLine(newLine);
                foreach (var peakFrameElementInternalForce in loadCaseOutput.PeakFrameElementInternalForces)
                {
                    newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7}",
                        peakFrameElementInternalForce.ElementIdx + 1, peakFrameElementInternalForce.IsMin? "min" : "max",
                        peakFrameElementInternalForce.Nx, peakFrameElementInternalForce.Vy, peakFrameElementInternalForce.Vz,
                        peakFrameElementInternalForce.Txx, peakFrameElementInternalForce.Myy, peakFrameElementInternalForce.Mzz);
                    csv.AppendLine(newLine);
                }
                File.AppendAllText(outputPath, csv.ToString());
                
            }
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


                ////////////////////////
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

                ////////////////////////
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

                ////////////////////////
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

                ////////////////////////
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

        //private static List<string> GetOutput(StreamReader sr)
        //{
        //    List<string> output = new List<string>();
        //    string line;
        //    while ((line = sr.ReadLine()) != null)
        //    {
        //        if (line.Contains("E L A S T I C   S T I F F N E S S   A N A L Y S I S"))
        //        {
        //            int readingState = 0;
        //            while ((line = sr.ReadLine()) != null)
        //            {

        //            }
        //        }

        //        break;
        //    }
        //    return noComentInput;
        //}

        //[Fact]
        //public void FindTrapLoadAndNodeLoad()
        //{
        //    string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
        //    //string[] pdfFiles = Directory.GetFiles(workspaceDir + "\\frame3ddInput24\\frame3ddInput", "*");
        //    //string[] pdfFiles = Directory.GetFiles(workspaceDir + "\\frame3ddInputBase\\frame3ddInput", "*");
        //    //string[] pdfFiles = Directory.GetFiles(workspaceDir + "\\frame3ddInputMore\\frame3ddInput", "*");
        //    string[] pdfFiles = Directory.GetFiles(workspaceDir + "\\frame3ddInputSkil\\frame3ddInput", "*");
        //    //string[] pdfFiles = Directory.GetFiles(workspaceDir + "\\frame3ddInput", "*");
        //    for (int i = 0; i < pdfFiles.Length; i++)
        //    {
        //        StreamReader sr = new StreamReader(pdfFiles[i]);
        //        Input input = Input.Parse(sr);
        //        bool t = false;
        //        bool n = false;
        //        foreach (LoadCase inputLoadCase in input.LoadCases)
        //        {
        //            if (inputLoadCase.NodeLoads.Count > 0)
        //                n = true;
        //            if (inputLoadCase.TrapLoads.Count > 0)
        //                t = true;
        //        }

        //        if (n)
        //        {
        //            string path = pdfFiles[i];
        //            Console.WriteLine(pdfFiles[i] + " has node loads.");
        //        }
        //        if (t)
        //        {
        //            string path = pdfFiles[i];
        //            Console.WriteLine(pdfFiles[i] + " has trap nodes.");
        //        }

        //    }

        //}

        [Fact]
        public void FindLoads()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            List<string[]> directoryList = new List<string[]>();
            directoryList.Add(Directory.GetFiles(workspaceDir + "\\frame3ddInput24\\frame3ddInput", "*"));
            directoryList.Add(Directory.GetFiles(workspaceDir + "\\frame3ddInputBase\\frame3ddInput", "*"));
            directoryList.Add(Directory.GetFiles(workspaceDir + "\\frame3ddInputMore\\frame3ddInput", "*"));
            directoryList.Add(Directory.GetFiles(workspaceDir + "\\frame3ddInputSkil\\frame3ddInput", "*"));
            directoryList.Add(Directory.GetFiles(workspaceDir + "\\frame3ddInput\\frame3ddInput", "*"));
            foreach (string[] pdfFiles in directoryList)
            {
                for (int i = 0; i < pdfFiles.Length; i++)
                {
                    StreamReader sr = new StreamReader(pdfFiles[i]);
                    Input input = Input.Parse(sr);
                    if (!input.IncludeGeometricStiffness)
                    {
                        string path = pdfFiles[i];
                        Console.WriteLine(path);
                    }
                }
            }
        }
    }
}
