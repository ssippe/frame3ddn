using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
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
