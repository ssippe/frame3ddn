using Frame3ddn.Model;
using Frame3ddn.Parsers;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
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
            Model.Input input = CsvParser.Parse(sr);
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
            Input input = CsvParser.Parse(sr);
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

            for (int i = 0; i < fileNameList.Count; i++)
            {
                string fileName = fileNameList[i];
                Compare(
                    Directory.GetFiles(inputPath, fileName + ".csv")[0],
                    Directory.GetFiles(outputFromCProgramPath, fileName + ".txt")[0]
                );
                System.Diagnostics.Debug.WriteLine("finished comparing file: " + i + " " + fileName);
            }
        }

        private void Compare(string inputFilePath, string outputFilePath)
        {
            StreamReader sr = new StreamReader(inputFilePath);
            Input input = CsvParser.Parse(sr);
            Solver solver = new Solver();
            Output outputCalculated = solver.Solve(input);

            string file = File.ReadAllText(outputFilePath);
            List<LoadCaseOutput> result = OutParser.Parse(file);

            for (int i = 0; i < outputCalculated.LoadCaseOutputs.Count; i++)
            {
                LoadCaseOutput loadCaseOutput = outputCalculated.LoadCaseOutputs[i];
                LoadCaseOutput loadCaseOutputToCompare = result[i];
                if (!IsEqual(loadCaseOutput, loadCaseOutputToCompare))
                    throw new Exception("?");
            }
        }

        [Theory]
        [InlineData("exA")]
        [InlineData("exB")]
        [InlineData("exC")]
        [InlineData("exD")]
        [InlineData("exE")]
        [InlineData("exF")]
        [InlineData("exG")]
        [InlineData("exH")]
        [InlineData("exI")]
        [InlineData("exJ")]
        public void UpstreamExampleParse(string fileName)
        {
            string inputPath = GetUpstreamPath("Input", fileName + ".csv");
            using StreamReader sr = new StreamReader(inputPath);
            CsvParser.Parse(sr);
        }

        [Theory]
        [InlineData("exA")]
        [InlineData("exB")]
        [InlineData("exC")]
        [InlineData("exD")]
        [InlineData("exE")]
        [InlineData("exF")]
        [InlineData("exG")]
        [InlineData("exH")]
        [InlineData("exI")]
        [InlineData("exJ")]
        public void UpstreamExampleSolveAndCompare(string fileName)
        {
            string inputPath = GetUpstreamPath("Input", fileName + ".csv");
            string referencePath = GetUpstreamPath("OutputFromCProgram", fileName + ".out");

            using StreamReader sr = new StreamReader(inputPath);
            Input input = CsvParser.Parse(sr);
            Solver solver = new Solver();
            Output outputCalculated = solver.Solve(input);

            List<LoadCaseOutput> reference = OutParser.Parse(File.ReadAllText(referencePath));

            Assert.Equal(reference.Count, outputCalculated.LoadCaseOutputs.Count);
            for (int i = 0; i < outputCalculated.LoadCaseOutputs.Count; i++)
            {
                Assert.True(
                    IsEqual(outputCalculated.LoadCaseOutputs[i], reference[i]),
                    $"Load case {i} of {fileName} did not match upstream reference output.");
            }
        }

        [Fact]
        public void PrescribedDisplacementAppliedAtRestrainedDoF()
        {
            // exA LC1 prescribes Dx = 0.1 in at node 8 (a restrained X DoF).
            // This test verifies the prescribed-displacement parser + solver wiring:
            // the prescribed value must appear as the computed displacement at that DoF,
            // and the free-DoF displacement at the same node must match upstream.
            // Reference: upstream exA.out, load case 1, node 8 row.
            string inputPath = GetUpstreamPath("Input", "exA_lc1.csv");

            using StreamReader sr = new StreamReader(inputPath);
            Input input = CsvParser.Parse(sr);

            Assert.Single(input.LoadCases);
            Assert.Single(input.LoadCases[0].PrescribedDisplacements);
            PrescribedDisplacement pd = input.LoadCases[0].PrescribedDisplacements[0];
            Assert.Equal(7, pd.NodeIdx);
            Assert.Equal(0.1, pd.Displacement.X);

            Solver solver = new Solver();
            Output output = solver.Solve(input);

            NodeDisplacement n8 = output.LoadCaseOutputs[0].NodeDisplacements.Single(d => d.NodeIdx == 7);
            Assert.True(CompareDouble(n8.Displacement.X, 0.100000),
                $"Prescribed displacement at node 8 X expected 0.1, got {n8.Displacement.X}");
            Assert.True(CompareDouble(n8.Displacement.Y, -0.147194),
                $"Free-DoF displacement at node 8 Y expected -0.147194, got {n8.Displacement.Y}");
        }

        [Fact]
        public void PrescribedDisplacementValidatesAgainstFreeDoF()
        {
            // The upstream solver requires prescribed displacements to be applied only at
            // restrained DoFs. Construct a load case that prescribes a displacement at a
            // free DoF (node 8 Mz) and confirm the C# port rejects it.
            string baseInput = File.ReadAllText(GetUpstreamPath("Input", "exA_lc1.csv"));
            // Replace the prescribed-disp row with a non-zero Mz value at node 8 (Mz is free per reactions table).
            string mutated = baseInput.Replace("8,0.1,0,0,0,0,0", "8,0,0,0,0,0,0.05");

            byte[] bytes = System.Text.Encoding.UTF8.GetBytes(mutated);
            using StreamReader srr = new StreamReader(new MemoryStream(bytes));
            Input input = CsvParser.Parse(srr);

            Solver solver = new Solver();
            Exception ex = Assert.Throws<Exception>(() => solver.Solve(input));
            Assert.Contains("restrained coordinates", ex.Message);
        }

        private static string GetUpstreamPath(params string[] segments)
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string upstreamPath = Path.Combine(testDataPath, "UpstreamExamples");
            return Path.Combine(new[] { upstreamPath }.Concat(segments).ToArray());
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
            List<LoadCaseOutput> result1 = OutParser.Parse(file1);
            List<LoadCaseOutput> result2 = OutParser.Parse(file2);


            for (int i = 0; i < result1.Count; i++)
            {
                LoadCaseOutput loadCaseOutput1 = result1[i];
                LoadCaseOutput loadCaseOutput2 = result2[i];
                if (!IsEqual(loadCaseOutput1, loadCaseOutput2))
                    throw new Exception("?");
            }
        }

        static bool IsMinMaxForceEqual(IEnumerable<PeakFrameElementInternalForce> o1,
            IEnumerable<PeakFrameElementInternalForce> o2)
        {
            List<PeakFrameElementInternalForce> o1Filt = o1.Where(f => f.IsMin.HasValue).ToList();
            List<PeakFrameElementInternalForce> o2Filt = o2.Where(f => f.IsMin.HasValue).ToList();
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

    }
}



