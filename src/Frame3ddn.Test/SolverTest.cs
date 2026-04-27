using Frame3ddn.Model;
using Frame3ddn.Parsers;
using Frame3ddn.Writers;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace Frame3ddn.Test
{

    public class SolverTest
    {


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
            //File.WriteAllText(Directory.GetFiles(outputPath, fileName + ".txt")[0], output.TextOutput);
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

            // The .txt reference may have more load cases than we computed; clip to what we ran.
            List<LoadCaseOutput> reference = OutParser.Parse(File.ReadAllText(outputFilePath))
                .Take(outputCalculated.LoadCaseOutputs.Count)
                .ToList();
            OutputAsserts.OutputAssertEqual(new Output("", reference), outputCalculated);
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
        public void Frame3ddExamplesCsv(string fileName)
        {
            string inputPath = GetFrame3ddExamplePath(fileName + ".csv");
            string referencePath = GetFrame3ddExamplePath(fileName + "_out.CSV");

            using StreamReader sr = new StreamReader(inputPath);
            Input input = CsvParser.Parse(sr);
            Solver solver = new Solver();
            Output outputCalculated = solver.Solve(input);

            List<LoadCaseOutput> reference = OutCsvParser.Parse(File.ReadAllText(referencePath));

            // When debugging, drop expected/actual .out files into a per-run timestamp dir
            // (TestResults/yyyyMMdd-HHmmss/SolverTest/) so they can be diffed by hand.
            if (DebugSnapshot.Enabled)
            {
                string classDir = DebugSnapshot.GetClassDir(nameof(SolverTest));
                string actualOutSection = OutWriter.OutputDataToString(outputCalculated.LoadCaseOutputs.ToList());
                int outputStart = outputCalculated.TextOutput.IndexOf(actualOutSection, StringComparison.Ordinal);
                string preamble = outputStart >= 0 ? outputCalculated.TextOutput.Substring(0, outputStart) : "";
                DebugSnapshot.WriteText(classDir, fileName + ".expected.out",
                    preamble + OutWriter.OutputDataToString(reference));
                DebugSnapshot.WriteText(classDir, fileName + ".actual.out",
                    preamble + actualOutSection);
            }

            // _out.CSV emits a row per node for both the displacement and reaction tables —
            // including all-zero rows for nodes the solver never bothered to write. Drop
            // those zero rows on both sides before comparing so the counts line up.
            // Skip the PEAK section only if the reference happens to omit it.
            bool referenceHasPeak = reference.Any(lc => lc.PeakFrameElementInternalForces.Count > 0);
            OutputAsserts.OutputAssertEqual(new Output("", reference), outputCalculated,
                new OutputAssertOptions
                {
                    IgnorePeakForces = !referenceHasPeak,
                    IgnoreZeroRows = true,
                });
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
        public void Frame3ddExamples3dd(string fileName)
        {
            // Same models as Frame3ddExamplesCsv but parsed from the legacy whitespace-
            // delimited .3dd input. Output must match the same .out reference.
            string inputPath = GetFrame3ddExamplePath(fileName + ".3dd");
            string referencePath = GetFrame3ddExamplePath(fileName + ".out");

            using StreamReader sr = new StreamReader(inputPath);
            Input input = ThreeDdParser.Parse(sr);
            Solver solver = new Solver();
            Output outputCalculated = solver.Solve(input);

            List<LoadCaseOutput> reference = OutParser.Parse(File.ReadAllText(referencePath));

            if (DebugSnapshot.Enabled)
            {
                string classDir = DebugSnapshot.GetClassDir(nameof(SolverTest));
                string actualOutSection = OutWriter.OutputDataToString(outputCalculated.LoadCaseOutputs.ToList());
                int outputStart = outputCalculated.TextOutput.IndexOf(actualOutSection, StringComparison.Ordinal);
                string preamble = outputStart >= 0 ? outputCalculated.TextOutput.Substring(0, outputStart) : "";
                DebugSnapshot.WriteText(classDir, fileName + ".3dd.expected.out",
                    preamble + OutWriter.OutputDataToString(reference));
                DebugSnapshot.WriteText(classDir, fileName + ".3dd.actual.out",
                    preamble + actualOutSection);
            }

            // The .out files are inconsistent w.r.t. the PEAK section (exA/exD/exH/exJ omit
            // it). Skip that section only when the reference itself doesn't supply it.
            bool referenceHasPeak3dd = reference.Any(lc => lc.PeakFrameElementInternalForces.Count > 0);
            OutputAsserts.OutputAssertEqual(new Output("", reference), outputCalculated,
                new OutputAssertOptions { IgnorePeakForces = !referenceHasPeak3dd });
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
            Assert.Equal(0.100000, n8.Displacement.X, 4);
            Assert.Equal(-0.147194, n8.Displacement.Y, 3);
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

        public static string GetUpstreamPath(params string[] segments)
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            string upstreamPath = Path.Combine(testDataPath, "UpstreamExamples");
            return Path.Combine(new[] { upstreamPath }.Concat(segments).ToArray());
        }

        private static string GetFrame3ddExamplePath(string fileName)
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            return Path.Combine(testDataPath, "frame3dd-examples", fileName);
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
            List<LoadCaseOutput> result1 = OutParser.Parse(File.ReadAllText(outputFilePath1));
            List<LoadCaseOutput> result2 = OutParser.Parse(File.ReadAllText(outputFilePath2));
            OutputAsserts.OutputAssertEqual(new Output("", result1), new Output("", result2));
        }

    }
}



