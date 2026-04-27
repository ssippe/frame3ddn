using Frame3ddn.Parsers;
using System.IO;
using Xunit;

namespace Frame3ddn.Test.Parsers
{
    public class CsvParserTest
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
            string inputPath = SolverTest.GetUpstreamPath("Input", fileName + ".csv");
            using StreamReader sr = new StreamReader(inputPath);
            CsvParser.Parse(sr);
        }

    }
}



