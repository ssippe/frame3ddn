using Frame3ddn.Parsers;
using System.IO;
using Xunit;

namespace Frame3ddn.Test.Parsers
{
    public class CsvParserTest
    {

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
            string inputPath = GetExamplePath(fileName + ".csv");
            using StreamReader sr = new StreamReader(inputPath);
            CsvParser.Parse(sr);
        }

        private static string GetExamplePath(string fileName)
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            string testDataPath = Directory.GetDirectories(workspaceDir, "TestData")[0];
            return Path.Combine(testDataPath, "frame3dd-examples", fileName);
        }
    }
}



