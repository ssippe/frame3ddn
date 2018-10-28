using System;
using System.IO;
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
            //StreamReader sr = new StreamReader(workspaceDir + "\\TestData\\exA.3dd");
            Input input = Input.Parse(sr);
        }
    }
}
