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

        [Fact]
        public void Run()
        {
            string workspaceDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory().ToString()).ToString()).ToString()).ToString();
            StreamReader sr = new StreamReader(workspaceDir + "\\TestData\\TEST2.csv");
            //StreamReader sr = new StreamReader(workspaceDir + "\\TestData\\exA.3dd");
            Input input = Input.Parse(sr);
            Solver solver = new Solver();
            solver.Solve(input);
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
    }
}
