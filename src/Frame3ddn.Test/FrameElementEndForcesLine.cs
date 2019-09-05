using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn.Test
{
//    class FrameElementEndForcesLine
//    {
//        public readonly int LoadCaseIdx;
//        public readonly int Elmnt;
//        public readonly int Node;
//        public readonly double Nx;
//        public readonly double Vy;
//        public readonly double Vz;
//        public readonly double Txx;
//        public readonly double Myy;
//        public readonly double Mzz;

//        public FrameElementEndForcesLine(int loadCaseIdx, int elmnt, int node, double nx, double vy, double vz, double txx, double myy, double mzz)
//        {
//            LoadCaseIdx = loadCaseIdx;
//            Elmnt = elmnt;
//            Node = node;
//            Nx = nx;
//            Vy = vy;
//            Vz = vz;
//            Txx = txx;
//            Myy = myy;
//            Mzz = mzz;
//        }

//        public static FrameElementEndForcesLine FromLine(string line, int loadCaseIdx)
//        {

//#if false
//EXAMPLE LINES
//     Elmnt  Node       Nx          Vy         Vz        Txx        Myy        Mzz
//     1      1  -8933.878t   4228.494      0.0        0.0        0.0   5541682.992
//     1      2   9098.536t  -4228.494      0.0        0.0        0.0   8547886.718
//     2      3  13183.544c   9412.641      0.0        0.0        0.0   1632656.956
            
//#endif
//            if (String.IsNullOrWhiteSpace(line))
//                return null;
//            var splits = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
//            if (splits.Length != 8)
//                return null;
//            var col = 0;
//            var Elmnt = Int32.Parse(splits[col++]) - 1;
//            var Node = Int32.Parse(splits[col++]) - 1;
//            string nxstring = splits[col++];
//            var nx = double.Parse(nxstring.Substring(0, nxstring.Length - 1)); //N
//            var vy = double.Parse(splits[col++]); //N
//            var vz = double.Parse(splits[col++]); //N
//            var txx = double.Parse(splits[col++]); //Nmm -> Nm
//            var myy = double.Parse(splits[col++]); //Nmm -> Nm
//            string mzzString = splits[col++];
//            var mzz = double.Parse(mzzString); //Nmm -> Nm
//            return new FrameElementEndForcesLine(loadCaseIdx, Elmnt, Node, nx, vy, vz, txx, myy, mzz);
//        }
//    }
}
