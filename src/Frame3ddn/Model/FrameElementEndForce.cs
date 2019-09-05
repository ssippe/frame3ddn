using System;

namespace Frame3ddn.Model
{
    public class FrameElementEndForce
    {
        public int LoadcaseIdx { get; }
        public int ElementIdx { get; }
        public int NodeIdx { get; }
        public double Nx { get; }
        public string NxType { get; }
        public double Vy { get; }
        public double Vz { get; }
        public double Txx { get; }
        public double Myy { get; }
        public double Mzz { get; }

        public FrameElementEndForce(int loadcaseIdx, int elementIdx, int nodeIdx, double nx, string nxType, double vy, double vz, double txx, double myy, double mzz)
        {
            LoadcaseIdx = loadcaseIdx;
            ElementIdx = elementIdx;
            NodeIdx = nodeIdx;
            Nx = nx;
            NxType = nxType;
            Vy = vy;
            Vz = vz;
            Txx = txx;
            Myy = myy;
            Mzz = mzz;
        }

        static string GetNxType(string nxText)
        {
            if (nxText.EndsWith("t"))
                return "t";
            if (nxText.EndsWith("c"))
                return "c";
            return "";
        }

        public static FrameElementEndForce FromLine(string line, int loadCaseIdx)
        {

#if false
EXAMPLE LINES
     Elmnt  Node       Nx          Vy         Vz        Txx        Myy        Mzz
     1      1  -8933.878t   4228.494      0.0        0.0        0.0   5541682.992
     1      2   9098.536t  -4228.494      0.0        0.0        0.0   8547886.718
     2      3  13183.544c   9412.641      0.0        0.0        0.0   1632656.956
            
#endif
            if (String.IsNullOrWhiteSpace(line))
                return null;
            var splits = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (splits.Length != 8)
                return null;
            var col = 0;
            var Elmnt = Int32.Parse(splits[col++]) - 1;
            var Node = Int32.Parse(splits[col++]) - 1;
            string nxstring = splits[col++];
            var nx = double.Parse(nxstring.Substring(0, nxstring.Length - 1)); //N
            var nxType = GetNxType(nxstring);
            var vy = double.Parse(splits[col++]); //N
            var vz = double.Parse(splits[col++]); //N
            var txx = double.Parse(splits[col++]); //Nmm -> Nm
            var myy = double.Parse(splits[col++]); //Nmm -> Nm
            string mzzString = splits[col++];
            var mzz = double.Parse(mzzString); //Nmm -> Nm
            return new FrameElementEndForce(loadCaseIdx, Elmnt, Node, nx, nxType, vy, vz, txx, myy, mzz);
        }
        
    }
}