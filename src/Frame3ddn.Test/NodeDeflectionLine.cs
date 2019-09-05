using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn.Test
{
//    public class NodeDeflectionLine
//    {
//        /// <summary>
//        /// 0 based index (vs frame3dd who uses 1 based in their files)
//        /// </summary>
//        public readonly int LoadCaseIdx;
//        /// <summary>
//        /// 0 based index (vs frame3dd who uses 1 based in their files)
//        /// </summary>
//        public readonly int NodeIdx;

//        /// <summary>
//        /// mm
//        /// </summary>
//        public readonly double DisplacementX;
//        /// <summary>
//        /// mm
//        /// </summary>
//        public readonly double DisplacementY;

//        public readonly double Fz;
//        public readonly double Mxx;
//        public readonly double Myy;
//        public readonly double Mzz;

//        public double DisplacementAbs => (double)Math.Sqrt((double)(DisplacementX * DisplacementX + DisplacementY * DisplacementY));

//        public NodeDeflectionLine(int loadCaseIdx, int nodeIdx, double displacementX, double displacementY)
//        {
//            LoadCaseIdx = loadCaseIdx;
//            NodeIdx = nodeIdx;
//            DisplacementX = displacementX;
//            DisplacementY = displacementY;
//        }

//        public NodeDeflectionLine(int loadCaseIdx, int nodeIdx, double displacementX, double displacementY, double fz, double mxx, double myy, double mzz)
//        {
//            LoadCaseIdx = loadCaseIdx;
//            NodeIdx = nodeIdx;
//            DisplacementX = displacementX;
//            DisplacementY = displacementY;
//            Fz = fz;
//            Mxx = mxx;
//            Myy = myy;
//            Mzz = mzz;
//        }

//        public static NodeDeflectionLine FromLine(string line, int loadCaseIdx)
//        {

//#if false
//EXAMPLE LINES
//L O A D   C A S E   1   O F   4  ... 

//N O D E   D I S P L A C E M E N T S  					(global)
//  Node    X-dsp       Y-dsp       Z-dsp       X-rot       Y-rot       Z-rot
//     2    0.364254    0.013822    0.0         0.0         0.0         0.067766
//     3    0.000002    0.802262    0.0         0.0         0.0         0.000000




//#endif

//            if (String.IsNullOrWhiteSpace(line))
//                return null;
//            var splits = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
//            if (splits.Length != 7)
//                return null;
//            var col = 0;
//            var nodeIdx = Int32.Parse(splits[col++]) - 1;
//            var x = double.Parse(splits[col++]);
//            var y = double.Parse(splits[col++]);
//            var z = double.Parse(splits[col++]);
//            var mx = double.Parse(splits[col++]);
//            var my = double.Parse(splits[col++]);
//            var mz = double.Parse(splits[col++]);
            
//            return new NodeDeflectionLine(loadCaseIdx, nodeIdx, x, y, z, mx, my, mz);
//        }
//    }
}
