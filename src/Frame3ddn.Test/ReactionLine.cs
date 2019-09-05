using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn.Test
{
//    public class ReactionLine
//    {
//        /// <summary>
//        /// Combination load case, 0 based index (vs frame3dd who uses 1 based in their files)
//        /// </summary>
//        public readonly int LoadCaseIdx;

//        /// <summary>
//        /// 0 based index (vs frame3dd who uses 1 based in their files)
//        /// </summary>
//        public readonly int NodeIndex;

//        /// <summary>
//        /// Force in x global axis [N]
//        /// </summary>
//        public readonly double Fx;

//        /// <summary>
//        /// Force in y global axis [N]
//        /// </summary>
//        public readonly double Fy;

//        /// <summary>
//        /// Force in z global axis [N]
//        /// </summary>
//        public readonly double Fz;


//        /// <summary>
//        /// Momentem in x global axis [N.mm]
//        /// </summary>
//        public readonly double Mxx;

//        /// <summary>
//        /// Momentem in y global axis [N.mm]
//        /// </summary>
//        public readonly double Myy;

//        /// <summary>
//        /// Momentem in z global axis [N.mm]
//        /// </summary>
//        public readonly double Mzz;

//        public ReactionLine(int loadCaseIdx,
//            int nodeIndex,
//            double fx, double fy, double fz,
//            double mxx, double myy, double mzz)
//        {
//            LoadCaseIdx = loadCaseIdx;
//            NodeIndex = nodeIndex;
//            Fx = fx;
//            Fy = fy;
//            Fz = fz;
//            Mxx = mxx;
//            Myy = myy;
//            Mzz = mzz;

//        }

//        public static ReactionLine FromLine(string line, int loadCaseIdx)
//        {

//#if false
//R E A C T I O N S							(global)
//  Node        Fx          Fy          Fz         Mxx         Myy         Mzz
//     1   -3968.510   -2192.546       0.000       0.000       0.000 4397503.949
//     5     285.445    -952.417       0.000       0.000       0.000  -51381.319
//    10   -7885.642   -4289.699       0.000       0.000       0.000 9148790.620
//    14     519.512   -2000.227       0.000       0.000       0.000  363163.833


//#endif

//            if (String.IsNullOrWhiteSpace(line))
//                return null;
//            var splits = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
//            if (splits.Length != 7)
//                return null;
//            var col = 0;
//            var nodeIdx = Int32.Parse(splits[col++]) - 1;
//            var fx = double.Parse(splits[col++]); //N        
//            var fy = double.Parse(splits[col++]); //N        
//            var fz = double.Parse(splits[col++]); //N        
//            var mxx = double.Parse(splits[col++]); //N.mm        
//            var myy = double.Parse(splits[col++]); //N.mm        
//            var mzz = double.Parse(splits[col++]); //N.mm        

//            return new ReactionLine(loadCaseIdx, nodeIdx, fx, fy, fz, mxx, myy, mzz);
//        }
//    }
}
