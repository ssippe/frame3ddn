using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn.Test
{
    public class PeakFrameElementInternalForceLine
    {
        /// <summary>
        /// Combination load case, 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public readonly int LoadCaseIdx;
        /// <summary>
        /// 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public readonly int MemberIndex;
        public readonly bool IsMax;
        /// <summary>
        /// Newtons (N) Force along the primiary axis of the member. +ve tension, -ve compression
        /// </summary>
        public readonly double Nx;
        /// <summary>
        /// Newtons (N)
        /// </summary>
        public readonly double Vy;
        /// <summary>
        /// Newtons (N)
        /// </summary>
        public readonly double Vz;
        /// <summary>
        /// Newton.Metres (Nm)
        /// </summary>
        public readonly double Txx;
        /// <summary>
        /// Newton.Metres (Nm)
        /// </summary>
        public readonly double Myy;
        /// <summary>
        /// Newton.Metres (Nm)
        /// </summary>
        public readonly double Mzz;

        public PeakFrameElementInternalForceLine(int loadCaseIdx,
            int memberIndex,
            bool isMax,
            double nx,
            double vy,
            double vz,
            double txx,
            double myy,
            double mzz)
        {
            LoadCaseIdx = loadCaseIdx;
            MemberIndex = memberIndex;
            IsMax = isMax;
            Nx = nx;
            Vy = vy;
            Vz = vz;
            Txx = txx;
            Myy = myy;
            Mzz = mzz;
        }

        public static PeakFrameElementInternalForceLine FromLine(string line, int loadCaseIdx)
        {

#if false
EXAMPLE LINES
     1   max      -5.000       1.892      0.000      0.000      0.000      3.486
     1   min      -5.000       1.892      0.000      0.000      0.000     -4.083
     2   max       3.800      -3.381      0.000      0.000      0.000      0.661



#endif

            if (String.IsNullOrWhiteSpace(line))
                return null;
            var splits = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (splits.Length < 8)
                return null;
            var col = 0;
            var memberIdx = Int32.Parse(splits[col++]) - 1;
            var isMax = splits[col++] == "max";
            var nx = double.Parse(splits[col++]); //N
            var vy = double.Parse(splits[col++]); //N
            var vz = double.Parse(splits[col++]); //N
            var txx = double.Parse(splits[col++]); //Nmm -> Nm
            var myy = double.Parse(splits[col++]); //Nmm -> Nm
            var mzz = double.Parse(splits[col++]); //Nmm -> Nm
            return new PeakFrameElementInternalForceLine(loadCaseIdx, memberIdx, isMax, nx, vy, vz, txx, myy, mzz);
        }
    }
}
