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
        public readonly decimal Nx;
        /// <summary>
        /// Newtons (N)
        /// </summary>
        public readonly decimal Vy;
        /// <summary>
        /// Newtons (N)
        /// </summary>
        public readonly decimal Vz;
        /// <summary>
        /// Newton.Metres (Nm)
        /// </summary>
        public readonly decimal Txx;
        /// <summary>
        /// Newton.Metres (Nm)
        /// </summary>
        public readonly decimal Myy;
        /// <summary>
        /// Newton.Metres (Nm)
        /// </summary>
        public readonly decimal Mzz;

        public PeakFrameElementInternalForceLine(int loadCaseIdx,
            int memberIndex,
            bool isMax,
            decimal nx,
            decimal vy,
            decimal vz,
            decimal txx,
            decimal myy,
            decimal mzz)
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
            var nx = Decimal.Parse(splits[col++]); //N
            var vy = Decimal.Parse(splits[col++]); //N
            var vz = Decimal.Parse(splits[col++]); //N
            var txx = Decimal.Parse(splits[col++]); //Nmm -> Nm
            var myy = Decimal.Parse(splits[col++]); //Nmm -> Nm
            var mzz = Decimal.Parse(splits[col++]); //Nmm -> Nm
            return new PeakFrameElementInternalForceLine(loadCaseIdx, memberIdx, isMax, nx, vy, vz, txx, myy, mzz);
        }
    }
}
