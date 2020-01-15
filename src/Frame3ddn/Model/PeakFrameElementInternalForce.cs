using System;

namespace Frame3ddn.Model
{
    public class PeakFrameElementInternalForce
    {
        /// <summary>
        /// Combination load case, 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public int LoadcaseIdx { get; }
        /// <summary>
        /// Element Index base-0 (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public int ElementIdx { get; }
        /// <summary>
        /// true=>minimum, false=>maximum, null=>value at XOffset
        /// </summary>
        public bool? IsMin { get; }
        /// <summary>
        /// Frame element axial force along the local x-axis [N]
        /// Force along the primiary axis of the member. +ve tension, -ve compression        
        /// </summary>
        public double Nx { get; }
        /// <summary>
        /// Frame element shear force in the local y direction [N]
        /// </summary>
        public double Vy { get; }
        /// <summary>
        /// Frame element shear force in the local z direction [N]
        /// </summary>
        public double Vz { get; }
        /// <summary>
        /// Frame element torsion about the local x-axis [N.mm]
        /// </summary>
        public double Txx { get; }
        /// <summary>
        /// Frame element bending moments about the local -y-axis [N.mm]
        /// </summary>
        public double Myy { get; }
        /// <summary>
        /// Frame element bending moments about the local z-axis [N.mm]
        /// </summary>
        public double Mzz { get; }
        /// <summary>
        /// Offset from the start of the member [mm]
        /// </summary>
        public double? XOffset { get; }

        public PeakFrameElementInternalForce(int loadcaseIdx, int elementIdx, bool? isMin, double nx, double vy, double vz, double txx, double myy, double mzz, double? xOffset)
        {
            LoadcaseIdx = loadcaseIdx;
            ElementIdx = elementIdx;
            IsMin = isMin;
            Nx = nx;
            Vy = vy;
            Vz = vz;
            Txx = txx;
            Myy = myy;
            Mzz = mzz;
            XOffset = xOffset;
        }

        public static PeakFrameElementInternalForce FromLine(string line, int loadCaseIdx)
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
            var txx = double.Parse(splits[col++]); //Nmm 
            var myy = double.Parse(splits[col++]); //Nmm 
            var mzz = double.Parse(splits[col++]); //Nmm 
            return new PeakFrameElementInternalForce(loadCaseIdx, memberIdx, !isMax, nx, vy, vz, txx, myy, mzz, null);
        }
    }
}