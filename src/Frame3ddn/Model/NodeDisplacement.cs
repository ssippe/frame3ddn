using System;

namespace Frame3ddn.Model
{
    public class NodeDisplacement
    {
        /// <summary>
        /// 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public int LoadcaseIdx { get; }
        /// <summary>
        /// 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public int NodeIdx { get; }
        /// <summary>
        /// Displacement in x,y,z axes [mm]
        /// </summary>
        public Vec3 Displacement { get; }
        /// <summary>
        /// Rotation [radians]
        /// </summary>
        public Vec3 Rotation { get; }

        public NodeDisplacement(int loadcaseIdx, int nodeIdx, Vec3 displacement, Vec3 rotation)
        {
            LoadcaseIdx = loadcaseIdx;
            NodeIdx = nodeIdx;
            Displacement = displacement;
            Rotation = rotation;
        }
        public static NodeDisplacement FromLine(string line, int loadCaseIdx)
        {

#if false
EXAMPLE LINES
L O A D   C A S E   1   O F   4  ... 

N O D E   D I S P L A C E M E N T S  					(global)
  Node    X-dsp       Y-dsp       Z-dsp       X-rot       Y-rot       Z-rot
     2    0.364254    0.013822    0.0         0.0         0.0         0.067766
     3    0.000002    0.802262    0.0         0.0         0.0         0.000000




#endif

            if (String.IsNullOrWhiteSpace(line))
                return null;
            var splits = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (splits.Length != 7)
                return null;
            var col = 0;
            var nodeIdx = Int32.Parse(splits[col++]) - 1;
            var x = double.Parse(splits[col++]);
            var y = double.Parse(splits[col++]);
            var z = double.Parse(splits[col++]);
            var mx = double.Parse(splits[col++]);
            var my = double.Parse(splits[col++]);
            var mz = double.Parse(splits[col++]);

            var displacement = new Vec3(x,y,z);
            var rotation = new Vec3(mx, my, mz);
            return new NodeDisplacement(loadCaseIdx, nodeIdx, displacement, rotation);
        }
    }
}