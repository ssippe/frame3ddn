using System;

namespace Frame3ddn.Model
{
    public class ReactionOutput
    {
        /// <summary>
        /// Combination load case, 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public int LoadcaseIdx { get; }
        /// <summary>
        /// 0 based index (vs frame3dd who uses 1 based in their files)
        /// </summary>
        public int NodeIdx { get; }
        /// <summary>
        /// Force in x,y,z global axes [N]
        /// </summary>
        public Vec3 F { get; }
        /// <summary>
        /// Momentum in x,y,z global axis [N.mm]
        /// </summary>
        public Vec3 M { get; }

        public ReactionOutput(int loadcaseIdx, int nodeIdx, Vec3 f, Vec3 m)
        {
            LoadcaseIdx = loadcaseIdx;
            NodeIdx = nodeIdx;
            F = f;
            M = m;
        }

        public static ReactionOutput FromLine(string line, int loadCaseIdx)
        {

#if false
R E A C T I O N S							(global)
  Node        Fx          Fy          Fz         Mxx         Myy         Mzz
     1   -3968.510   -2192.546       0.000       0.000       0.000 4397503.949
     5     285.445    -952.417       0.000       0.000       0.000  -51381.319
    10   -7885.642   -4289.699       0.000       0.000       0.000 9148790.620
    14     519.512   -2000.227       0.000       0.000       0.000  363163.833


#endif

            if (String.IsNullOrWhiteSpace(line))
                return null;
            var splits = line.Split(new[] {' '}, StringSplitOptions.RemoveEmptyEntries);
            if (splits.Length != 7)
                return null;
            var col = 0;
            var nodeIdx = Int32.Parse(splits[col++]) - 1;
            var fx = double.Parse(splits[col++]); //N        
            var fy = double.Parse(splits[col++]); //N        
            var fz = double.Parse(splits[col++]); //N        
            var mxx = double.Parse(splits[col++]); //N.mm        
            var myy = double.Parse(splits[col++]); //N.mm        
            var mzz = double.Parse(splits[col++]); //N.mm        

            var f = new Vec3(fx, fy, fz);
            var m = new Vec3(mxx, myy, mzz);
            return new ReactionOutput(loadCaseIdx, nodeIdx, f, m);
        }
    }
}