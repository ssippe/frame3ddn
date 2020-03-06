using System;
using System.Linq;
using System.Text;
using Frame3ddn.Core;
using Frame3ddn.Model;

namespace Frame3ddn
{
    public static class InputCsvWriter
    {
        public static string GetCsv(Input input, string fileStamp)
        {
            var textFile = new StringBuilder();
            textFile.AppendLine(CreateHeader(fileStamp));
            textFile.AppendLine(CreateNodes(input));
            textFile.AppendLine(CreateReactions(input));
            textFile.AppendLine(CreateFrameElements(input));
            textFile.AppendLine(CreateConfigSection());
            textFile.AppendLine(CreateLoads(input));
            textFile.AppendLine(CreateEnd());
            return textFile.ToString();
        }

        private static string CreateConfigSection()
        {
            return @",,,,,,,,,,,,
,,,,,,,,,,,,
0,,""# 1: include shear deformations, 0: don't"",,,,,,,,,,
0,,""# 1: include geometric stiffness, 0: don't"",,,,,,,,,,
10,,# exaggerate mesh deformations,,,,,,,,,,
1,,# zoom scale for 3D plotting,,,,,,,,,,
100,,# x-axis increment for internal forces,,,,,,,,,,
,,,,,,,,,,,,
";
        }

        private static string CreateLoads(Input input)
        {
            var sb = new StringBuilder();

            sb.AppendLine($@",,,,,,,,,,,,
,,,,,,,,,,,,
{input.LoadCases.Count},,,,,# number of static load cases");

            for (int i = 0; i < input.LoadCases.Count; i++)
            {
                var loadCase = input.LoadCases[i];
                //var realisedLoadCase = combinationLoadCaseIdx.Value.ToLoadCase(structure, primaryLoadCases);
                sb.AppendLine($@"# Begin Load Case {i + 1}/{input.LoadCases.Count}");
                sb.AppendLine(CreateLoadGravity(loadCase));
                sb.AppendLine(CreateLoadNode(loadCase));
                sb.AppendLine(CreateLoadUniform(loadCase));
                sb.AppendLine(CreateLoadTrapezoid(loadCase));
                sb.AppendLine(CreateLoadUnused());
                sb.AppendLine($@"# End Load Case");
            }

            return sb.ToString();
        }

        private static string CreateLoadUnused()
        {
            return @"
0,,,,,# number of internal concentrated loads
0,,,,,# number of temperature loads
0,,,,,# number of nodes with prescribed displacements";
        }

        private static string CreateLoadTrapezoid(LoadCase loadCase)
        {
            var sb = new StringBuilder();
            /*
                # Trapezoidally distributed loading,,,,,,,,,,,,
                nW,,,,,# number of trapezoidally-distributed element loads (local),,,,,,,
                #,start,stop,start,stop,,,,,,,,
                #.elmnt,loc'n,loc'n,load,load,,,,,,,,
                #,mm,mm,N/mm,N/mm,,,,,,,,
                M[1],xx1[1],xx2[2],wx1[1],wx2[1],# locations and loads - local x-axis,,,,,,,
                ,xy1[1],xy2[2],wy1[1],wy2[1],# locations and loads - local y-axis,,,,,,,
                ,xz1[1],xz2[2],wz1[1],wz2[1],# locations and loads - local z-axis,,,,,,,
                ,:,:,:,:,,,,,,,,
                M[nW],xx1[nW],xx2[nW],wx1[nW],wx2[nW],# x1 and x2: start and end locations,,,,,,,
                ,xy1[nW],xy2[nW],wy1[nW],wy2[nW],# w1 and w2: start load and end load,,,,,,,
                ,xz1[nW],xz2[nW],wz1[nW],wz2[nW],# 0 < x1 < x2 < L,,,,,,,
            */
            var loadCount = 0;
            //var loads = loadCase.MemberLoads.Where(f => !f.IsUniform()).ToList();
            for (int i = 0; i < loadCase.TrapLoads.Count; i++)
            {
                var load = loadCase.TrapLoads[i];
                var m = load.ElementIdx + 1;

                var xx1 = load.LocationStart.X;
                var xx2 = load.LocationEnd.X;
                var wx1 = load.LoadStart.X;
                var wx2 = load.LoadEnd.X;

                var xy1 = load.LocationStart.Y;
                var xy2 = load.LocationEnd.Y;
                var wy1 = load.LoadStart.Y;
                var wy2 = load.LoadEnd.Y;

                var xz1 = load.LocationStart.Z;
                var xz2 = load.LocationEnd.Z;
                var wz1 = load.LoadStart.Z;
                var wz2 = load.LoadEnd.Z;




                sb.AppendLine(
                    $@"{m}, {xx1}, {xx2}, {wx1}, {wx2}, # locations and loads - local x-axis
, {xy1}, {xy2}, {wy1}, {wy2} , # locations and loads - local y-axis
, {xz1}, {xz2}, {wz1}, {wz2}, # locations and loads - local z-axis");
            }

            var header = $@"{loadCount}, ,,,,# number of trapezoidally-distributed element loads (local)";
            if (loadCount > 0)
            {
                header += @"
#,start,stop,start,stop
#.elmnt,loc'n,loc'n,load,load
#,mm,mm,N/mm,N/mm
";
            }

            sb.Insert(0, header);
            return sb.ToString();
        }

        private static string CreateLoadUniform(LoadCase loadCase)
        {
            var sb = new StringBuilder();
            var loadCount = 0;

#if false
"# Uniformly distributed loading",,,,,,,,,,,,
nU,,,,,"# number of uniformly distributed element loads (local)",,,,,,,
.elmnt,X-load,Y-load,Z-load,,"Uniform member loads in member coordinates.",,,,,,,
idx,N/mm,N/mm,N/mm,,,,,,,,,
M[1],Ux[1],Uy[1],Uz[1],,,,,,,,,
:,:,:,:,,,,,,,,,
M[nU],Ux[nU],Uy[nU],Uz[nU],,,,,,,,,
#endif

            // uniform loads are in Newtons per Millimetre [N/mm] which is the same as kilo-Newtons per metre [kN/m]
            for (int i = 0; i < loadCase.UniformLoads.Count; i++)
            {
                var load = loadCase.UniformLoads[i];
                sb.AppendLine($"{i + 1}, {load.Load.X}, {load.Load.Y}, {load.Load.Z},");
            }

            sb.Insert(0,
                $@",,,,,,,,,,,,
,,,,,,,,,,,,
{loadCount}, ,,,,# number of uniform loads
#.e,Ux,Uy,Uz
#,N/mm,N/mm,N/mm
");

            return sb.ToString();
        }


        private static string CreateLoadNode(LoadCase loadCase)
            {
                // units are Newtons [N] for loads and Newton Millimetres for moments [N.mm]
#if false
text file version below, we'll emit csv

nF                   # number of loaded nodes (global)
 .node  X-load   Y-load   Z-load   X-mom     Y-mom     Z-mom
  N        N        N        N.mm      N.mm      N.mm
  N[1]    Fx[1]    Fy[1]    Fz[1]    Mxx[1]    Myy[1]    Mzz[1]
    :        :        :        :         :         :         :  
  N[nF]   Fx[nF]   Fy[nF]   Fz[nF]   Mxx[nF]   Myy[nF]   Mzz[nF]
#endif
                if (loadCase.NodeLoads?.Any() != true)
                    return @"0, ,,,,# number of loaded nodes,,,,,,,";
                var sb = new StringBuilder();
                sb.Append($@"{loadCase.NodeLoads.Count}, ,,,,# number of loaded nodes,,,,,,,
# nodeIdx, X-load, Y-load, Z-load, X-mom, Y-mom, Z-mom
# , N, N, N, N.mm, N.mm, N.mm
");
                var nodeLines = loadCase.NodeLoads
                    .Select(f =>
                        $"{f.NodeIdx}, {f.Load.X}, {f.Load.Y}, {f.Load.Z}, {f.Moment.X}, {f.Moment.Y}, {f.Moment.Z}")
                    .JoinString(Environment.NewLine);
                sb.Append(nodeLines).AppendLine().AppendLine();
                return sb.ToString();
            }

            private static string CreateLoadGravity(LoadCase loadCase)
            {
                var g = loadCase.Gravity;                     
                return
                    $@"# gravitational acceleration for self-weight loading (global)
#,gX,gY,gZ
#,mm/s^2,mm/s^2,mm/s^2
, {g.X}, {g.Y}, {g.Z}
,,,,,,,,,,,,";
            }

            private static string CreateEnd()
            {
                return @",,,,,,,,,,,,
0,# number of desired dynamic modes
# End of input data file";
            }

            private static string ToStr(double d, int padLeft = 11)
            {
                return d.ToString("0.0000").PadLeft(padLeft);
            }

            private static string CreateHeader(string fileStamp)
            {

                return $@"""Generated by SteelX.SMS.Business.LoadAnalysis.Frame3DD.Frame3DDTextFileWriter units(N, mm, ton) {fileStamp}""
""# this template indicates units of Newton, millimeter, and tonne""
""# other units may be specified as desired""
# Portals
";
            }

            private static string CreateFrameElements(Input input)
            {



                //Pa = N/m^2 we want N/mm^2 = Pa * 10e6 = MPa
                var sb = new StringBuilder();
                sb.AppendLine($@"

# frame element data 
{input.FrameElements.Count}, ,,,,# number of frame elements
# e, n1, n2, Ax, Asy, Asz, Jxx, Iyy, Izz, E, G, roll, density
#., ., ., mm^2, mm^2, mm^2, mm^4,mm^4, mm^4, MPa=N/mm^2, MPa=N/mm^2, deg, tonne/mm^3
");

                for (int i = 0; i < input.FrameElements.Count; i++)
                {
                    var member = input.FrameElements[i];
                    sb.AppendLine(CreateFrameElementLine(i, member));
                }

                return sb.ToString().TrimEndExt(Environment.NewLine);
            }

            private static string CreateFrameElementLine(int idx, FrameElement frameElement)
            {
                //# e,n1,n2,Ax,Asy,Asz,Jxx,Iyy,Izz,E,G,roll,density
                //#.,.,.,mm^2,mm^2,mm^2,mm^4,mm^4,mm^4,MPa,MPa,deg,T/mm^3
                var ei = idx + 1;
                var n1 = frameElement.NodeIdx1 + 1;
                var n2 = frameElement.NodeIdx2 + 1;
                var ax = frameElement.Ax;
                var asy = frameElement.Asy;
                var asz = frameElement.Asz;
                var jxx = frameElement.Jx;
                var iyy = frameElement.Iy;
                var izz = frameElement.Iz;
                var e = frameElement.E; 
                var g = frameElement.G;
                var roll = 0;
                var density = frameElement.Density;
                return $"{ei}, {n1}, {n2}, {ax}, {asy}, {asz}, {jxx}, {iyy}, {izz}, {e}, {g}, {roll}, {density} ";
            }

            private static string CreateNodes(Input input)
            {
                var sb = new StringBuilder();
                sb.AppendLine($@"###### Nodes ######
{input.Nodes.Count},,,,, # number of nodes
#.node,X-coord,Y-coord,Z-coord,radius
#.,mm,mm,mm,deg");
                for (int i = 0; i < input.Nodes.Count; i++)
                {
                    var node = input.Nodes[i];                    
                    sb.AppendLine(CreateNodeLine(i + 1, node));
                }
                return sb.ToString().TrimEndExt(Environment.NewLine);
            }

            private static string CreateNodeLine(int lineNum, Node node)
            {                
                return $"{lineNum}, {ToStr(node.Position.X)}, {ToStr(node.Position.Y)}, {ToStr(node.Position.Z)}, {ToStr(node.Radius)}";
            }

            private static string CreateReactions(Input input)
            {
                var reactions = input.ReactionInputs;
                var sb = new StringBuilder();
                sb.AppendLine(
                    $@"

# reaction data 
{reactions.Count}, ,,,,# number of nodes with reactions
#.n,x,y,z,xx,yy,zz,1 = fixed 0 = free
");

                for (int i = 0; i < reactions.Count; i++)
                {
                    sb.AppendLine(CreateReactionLine(i, reactions[i]));
            }
                
                return sb.ToString().TrimEndExt(Environment.NewLine);
            }

        private static string CreateReactionLine(int nodeIndex, ReactionInput reaction)
        {
            int r(float f) => f < 0.5 ? 0 : 1;
            return
                $"{reaction.NodeIdx+1}, {r(reaction.Force.X)}, {r(reaction.Force.Y)}, {r(reaction.Force.Z)}, {r(reaction.Moment.X)}, {r(reaction.Moment.Y)}, {r(reaction.Moment.Z)},";
        }
    }
}
