using System;
using System.Collections.Generic;

namespace Frame3ddn
{
    public struct Vec3
    {
        public double X;
        public double Y;
        public double Z;
    }

    public class Node
    {
        public Node(Vec3 position, double radius)
        {
            Position = position;
            Radius = radius;
        }

        public Vec3 Position { get; }
        public double Radius { get; }
    }

    public class Reaction
    {
        public Reaction(Vec3 position, Vec3 r, bool isFixed)
        {
            Position = position;
            R = r;
            IsFixed = isFixed;
        }

        public Vec3 Position { get; }
        public Vec3 R { get; }
        public bool IsFixed { get; }
    }

    public class FrameElement
    {
        public int NodeIdx1 { get; }
        public int NodeIdx2 { get; }
        public double Ax { get; }
        public double Asy { get; }
        public double Jx { get; }
        public double Iy { get; }
        public double Iz { get; }
        public double E { get; }
        public double G { get; }
        public double Roll { get; }
        public double Density { get; }

        public FrameElement(int nodeIdx1, int nodeIdx2, double ax, double asy, double jx, double iy, double iz, double e, double g, double roll, double density)
        {
            NodeIdx1 = nodeIdx1;
            NodeIdx2 = nodeIdx2;
            Ax = ax;
            Asy = asy;
            Jx = jx;
            Iy = iy;
            Iz = iz;
            E = e;
            G = g;
            Roll = roll;
            Density = density;
        }
    }

    public class NodeLoad
    {
        public int NodeIdx { get; }
        public Vec3 Load { get; }
        public Vec3 Moment { get; }

        public NodeLoad(int nodeIdx, Vec3 load, Vec3 moment)
        {
            NodeIdx = nodeIdx;
            Load = load;
            Moment = moment;
        }
    }

    public class UniformLoad
    {
        public int ElementIdx { get; }
        public Vec3 Load { get; }

        public UniformLoad(int elementIdx, Vec3 load)
        {
            ElementIdx = elementIdx;
            Load = load;
        }        
    }

    public class TrapLoad
    {
        public int ElementIdx { get; }
        public Vec3 LocationStart { get; }
        public Vec3 LocationEnd { get; }
        public Vec3 LoadStart { get; }
        public Vec3 LoadEnd { get; }
        public TrapLoad(int elementIdx, Vec3 locationStart, Vec3 locationEnd, Vec3 loadStart, Vec3 loadEnd)
        {
            ElementIdx = elementIdx;
            LocationStart = locationStart;
            LocationEnd = locationEnd;
            LoadStart = loadStart;
            LoadEnd = loadEnd;
        }
    }

    public class LoadCase
    {
        public Vec3 Gravity { get; }
        public IReadOnlyList<NodeLoad> NodeLoads { get; }
        public IReadOnlyList<UniformLoad> UniformLoads { get; }
        public IReadOnlyList<TrapLoad> TrapLoads { get; }

        public LoadCase(Vec3 gravity,
            IReadOnlyList<NodeLoad> nodeLoads,
            IReadOnlyList<UniformLoad> uniformLoads,
            IReadOnlyList<TrapLoad> trapLoads)
        {
            Gravity = gravity;
            NodeLoads = nodeLoads;
            UniformLoads = uniformLoads;
            TrapLoads = trapLoads;
        }


    }

    public class Input
    {
        public IReadOnlyList<Node> Nodes { get; }
        public IReadOnlyList<Reaction> Reactions { get; }
        public IReadOnlyList<FrameElement> FrameElements { get; }
        public IReadOnlyList<LoadCase> LoadCases { get; }

        public Input(IReadOnlyList<Node> nodes, IReadOnlyList<FrameElement> frameElements, IReadOnlyList<Reaction> reactions, IReadOnlyList<LoadCase> loadCases)
        {
            Nodes = nodes;
            FrameElements = frameElements;
            Reactions = reactions;
            LoadCases = loadCases;
        }

        
    }
}
