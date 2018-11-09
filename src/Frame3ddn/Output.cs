using System;
using System.Collections.Generic;

namespace Frame3ddn
{
    public class NodeDisplacement
    {

        public int LoadcaseIdx { get; }
        public int NodeIdx { get; }
        public Vec3 Displacement { get; }
        public Vec3 Rotation { get; }

        public NodeDisplacement(int loadcaseIdx, int nodeIdx, Vec3 displacement, Vec3 rotation)
        {
            LoadcaseIdx = loadcaseIdx;
            NodeIdx = nodeIdx;
            Displacement = displacement;
            Rotation = rotation;
        }
    }

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
    }

    public class ReactionOutput
    {
        public int LoadcaseIdx { get; }
        public int NodeIdx { get; }
        public Vec3 F { get; }
        public Vec3 M { get; }

        public ReactionOutput(int loadcaseIdx, int nodeIdx, Vec3 f, Vec3 m)
        {
            LoadcaseIdx = loadcaseIdx;
            NodeIdx = nodeIdx;
            F = f;
            M = m;
        }
    }

    public class PeakFrameElementInternalForce
    {
        public int LoadcaseIdx { get; }
        public int ElementIdx { get; }
        public bool IsMin { get; }
        public double Nx { get; }
        public double Vy { get; }
        public double Vz { get; }
        public double Txx { get; }
        public double Myy { get; }
        public double Mzz { get; }

        public PeakFrameElementInternalForce(int loadcaseIdx, int elementIdx, bool isMin, double nx, double vy, double vz, double txx, double myy, double mzz)
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
        }
    }

    public class LoadCaseOutput
    {
        public double RmsRelativeEquilibriumError { get; }
        public IReadOnlyList<NodeDisplacement> NodeDisplacements { get; }
        public IReadOnlyList<FrameElementEndForce> FrameElementEndForces { get; }
        public IReadOnlyList<ReactionOutput> ReactionOutputs { get; }
        public IReadOnlyList<PeakFrameElementInternalForce> PeakFrameElementInternalForces { get; }

        public LoadCaseOutput(double rmsRelativeEquilibriumError, IReadOnlyList<NodeDisplacement> nodeDisplacements, IReadOnlyList<FrameElementEndForce> frameElementEndForces, IReadOnlyList<ReactionOutput> reactionOutputs, IReadOnlyList<PeakFrameElementInternalForce> peakFrameElementInternalForces)
        {
            RmsRelativeEquilibriumError = rmsRelativeEquilibriumError;
            NodeDisplacements = nodeDisplacements;
            FrameElementEndForces = frameElementEndForces;
            ReactionOutputs = reactionOutputs;
            PeakFrameElementInternalForces = peakFrameElementInternalForces;
        }
    }

    public class Output
    {
        public string TextOutput { get; set; }
        public IReadOnlyList<LoadCaseOutput> LoadCaseOutputs { get; }
        public Output(string textOutput, IReadOnlyList<LoadCaseOutput> loadCaseOutputs)
        {
            TextOutput = textOutput;
            LoadCaseOutputs = loadCaseOutputs;
        }
    }
}

