using System.Collections.Generic;

namespace Frame3ddn.Model
{
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
}