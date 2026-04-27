using System.Collections.Generic;

namespace Frame3ddn.Model
{
    public class Input
    {
        public string Title { get; }
        public bool IncludeShearDeformation { get; }
        public bool IncludeGeometricStiffness { get; }
        public float ExaggerateMeshDeformations { get; }
        public float ZoomScale { get; }
        public float XAxisIncrementForInternalForces { get; }

        public IReadOnlyList<Node> Nodes { get; }
        public IReadOnlyList<ReactionInput> ReactionInputs { get; }
        public IReadOnlyList<FrameElement> FrameElements { get; }
        public IReadOnlyList<LoadCase> LoadCases { get; }

        public Input(string title, IReadOnlyList<Node> nodes, IReadOnlyList<FrameElement> frameElements,
            IReadOnlyList<ReactionInput> reactionInputs, IReadOnlyList<LoadCase> loadCases,
            bool includeShearDeformation, bool includeGeometricStiffness, float exaggerateMeshDeformations,
            float zoomScale, float xAxisIncrementForInternalForces)
        {
            Title = title;
            Nodes = nodes;
            FrameElements = frameElements;
            ReactionInputs = reactionInputs;
            LoadCases = loadCases;
            IncludeShearDeformation = includeShearDeformation;
            IncludeGeometricStiffness = includeGeometricStiffness;
            ExaggerateMeshDeformations = exaggerateMeshDeformations;
            ZoomScale = zoomScale;
            XAxisIncrementForInternalForces = xAxisIncrementForInternalForces;
        }
    }
}
