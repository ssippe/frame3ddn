using System.Collections.Generic;

namespace Frame3ddn.Model
{
    /// <summary>
    /// One mode of a modal analysis: natural frequency and mass-normalised mode shape.
    /// Mirrors a single entry of upstream's frame3dd "M O D A L   A N A L Y S I S   R E S U L T S"
    /// section.
    /// </summary>
    public class ModalResult
    {
        /// <summary>Mode index, 0-based. The lowest mode is index 0.</summary>
        public int ModeIndex { get; }

        /// <summary>Natural frequency [Hz]. Computed as <c>√ω² / (2π)</c>.</summary>
        public double FrequencyHz { get; }

        /// <summary>Eigenvalue ω² [rad²/time²] from the generalised problem K φ = ω² M φ.</summary>
        public double Eigenvalue { get; }

        /// <summary>
        /// Mass-normalised mode shape: a vector of length 6·nN, indexed by
        /// <c>[6·nodeIdx + dof]</c> with <c>dof ∈ {x, y, z, xx, yy, zz}</c>.
        /// </summary>
        public IReadOnlyList<double> ModeShape { get; }

        public ModalResult(int modeIndex, double frequencyHz, double eigenvalue, IReadOnlyList<double> modeShape)
        {
            ModeIndex = modeIndex;
            FrequencyHz = frequencyHz;
            Eigenvalue = eigenvalue;
            ModeShape = modeShape;
        }
    }
}
