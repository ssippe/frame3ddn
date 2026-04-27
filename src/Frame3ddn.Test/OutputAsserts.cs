using System;
using System.Collections.Generic;
using System.Linq;
using Frame3ddn.Model;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Optional comparison settings for <see cref="OutputAsserts.OutputAssertEqual"/>.
    /// Defaults match the original SolvingTest comparator: 1% relative or 0.01 absolute,
    /// all output sections compared.
    /// </summary>
    public class OutputAssertOptions
    {
        public double RelativeTolerance { get; set; } = 0.01;
        public double AbsoluteTolerance { get; set; } = 0.01;
        /// <summary>Skip the FrameElementEndForces section (e.g. for Microstran .p1 references).</summary>
        public bool IgnoreFrameElementEndForces { get; set; }
        /// <summary>Skip the PeakFrameElementInternalForces section.</summary>
        public bool IgnorePeakForces { get; set; }
    }

    /// <summary>
    /// Assertion helpers for comparing solver <see cref="Output"/> values against parsed
    /// reference outputs. Mzz is scaled by 1e-5 in element-force comparisons so large
    /// nominal moment values do not dominate the relative-tolerance check.
    /// </summary>
    public static class OutputAsserts
    {
        public static void OutputAssertEqual(Output expected, Output actual, OutputAssertOptions options = null)
        {
            options = options ?? new OutputAssertOptions();
            if (expected.LoadCaseOutputs.Count != actual.LoadCaseOutputs.Count)
                throw new Exception(
                    $"LoadCaseOutputs count mismatch: expected {expected.LoadCaseOutputs.Count}, got {actual.LoadCaseOutputs.Count}");

            for (int i = 0; i < expected.LoadCaseOutputs.Count; i++)
                AssertLoadCaseEqual(expected.LoadCaseOutputs[i], actual.LoadCaseOutputs[i], i, options);
        }

        private static void AssertLoadCaseEqual(LoadCaseOutput expected, LoadCaseOutput actual, int lc, OutputAssertOptions o)
        {
            if (!o.IgnoreFrameElementEndForces)
            {
                AssertCountEqual(expected.FrameElementEndForces.Count, actual.FrameElementEndForces.Count,
                    $"LC{lc} FrameElementEndForces");
                for (int i = 0; i < expected.FrameElementEndForces.Count; i++)
                    AssertEqual(expected.FrameElementEndForces[i], actual.FrameElementEndForces[i], $"LC{lc} elementEndForce[{i}]", o);
            }

            if (!o.IgnorePeakForces)
                AssertMinMaxForcesEqual(expected.PeakFrameElementInternalForces, actual.PeakFrameElementInternalForces, lc, o);

            AssertCountEqual(expected.ReactionOutputs.Count, actual.ReactionOutputs.Count, $"LC{lc} ReactionOutputs");
            for (int i = 0; i < expected.ReactionOutputs.Count; i++)
                AssertEqual(expected.ReactionOutputs[i], actual.ReactionOutputs[i], $"LC{lc} reaction[{i}]", o);

            AssertCountEqual(expected.NodeDisplacements.Count, actual.NodeDisplacements.Count, $"LC{lc} NodeDisplacements");
            for (int i = 0; i < expected.NodeDisplacements.Count; i++)
                AssertEqual(expected.NodeDisplacements[i], actual.NodeDisplacements[i], $"LC{lc} displacement[{i}]", o);
        }

        private static void AssertMinMaxForcesEqual(
            IEnumerable<PeakFrameElementInternalForce> expected,
            IEnumerable<PeakFrameElementInternalForce> actual,
            int lc, OutputAssertOptions o)
        {
            List<PeakFrameElementInternalForce> e = expected.Where(f => f.IsMin.HasValue).ToList();
            List<PeakFrameElementInternalForce> a = actual.Where(f => f.IsMin.HasValue).ToList();
            AssertCountEqual(e.Count, a.Count, $"LC{lc} PeakFrameElementInternalForces (min/max only)");
            for (int i = 0; i < e.Count; i++)
                AssertEqual(e[i], a[i], $"LC{lc} peakForce[{i}]", o);
        }

        private static void AssertEqual(NodeDisplacement expected, NodeDisplacement actual, string label, OutputAssertOptions o)
        {
            if (expected.NodeIdx != actual.NodeIdx)
                throw new Exception($"{label} NodeIdx: expected {expected.NodeIdx}, got {actual.NodeIdx}");
            AssertVec3Close(expected.Displacement, actual.Displacement, label + ".Displacement", o);
            AssertVec3Close(expected.Rotation, actual.Rotation, label + ".Rotation", o);
        }

        private static void AssertEqual(ReactionOutput expected, ReactionOutput actual, string label, OutputAssertOptions o)
        {
            AssertDoubleClose(expected.NodeIdx, actual.NodeIdx, label + ".NodeIdx", o);
            AssertDoubleClose(expected.LoadcaseIdx, actual.LoadcaseIdx, label + ".LoadcaseIdx", o);
            AssertVec3Close(expected.F, actual.F, label + ".F", o);
            AssertVec3Close(expected.M, actual.M, label + ".M", o);
        }

        private static void AssertEqual(FrameElementEndForce expected, FrameElementEndForce actual, string label, OutputAssertOptions o)
        {
            AssertDoubleClose(expected.ElementIdx, actual.ElementIdx, label + ".ElementIdx", o);
            AssertDoubleClose(expected.NodeIdx, actual.NodeIdx, label + ".NodeIdx", o);
            AssertDoubleClose(expected.Nx, actual.Nx, label + ".Nx", o);
            AssertDoubleClose(expected.Vy, actual.Vy, label + ".Vy", o);
            AssertDoubleClose(expected.Vz, actual.Vz, label + ".Vz", o);
            AssertDoubleClose(expected.Txx, actual.Txx, label + ".Txx", o);
            AssertDoubleClose(expected.Myy, actual.Myy, label + ".Myy", o);
            AssertDoubleClose(expected.Mzz / 100000, actual.Mzz / 100000, label + ".Mzz/1e5", o);
        }

        private static void AssertEqual(PeakFrameElementInternalForce expected, PeakFrameElementInternalForce actual, string label, OutputAssertOptions o)
        {
            AssertDoubleClose(expected.ElementIdx, actual.ElementIdx, label + ".ElementIdx", o);
            AssertDoubleClose(expected.Nx, actual.Nx, label + ".Nx", o);
            AssertDoubleClose(expected.Vy, actual.Vy, label + ".Vy", o);
            AssertDoubleClose(expected.Vz, actual.Vz, label + ".Vz", o);
            AssertDoubleClose(expected.Txx, actual.Txx, label + ".Txx", o);
            AssertDoubleClose(expected.Myy, actual.Myy, label + ".Myy", o);
            AssertDoubleClose(expected.Mzz / 100000, actual.Mzz / 100000, label + ".Mzz/1e5", o);
            if (expected.IsMin != actual.IsMin)
                throw new Exception($"{label}.IsMin: expected {expected.IsMin}, got {actual.IsMin}");
        }

        private static void AssertVec3Close(Vec3 expected, Vec3 actual, string label, OutputAssertOptions o)
        {
            AssertDoubleClose(expected.X, actual.X, label + ".X", o);
            AssertDoubleClose(expected.Y, actual.Y, label + ".Y", o);
            AssertDoubleClose(expected.Z, actual.Z, label + ".Z", o);
        }

        private static void AssertDoubleClose(double expected, double actual, string label, OutputAssertOptions o)
        {
            if (!IsClose(expected, actual, o))
                throw new Exception($"{label}: expected {expected}, got {actual} (rtol {o.RelativeTolerance}, atol {o.AbsoluteTolerance})");
        }

        private static void AssertCountEqual(int expected, int actual, string label)
        {
            if (expected != actual)
                throw new Exception($"{label} count: expected {expected}, got {actual}");
        }

        private static bool IsClose(double a, double b, OutputAssertOptions o) =>
            Math.Abs(a - b) < Math.Abs(a) * o.RelativeTolerance || Math.Abs(a - b) < o.AbsoluteTolerance;
    }
}
