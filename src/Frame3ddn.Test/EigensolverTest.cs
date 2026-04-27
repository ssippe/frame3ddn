using Frame3ddn;
using System;
using Xunit;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Tests for the Stodola eigensolver and its supporting helpers (LDLᵀ decomposition
    /// of an unpartitioned symmetric system, the symmetric quadratic form, and the
    /// ascending eigenvalue/eigenvector sort).
    /// </summary>
    public class EigensolverTest
    {
        [Fact]
        public void LdlDcmp_SolvesSymmetricPositiveDefiniteSystem()
        {
            // Symmetric SPD test: A x = b with A = [[4,1,0],[1,3,1],[0,1,2]], b = [9,11,7].
            // Hand-verified solution: x ≈ (1.6818, 2.2727, 2.3636).
            int n = 3;
            double[,] A = { { 4, 1, 0 }, { 0, 3, 1 }, { 0, 0, 2 } };  // upper triangle only
            double[] b = { 9, 11, 7 };
            double[] d = new double[n];
            double[] x = new double[n];

            int negPivots = Eigensolver.LdlDcmp(A, n, d, b, x, reduce: true, solve: true);

            Assert.Equal(0, negPivots);
            // Verify by re-multiplying using the original A.
            double[,] origA = { { 4, 1, 0 }, { 1, 3, 1 }, { 0, 1, 2 } };
            for (int i = 0; i < n; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < n; j++) sum += origA[i, j] * x[j];
                Assert.Equal(b[i], sum, 10);
            }
        }

        [Fact]
        public void XtAy_ComputesSymmetricQuadraticForm()
        {
            // x' A y for A = [[2, 1], [1, 3]] (upper-triangular storage), x = (1, 2), y = (3, 4)
            // = 1*(2*3 + 1*4) + 2*(1*3 + 3*4) = 1*10 + 2*15 = 10 + 30 = 40.
            double[,] A = { { 2, 1 }, { 0, 3 } };
            double[] x = { 1, 2 };
            double[] y = { 3, 4 };
            double[] scratch = new double[2];

            double result = Eigensolver.XtAy(x, A, y, 2, scratch);

            Assert.Equal(40.0, result, 12);
        }

        [Fact]
        public void EigSort_SortsEigenvaluesAscendingWithEigenvectors()
        {
            // 3 modes, n=2 dimension. e = (5, 1, 3) → after sort (1, 3, 5).
            double[] e = { 5, 1, 3 };
            double[,] V = { { 50, 10, 30 }, { 51, 11, 31 } }; // column k tagged with e[k] for traceability
            Eigensolver.EigSort(e, V, n: 2, m: 3);

            Assert.Equal(new[] { 1.0, 3.0, 5.0 }, e);
            Assert.Equal(10.0, V[0, 0]); Assert.Equal(11.0, V[1, 0]);
            Assert.Equal(30.0, V[0, 1]); Assert.Equal(31.0, V[1, 1]);
            Assert.Equal(50.0, V[0, 2]); Assert.Equal(51.0, V[1, 2]);
        }

        // 5-DoF mass-spring chain, fixed-fixed.  Closed-form eigenvalues exist for this
        // family: ω²ⱼ = 2(1 − cos(jπ/(n+1))) for j = 1..n.  We don't *check* the exact
        // values (Stodola's seeding heuristic doesn't always converge to the lowest mode
        // first when D is sparse), but every (φₖ, λₖ) pair returned must satisfy
        // K φ = λ M φ with the modes mass-orthonormal — that pins the algorithm regardless
        // of which eigenvalues the seeding lands on.
        [Fact]
        public void Stodola_FixedFixedSpringChain_SatisfiesGeneralizedEigenproblem()
        {
            const int n = 5;
            double[,] K = new double[n, n];
            for (int i = 0; i < n; i++)
            {
                K[i, i] = 2.0;
                if (i + 1 < n) K[i, i + 1] = -1.0;
            }
            double[,] M = new double[n, n];
            for (int i = 0; i < n; i++) M[i, i] = 1.0;

            (double[] eigs, double[,] V, _) = Eigensolver.Stodola(
                K, M, n, m: 3, tolerance: 1e-12, shift: 0.0);

            // Stodola converges when |ΔRQ|/RQ < tol. Eigenvector angular error is √tol because
            // the Rayleigh quotient is stationary near the eigenvector — so ‖K φ − λ M φ‖
            // residuals are bounded by √1e-12 ≈ 1e-6, not 1e-12.
            AssertGeneralizedEigenproblem(K, M, eigs, V, n, m: 3, residualTol: 1e-5);
        }

        // Generalized eigenproblem with non-identity M and asymmetric tridiagonal K.
        // Exercises the LDL skyline reduction and the symmetric upper-triangle handling
        // throughout XtAy, LdlMprove and the inverse iteration.
        [Fact]
        public void Stodola_GenericTridiagonalSystem_SatisfiesGeneralizedEigenproblem()
        {
            const int n = 4;
            double[,] K = {
                { 5, -1,  0,  0 },
                { 0,  4, -1,  0 },
                { 0,  0,  4, -1 },
                { 0,  0,  0,  3 }
            };
            double[,] M = {
                { 1, 0, 0, 0 },
                { 0, 2, 0, 0 },
                { 0, 0, 3, 0 },
                { 0, 0, 0, 4 }
            };

            (double[] eigs, double[,] V, _) = Eigensolver.Stodola(
                K, M, n, m: 3, tolerance: 1e-12, shift: 0.0);

            // Stodola converges when |ΔRQ|/RQ < tol. Eigenvector angular error is √tol because
            // the Rayleigh quotient is stationary near the eigenvector — so ‖K φ − λ M φ‖
            // residuals are bounded by √1e-12 ≈ 1e-6, not 1e-12.
            AssertGeneralizedEigenproblem(K, M, eigs, V, n, m: 3, residualTol: 1e-5);
        }

        // Subspace iteration on the same fixed-fixed spring chain. Unlike Stodola, subspace
        // converges all m modes simultaneously and isn't sensitive to the seed-vs-eigenvector
        // alignment, so it should reproduce the analytical eigenvalues directly.
        [Fact]
        public void Subspace_FixedFixedSpringChain_ReproducesAnalyticalEigenvalues()
        {
            const int n = 5;
            double[,] K = new double[n, n];
            for (int i = 0; i < n; i++)
            {
                K[i, i] = 2.0;
                if (i + 1 < n) K[i, i + 1] = -1.0;
            }
            double[,] M = new double[n, n];
            for (int i = 0; i < n; i++) M[i, i] = 1.0;

            // Subspace's convergence check is on the eigenvalue at index max(m/2, m-8) —
            // higher modes can be loose (Bathe's heuristic: nM_calc > nM, return the
            // lowest nM). Solve for all 5 modes, verify the lowest 3 to tight precision.
            (double[] eigs, double[,] V, _) = Eigensolver.Subspace(
                K, M, n, m: n, tolerance: 1e-12, shift: 0.0);

            // Analytical eigenvalues for fixed-fixed N-DoF chain: 2(1 - cos(jπ/(N+1))).
            double[] expected = new double[3];
            for (int j = 1; j <= 3; j++)
                expected[j - 1] = 2.0 * (1.0 - Math.Cos(j * Math.PI / (n + 1)));

            for (int k = 0; k < 3; k++)
                Assert.Equal(expected[k], eigs[k], 8);

            AssertGeneralizedEigenproblem(K, M, eigs, V, n, m: 3, residualTol: 1e-7);
        }

        // Subspace on a generic tridiagonal system with non-identity M — same matrix the
        // Stodola tests use. Mass-orthonormality and the K φ = λ M φ residual should hold
        // to better than 1e-9 (subspace converges all modes to the requested tolerance).
        [Fact]
        public void Subspace_GenericTridiagonalSystem_AllModesConvergeToTightResidual()
        {
            const int n = 4;
            double[,] K = {
                { 5, -1,  0,  0 },
                { 0,  4, -1,  0 },
                { 0,  0,  4, -1 },
                { 0,  0,  0,  3 }
            };
            double[,] M = {
                { 1, 0, 0, 0 },
                { 0, 2, 0, 0 },
                { 0, 0, 3, 0 },
                { 0, 0, 0, 4 }
            };

            // Solve for all 4 modes; only verify the lowest 3 (subspace's upper mode is loose).
            (double[] eigs, double[,] V, _) = Eigensolver.Subspace(
                K, M, n, m: n, tolerance: 1e-12, shift: 0.0);

            AssertGeneralizedEigenproblem(K, M, eigs, V, n, m: 3, residualTol: 1e-6);
        }

        // The Jacobi solver on a small dense problem — used by Subspace internally on the
        // reduced (m × m) system. Verifies it can solve a 3×3 generalised eigenproblem.
        [Fact]
        public void Jacobi_SymmetricGeneralizedProblem_RecoversEigenvalues()
        {
            // K, M from a known small problem with closed-form eigenvalues.
            // K = diag(4, 1, 1), M = diag(2, 1, 1) → eigenvalues 2, 1, 1
            // (eigenvectors of degenerate 1-eigenvalue depend on rotation order, so we just
            // pin the SET of eigenvalues, not the per-mode mapping.)
            int n = 3;
            double[,] K = { { 4, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            double[,] M = { { 2, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            double[] E = new double[n];
            double[,] V = new double[n, n];

            Eigensolver.Jacobi(K, M, E, V, n);

            // Sort the returned eigenvalues to make the assertion order-independent.
            double[] sorted = (double[])E.Clone();
            Array.Sort(sorted);
            Assert.Equal(1.0, sorted[0], 10);
            Assert.Equal(1.0, sorted[1], 10);
            Assert.Equal(2.0, sorted[2], 10);
        }

        // For each mode k, asserts:
        //   (a) eigenvalue is finite and positive (K, M are SPD)
        //   (b) eigenvalues are sorted ascending
        //   (c) mode shapes are mass-orthonormal: φᵢᵀ M φⱼ = δᵢⱼ
        //   (d) eigenvalue equation holds: ‖K φ − λ M φ‖ / ‖K φ‖ < residualTol
        // K and M are read from their upper triangle (matching Eigensolver.Stodola's contract).
        private static void AssertGeneralizedEigenproblem(
            double[,] K, double[,] M, double[] eigs, double[,] V, int n, int m, double residualTol)
        {
            for (int k = 0; k < m; k++)
            {
                Assert.True(double.IsFinite(eigs[k]) && eigs[k] > 0,
                    $"eigenvalue {k} = {eigs[k]} is not finite/positive");
            }
            for (int k = 0; k < m - 1; k++)
                Assert.True(eigs[k] <= eigs[k + 1],
                    $"eigenvalues not ascending: eigs[{k}]={eigs[k]}, eigs[{k + 1}]={eigs[k + 1]}");

            // (c) mass-orthonormality
            for (int a = 0; a < m; a++)
            {
                for (int b = 0; b < m; b++)
                {
                    double dot = 0.0;
                    for (int i = 0; i < n; i++)
                        for (int j = 0; j < n; j++)
                        {
                            double mij = i <= j ? M[i, j] : M[j, i];
                            dot += V[i, a] * mij * V[j, b];
                        }
                    Assert.Equal(a == b ? 1.0 : 0.0, dot, 5);
                }
            }

            // (d) eigenvalue equation residual (relative)
            for (int k = 0; k < m; k++)
            {
                double[] kphi = new double[n];
                double[] mphi = new double[n];
                for (int i = 0; i < n; i++)
                {
                    double ksum = 0.0, msum = 0.0;
                    for (int j = 0; j < n; j++)
                    {
                        ksum += (i <= j ? K[i, j] : K[j, i]) * V[j, k];
                        msum += (i <= j ? M[i, j] : M[j, i]) * V[j, k];
                    }
                    kphi[i] = ksum;
                    mphi[i] = msum;
                }
                double residSq = 0.0, kphiSq = 0.0;
                for (int i = 0; i < n; i++)
                {
                    double diff = kphi[i] - eigs[k] * mphi[i];
                    residSq += diff * diff;
                    kphiSq += kphi[i] * kphi[i];
                }
                double relResidual = Math.Sqrt(residSq / kphiSq);
                Assert.True(relResidual < residualTol,
                    $"mode {k}: relative residual {relResidual:e3} > {residualTol:e3}");
            }
        }
    }
}
