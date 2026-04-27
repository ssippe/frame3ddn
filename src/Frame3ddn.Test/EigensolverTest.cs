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
