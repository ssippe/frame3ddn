using System;

namespace Frame3ddn
{
    /// <summary>
    /// Generalised eigenvalue solver for the modal problem K φ = ω² M φ.
    /// Mirrors upstream <c>eig.c</c> + <c>HPGmatrix.c</c> in pslack/frame3dd.
    ///
    /// The caller is responsible for masking out restrained DoFs (upstream does this in
    /// <c>main.c</c> by zeroing reaction-row off-diagonals and putting a large stiffness on
    /// the corresponding diagonals); this class operates on the full <c>n × n</c> system.
    /// </summary>
    internal static class Eigensolver
    {
        /// <summary>
        /// LDLᵀ decomposition of a symmetric matrix stored in the upper triangle.
        /// Mirrors upstream <c>ldl_dcmp</c> (HPGmatrix.c). On <c>reduce==true</c> the matrix
        /// is overwritten with <c>L</c> in the strict lower triangle while <paramref name="d"/>
        /// receives the diagonal of <c>D</c>. On <c>solve==true</c> the system <c>A x = b</c>
        /// is solved via forward + diagonal + backward substitution.
        /// </summary>
        /// <returns>Number of negative pivots encountered (used by the Sturm sequence count).</returns>
        public static int LdlDcmp(double[,] A, int n, double[] d, double[] b, double[] x, bool reduce, bool solve)
        {
            int pd = 0;

            if (reduce)
            {
                for (int j = 0; j < n; j++)
                {
                    int m = 0;
                    for (int i = 0; i < j; i++)         // scan the skyline
                    {
                        if (A[i, j] == 0.0) m++;
                        else break;
                    }

                    for (int i = m; i < j; i++)
                    {
                        A[j, i] = A[i, j];
                        for (int k = m; k < i; k++)
                            A[j, i] -= A[j, k] * A[i, k];
                    }

                    d[j] = A[j, j];
                    for (int i = m; i < j; i++)
                        d[j] -= A[j, i] * A[j, i] / d[i];
                    for (int i = m; i < j; i++)
                        A[j, i] /= d[i];

                    if (d[j] == 0.0)
                    {
                        Console.WriteLine($" ldl_dcmp(): zero found on diagonal d[{j}] = {d[j]}");
                        return pd;
                    }
                    if (d[j] < 0.0) pd--;
                }
            }

            if (solve)
            {
                // Forward substitution: same elimination steps applied to {x = b}.
                for (int i = 0; i < n; i++)
                {
                    x[i] = b[i];
                    for (int j = 0; j < i; j++) x[i] -= A[i, j] * x[j];
                }
                for (int i = 0; i < n; i++) x[i] /= d[i];
                // Backward substitution: A is preserved (we use only the strict lower triangle).
                for (int i = n - 1; i > 0; i--)
                    for (int j = 0; j < i; j++) x[j] -= A[i, j] * x[i];
            }

            return pd;
        }

        /// <summary>
        /// Iterative refinement for the LDLᵀ solve. Mirrors upstream <c>ldl_mprove</c>.
        /// <paramref name="A"/> must already hold the LDLᵀ factors (call <see cref="LdlDcmp"/>
        /// with <c>reduce=true</c> first); <paramref name="d"/> must hold the diagonal of D.
        /// On entry <paramref name="rmsResid"/> is the previous residual; on exit it is the
        /// new RMS residual if the solution improved (return value true), else unchanged.
        /// </summary>
        public static bool LdlMprove(double[,] A, int n, double[] d, double[] b, double[] x, ref double rmsResid)
        {
            double[] resid = new double[n];

            for (int i = 0; i < n; i++)
            {
                double sdp = b[i];
                for (int j = 0; j < n; j++)
                    sdp -= (i <= j ? A[i, j] : A[j, i]) * x[j];
                resid[i] = sdp;
            }

            // Solve A·δ = residual using the existing factorisation.
            LdlDcmp(A, n, d, resid, resid, reduce: false, solve: true);

            double newRms = 0.0;
            for (int i = 0; i < n; i++) newRms += resid[i] * resid[i];
            newRms = Math.Sqrt(newRms / n);

            if (newRms / rmsResid < 0.90)        // good improvement — accept
            {
                for (int i = 0; i < n; i++) x[i] += resid[i];
                rmsResid = newRms;
                return true;
            }
            return false;
        }

        /// <summary>
        /// Computes the symmetric quadratic form <c>xᵀ A y</c>. <paramref name="A"/> is read
        /// from its upper triangle. Matches upstream <c>xtAy</c>. <paramref name="scratch"/>
        /// is a length-<paramref name="n"/> working vector that holds <c>A·y</c> on return.
        /// </summary>
        public static double XtAy(double[] x, double[,] A, double[] y, int n, double[] scratch)
        {
            for (int i = 0; i < n; i++)
            {
                scratch[i] = 0.0;
                for (int j = 0; j < n; j++)
                    scratch[i] += (i <= j ? A[i, j] : A[j, i]) * y[j];
            }
            double xay = 0.0;
            for (int i = 0; i < n; i++) xay += x[i] * scratch[i];
            return xay;
        }

        /// <summary>
        /// Sorts eigenvalues ascending with their eigenvectors. <paramref name="V"/> is
        /// an n × m matrix of mode shapes (columns); the first <paramref name="m"/> entries
        /// of <paramref name="e"/> are sorted in place. Mirrors upstream <c>eigsort</c>.
        /// </summary>
        public static void EigSort(double[] e, double[,] V, int n, int m)
        {
            for (int i = 0; i < m - 1; i++)
            {
                int k = i;
                double p = e[k];
                for (int j = i + 1; j < m; j++)
                {
                    if (e[j] <= p)
                    {
                        k = j;
                        p = e[k];
                    }
                }
                if (k != i)
                {
                    e[k] = e[i];
                    e[i] = p;
                    for (int j = 0; j < n; j++)
                    {
                        double tmp = V[j, i];
                        V[j, i] = V[j, k];
                        V[j, k] = tmp;
                    }
                }
            }
        }

        /// <summary>
        /// Stodola inverse-iteration eigensolver. Solves <c>K φ = ω² M φ</c> for the lowest
        /// <paramref name="m"/> modes via mode-by-mode inverse iteration with Gram–Schmidt
        /// purging. Mirrors upstream <c>stodola</c> in eig.c.
        ///
        /// <paramref name="K"/> and <paramref name="M"/> are not modified — the routine
        /// works on internal copies.
        /// </summary>
        /// <returns>Eigenvalues (ω², ascending), n × m mode shapes (mass-normalised), and
        /// the total inverse-iteration count summed across all modes.</returns>
        public static (double[] eigenvalues, double[,] modeShapes, int iterations) Stodola(
            double[,] K, double[,] M, int n, int m, double tolerance, double shift)
        {
            if (m > n)
                throw new ArgumentException(
                    $"Number of modes ({m}) must be ≤ problem dimension ({n})");

            double[,] Kw = new double[n, n];
            double[,] Mw = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    Kw[i, j] = K[i, j];
                    Mw[i, j] = M[i, j];
                }

            // Apply spectral shift (upper triangle only).
            if (shift != 0.0)
                for (int i = 0; i < n; i++)
                    for (int j = i; j < n; j++)
                        Kw[i, j] += shift * Mw[i, j];

            double[] dDiag = new double[n];
            double[] u = new double[n];
            double[] v = new double[n];
            double[] dCol = new double[n];
            double[] cPurge = new double[m];
            double[] scratch = new double[n];

            // Factor K once: Kw will hold the LDLᵀ factors (L below diagonal, D in dDiag).
            LdlDcmp(Kw, n, dDiag, u, v, reduce: true, solve: false);

            // Compute D = K⁻¹ M, column-by-column.
            double[,] dynMatrix = new double[n, n];
            for (int j = 0; j < n; j++)
            {
                for (int i = 0; i < n; i++)
                    v[i] = (i <= j ? Mw[i, j] : Mw[j, i]);

                LdlDcmp(Kw, n, dDiag, v, dCol, reduce: false, solve: true);

                // Iterative refinement of the solve. Capped at 30 iterations — for an
                // ill-conditioned K each refinement only halves the residual, and the
                // unbounded loop in upstream relies on natural convergence which can take
                // hundreds of iterations on larger systems and dominate the runtime.
                double rms = 1.0;
                for (int it = 0; it < 30; it++)
                    if (!LdlMprove(Kw, n, dDiag, v, dCol, ref rms)) break;

                for (int i = 0; i < n; i++) dynMatrix[i, j] = dCol[i];
            }

            // Track the smallest n diagonals of D (used to seed each mode).
            double dMax = double.MinValue;
            double dMin;
            for (int i = 0; i < n; i++) if (dynMatrix[i, i] > dMax) dMax = dynMatrix[i, i];
            dMin = dMax;
            for (int i = 0; i < n; i++) if (dynMatrix[i, i] < dMin) dMin = dynMatrix[i, i];

            double[] eigenvalues = new double[m];
            double[,] V = new double[n, m];
            int iter = 0;
            // dOld starts at the global maximum and is monotonically lowered each iteration to
            // pick the next-largest D[i,i] under it. iSeed is *preserved* across iterations —
            // when no diagonal qualifies (later modes), upstream re-uses the previous seed and
            // relies on lower-mode purging to differentiate the converged eigenvector.
            double dOld = dMax;
            int iSeed = 0;

            for (int k = 0; k < m; k++)
            {
                double dPick = dMin;
                for (int i = 0; i < n; i++)
                {
                    u[i] = 0.0;
                    if (dynMatrix[i, i] < dOld && dynMatrix[i, i] > dPick)
                    {
                        dPick = dynMatrix[i, i];
                        iSeed = i;
                    }
                }
                u[iSeed] = 1.0;
                if (iSeed + 1 < n) u[iSeed + 1] = 1.0e-4;
                dOld = dPick;

                // Mass-normalise the seed.
                double vMv = XtAy(u, Mw, u, n, scratch);
                double scale = 1.0 / Math.Sqrt(vMv);
                for (int i = 0; i < n; i++) u[i] *= scale;

                // Purge previously-found modes from the seed.
                PurgeLowerModes(u, V, Mw, k, n, scratch, cPurge);

                vMv = XtAy(u, Mw, u, n, scratch);
                scale = 1.0 / Math.Sqrt(vMv);
                for (int i = 0; i < n; i++) u[i] *= scale;
                double rq = XtAy(u, Kw, u, n, scratch);

                double rqOld;
                int modeIter = 0;
                bool converged;
                do
                {
                    // v = D u
                    for (int i = 0; i < n; i++)
                    {
                        double s = 0.0;
                        for (int j = 0; j < n; j++) s += dynMatrix[i, j] * u[j];
                        v[i] = s;
                    }

                    vMv = XtAy(v, Mw, v, n, scratch);
                    scale = 1.0 / Math.Sqrt(vMv);
                    for (int i = 0; i < n; i++) v[i] *= scale;

                    PurgeLowerModes(v, V, Mw, k, n, scratch, cPurge);

                    vMv = XtAy(v, Mw, v, n, scratch);
                    scale = 1.0 / Math.Sqrt(vMv);
                    for (int i = 0; i < n; i++) u[i] = v[i] * scale;

                    rqOld = rq;
                    rq = XtAy(u, Kw, u, n, scratch);
                    iter++;
                    modeIter++;

                    converged = Math.Abs(rq - rqOld) / rq <= tolerance;
                    if (!converged && modeIter > 1000)
                    {
                        // Mode didn't converge to the requested tolerance — use the current
                        // best estimate and continue with subsequent modes. Upstream aborts
                        // the whole run; we keep the partial results so the caller still gets
                        // the converged lower modes (which the caller typically asks for more
                        // of than it actually needs, expecting some of the higher computed
                        // modes to be loose).
                        Console.WriteLine(
                            $"  warning: Stodola mode {k + 1} did not converge in {modeIter} iterations " +
                            $"(rel. error = {Math.Abs(rq - rqOld) / rq:e3} > {tolerance:e3}) — using current estimate");
                        break;
                    }
                }
                while (!converged);

                for (int i = 0; i < n; i++) V[i, k] = v[i];
                eigenvalues[k] = rq > shift ? rq - shift : shift - rq;
            }

            EigSort(eigenvalues, V, n, m);
            return (eigenvalues, V, iter);
        }

        /// <summary>
        /// Removes the components of <paramref name="trial"/> that lie along the previously
        /// converged mode shapes (columns 0..<paramref name="kCurrent"/>-1 of <paramref name="V"/>).
        /// Implements the Gram–Schmidt-style purge from upstream's stodola loop.
        /// </summary>
        private static void PurgeLowerModes(
            double[] trial, double[,] V, double[,] M,
            int kCurrent, int n, double[] scratch, double[] coeffs)
        {
            double[] modeColumn = scratch;
            for (int j = 0; j < kCurrent; j++)
            {
                for (int i = 0; i < n; i++) modeColumn[i] = V[i, j];
                double c = 0.0;
                // c = modeColumn' M trial   — inline to avoid clobbering scratch via XtAy.
                for (int i = 0; i < n; i++)
                {
                    double s = 0.0;
                    for (int p = 0; p < n; p++)
                        s += (i <= p ? M[i, p] : M[p, i]) * trial[p];
                    c += modeColumn[i] * s;
                }
                coeffs[j] = c;
            }
            for (int j = 0; j < kCurrent; j++)
                for (int i = 0; i < n; i++)
                    trial[i] -= coeffs[j] * V[i, j];
        }
    }
}
