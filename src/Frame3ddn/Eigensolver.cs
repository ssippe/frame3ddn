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
        /// Subspace iteration eigensolver — solves <c>K φ = ω² M φ</c> for the lowest
        /// <paramref name="m"/> modes by repeatedly projecting the full system onto an
        /// <paramref name="m"/>-dimensional trial subspace, solving the reduced problem with
        /// <see cref="Jacobi"/>, then updating the subspace. Mirrors upstream <c>subspace</c>
        /// in eig.c. Converges much faster than <see cref="Stodola"/> on production-size
        /// systems because every iteration improves all <paramref name="m"/> modes
        /// simultaneously.
        ///
        /// <paramref name="K"/> and <paramref name="M"/> are not modified.
        /// </summary>
        public static (double[] eigenvalues, double[,] modeShapes, int iterations) Subspace(
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
            double[,] V = new double[n, m];
            double[,] Xb = new double[n, m];
            double[,] Kb = new double[m, m];
            double[,] Mb = new double[m, m];
            double[,] Qb = new double[m, m];
            double[] eigenvalues = new double[m];

            // Factor K (now with shift) once.
            LdlDcmp(Kw, n, dDiag, u, v, reduce: true, solve: false);

            // Approximate eigenvalues from diag(K)/diag(M) — used to pick m well-spread seeds.
            double[] dKM = new double[n];
            for (int i = 0; i < n; i++)
            {
                if (Mw[i, i] <= 0.0)
                    throw new InvalidOperationException(
                        $"Subspace: M[{i},{i}] = {Mw[i, i]} ≤ 0 — mass matrix not positive definite");
                dKM[i] = Kw[i, i] / Mw[i, i];
            }

            // Pick m distinct DoF indices in ascending order of dKM[i] — these are
            // approximations of the m lowest ω². Mirrors upstream's nested seeding loop.
            int[] idx = new int[m];
            for (int k = 0; k < m; k++) idx[k] = -1;
            double kmOld = 0.0;
            for (int k = 0; k < m; k++)
            {
                double km = dKM[0];
                for (int i = 0; i < n; i++)
                {
                    if (kmOld <= dKM[i] && dKM[i] <= km)
                    {
                        bool unused = true;
                        for (int j = 0; j < k; j++) if (i == idx[j]) { unused = false; break; }
                        if (unused) { km = dKM[i]; idx[k] = i; }
                    }
                }
                if (idx[k] == -1)
                {
                    // Fallback: take next index after the largest used so far.
                    int largestUsed = idx[0];
                    for (int j = 1; j < k; j++) if (idx[j] > largestUsed) largestUsed = idx[j];
                    idx[k] = largestUsed + 1;
                    if (idx[k] >= n) idx[k] = n - 1;
                    km = dKM[idx[k]];
                }
                kmOld = km;
            }

            // Seed V from the picked indices: V[idx[k], k] = 1.0 plus 0.2 on two neighbouring
            // DoFs at the same node (upstream's heuristic). The neighbour offsets depend on
            // the seed's role in the 6-DoF group at that node so the trial vector spans X/Y/Z
            // (or xx/yy/zz) mass.
            for (int k = 0; k < m; k++)
            {
                V[idx[k], k] = 1.0;
                int posInGroup = idx[k] % 6;     // 0=X, 1=Y, 2=Z, 3=xx, 4=yy, 5=zz
                int o1, o2;
                switch (posInGroup)
                {
                    case 0: o1 = 1; o2 = 2; break;
                    case 1: o1 = -1; o2 = 1; break;
                    case 2: o1 = -1; o2 = -2; break;
                    case 3: o1 = 1; o2 = 2; break;
                    case 4: o1 = -1; o2 = 1; break;
                    default: o1 = -1; o2 = -2; break;     // case 5
                }
                int n1 = idx[k] + o1, n2 = idx[k] + o2;
                if (n1 >= 0 && n1 < n) V[n1, k] = 0.2;
                if (n2 >= 0 && n2 < n) V[n2, k] = 0.2;
            }

            // The convergence check tracks the error on a "well-converged" eigenvalue
            // (Bathe's heuristic — the lowest of the modes that haven't been pulled
            // toward the upper end of the requested range by subspace projection error).
            int convMode = (m / 2 > m - 8) ? m / 2 : m - 8;
            if (convMode < 0) convMode = 0;
            if (convMode >= m) convMode = m - 1;

            int iter = 0;
            double wOld = 0.0;
            double error;
            do
            {
                // Xb = K⁻¹ M V (column-by-column LDL solves; refinement omitted — direct
                // LDL solve at machine precision is plenty for subspace iteration's outer
                // convergence loop).
                for (int k = 0; k < m; k++)
                {
                    ProdABj(Mw, V, v, n, k);
                    LdlDcmp(Kw, n, dDiag, v, u, reduce: false, solve: true);
                    for (int i = 0; i < n; i++) Xb[i, k] = u[i];
                }

                // Project K and M onto the subspace.
                XtAx(Kw, Xb, Kb, n, m);
                XtAx(Mw, Xb, Mb, n, m);

                // Solve the reduced (m × m) generalised eigenproblem.
                Jacobi(Kb, Mb, eigenvalues, Qb, m);

                // V = Xb · Qb — the new trial subspace.
                ProdAB(Xb, Qb, V, n, m, m);

                EigSort(eigenvalues, V, n, m);

                if (eigenvalues[convMode] == 0.0)
                    throw new InvalidOperationException(
                        $"Subspace: zero frequency at convergence mode {convMode + 1}");
                error = Math.Abs(eigenvalues[convMode] - wOld) / eigenvalues[convMode];
                wOld = eigenvalues[convMode];
                iter++;

                if (iter > 1000)
                    throw new InvalidOperationException(
                        $"Subspace: iteration limit exceeded (rel. error = {error:e3} > {tolerance:e3})");
            }
            while (error > tolerance);

            // Undo spectral shift on eigenvalues.
            if (shift != 0.0)
                for (int k = 0; k < m; k++)
                    eigenvalues[k] = eigenvalues[k] > shift
                        ? eigenvalues[k] - shift
                        : shift - eigenvalues[k];

            return (eigenvalues, V, iter);
        }

        /// <summary>
        /// <c>u = A · B[:,j]</c>, where <paramref name="A"/> is symmetric (read from upper
        /// triangle) and <paramref name="B"/> is general. Mirrors upstream <c>prodABj</c>.
        /// </summary>
        public static void ProdABj(double[,] A, double[,] B, double[] u, int n, int j)
        {
            for (int i = 0; i < n; i++) u[i] = 0.0;
            for (int i = 0; i < n; i++)
                for (int k = 0; k < n; k++)
                    u[i] += (i <= k ? A[i, k] : A[k, i]) * B[k, j];
        }

        /// <summary>
        /// General matrix-matrix multiply: <c>C = A · B</c> with sizes
        /// <paramref name="I"/>×<paramref name="J"/>, <paramref name="J"/>×<paramref name="K"/>,
        /// <paramref name="I"/>×<paramref name="K"/>. Mirrors upstream <c>prodAB</c>.
        /// </summary>
        public static void ProdAB(double[,] A, double[,] B, double[,] C, int I, int J, int K)
        {
            for (int i = 0; i < I; i++)
                for (int k = 0; k < K; k++) C[i, k] = 0.0;
            for (int i = 0; i < I; i++)
                for (int k = 0; k < K; k++)
                    for (int j = 0; j < J; j++)
                        C[i, k] += A[i, j] * B[j, k];
        }

        /// <summary>
        /// Symmetric triple product: <c>C = X' A X</c> where <paramref name="A"/> is
        /// <paramref name="N"/>×<paramref name="N"/> symmetric (upper triangle), <paramref name="X"/>
        /// is <paramref name="N"/>×<paramref name="J"/>, and the <paramref name="J"/>×<paramref name="J"/>
        /// result <paramref name="C"/> is symmetrised on output. Mirrors upstream <c>xtAx</c>.
        /// </summary>
        public static void XtAx(double[,] A, double[,] X, double[,] C, int N, int J)
        {
            double[,] AX = new double[N, J];
            for (int i = 0; i < J; i++)
                for (int j = 0; j < J; j++) C[i, j] = 0.0;
            for (int i = 0; i < N; i++)
                for (int j = 0; j < J; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < N; k++)
                        s += (i <= k ? A[i, k] : A[k, i]) * X[k, j];
                    AX[i, j] = s;
                }
            for (int i = 0; i < J; i++)
                for (int j = 0; j < J; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < N; k++) s += X[k, i] * AX[k, j];
                    C[i, j] = s;
                }
            // Enforce symmetry — accumulated rounding can leave tiny asymmetries.
            for (int i = 0; i < J; i++)
                for (int j = i; j < J; j++)
                    C[i, j] = C[j, i] = 0.5 * (C[i, j] + C[j, i]);
        }

        /// <summary>
        /// Paired Jacobi rotation on a symmetric matrix <paramref name="A"/>. Updates
        /// <c>A → Pᵀ A P</c> where <c>diag(P) = 1</c>, <c>P[i,j] = α</c>, <c>P[j,i] = β</c>.
        /// Used in tandem on K and M during the Jacobi sweep so both <c>K[i,j]</c> and
        /// <c>M[i,j]</c> can be eliminated by the same (α, β) rotation.
        /// </summary>
        public static void Rotate(double[,] A, int n, double alpha, double beta, int i, int j)
        {
            double[] Ai = new double[n];
            double[] Aj = new double[n];
            for (int k = 0; k < n; k++)
            {
                Ai[k] = A[i, k];
                Aj[k] = A[j, k];
            }

            double Aii = A[i, i];
            double Ajj = A[j, j];
            double Aij = A[i, j];

            A[i, i] = Aii + 2.0 * beta * Aij + beta * beta * Ajj;
            A[j, j] = Ajj + 2.0 * alpha * Aij + alpha * alpha * Aii;

            for (int k = 0; k < n; k++)
            {
                if (k != i && k != j)
                {
                    A[k, i] = A[i, k] = Ai[k] + beta * Aj[k];
                    A[k, j] = A[j, k] = Aj[k] + alpha * Ai[k];
                }
            }
            A[j, i] = A[i, j] = 0.0;
        }

        /// <summary>
        /// Solves the generalised eigenproblem <c>K φ = ω² M φ</c> for symmetric K and SPD M
        /// via paired Jacobi rotations. Used by <see cref="Subspace"/> on the small (m × m)
        /// reduced system. Mirrors upstream <c>jacobi</c> in eig.c.
        ///
        /// On return: <paramref name="E"/> holds the eigenvalues, <paramref name="V"/> the
        /// mass-normalised eigenvectors. K and M are destroyed.
        /// </summary>
        public static void Jacobi(double[,] K, double[,] M, double[] E, double[,] V, int n)
        {
            // V starts as identity.
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++) V[i, j] = i == j ? 1.0 : 0.0;

            // Up to 2n sweeps. Upstream uses tol=0 inside the inner test, so any non-zero
            // off-diagonal triggers a rotation — convergence is reached when no rotations
            // happen during a sweep, which it always does within 2n sweeps in practice.
            for (int sweep = 0; sweep < 2 * n; sweep++)
            {
                for (int d = 1; d < n; d++)
                {
                    for (int i = 0; i < n - d; i++)
                    {
                        int j = i + d;
                        double Kij = K[i, j];
                        double Mij = M[i, j];

                        // Skip if both off-diagonals are already (effectively) zero.
                        bool kSig = Kij * Kij > 0.0 && K[i, i] * K[j, j] != 0.0
                                    && Kij * Kij / (K[i, i] * K[j, j]) > 0.0;
                        bool mSig = Mij * Mij > 0.0 && M[i, i] * M[j, j] != 0.0
                                    && Mij * Mij / (M[i, i] * M[j, j]) > 0.0;
                        if (!kSig && !mSig) continue;

                        // Determine α, β so that the joint rotation eliminates both
                        // off-diagonal entries simultaneously. From Bathe (1982).
                        double Kii = K[i, i] * Mij - Kij * M[i, i];
                        double Kjj = K[j, j] * Mij - Kij * M[j, j];
                        double s = K[i, i] * M[j, j] - K[j, j] * M[i, i];
                        double gamma = s >= 0
                            ? 0.5 * s + Math.Sqrt(0.25 * s * s + Kii * Kjj)
                            : 0.5 * s - Math.Sqrt(0.25 * s * s + Kii * Kjj);
                        if (gamma == 0.0) continue;
                        double alpha = Kjj / gamma;
                        double beta = -Kii / gamma;

                        Rotate(K, n, alpha, beta, i, j);
                        Rotate(M, n, alpha, beta, i, j);

                        for (int k = 0; k < n; k++)
                        {
                            double Vki = V[k, i];
                            double Vkj = V[k, j];
                            V[k, i] = Vki + beta * Vkj;
                            V[k, j] = Vkj + alpha * Vki;
                        }
                    }
                }
            }

            // Mass-normalise eigenvectors.
            for (int j = 0; j < n; j++)
            {
                double Mjj = Math.Sqrt(M[j, j]);
                if (Mjj == 0.0) continue;
                for (int i = 0; i < n; i++) V[i, j] /= Mjj;
            }

            // Eigenvalues from the diagonal Rayleigh quotients.
            for (int j = 0; j < n; j++)
                E[j] = M[j, j] != 0.0 ? K[j, j] / M[j, j] : 0.0;
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
