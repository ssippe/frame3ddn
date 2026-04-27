using Frame3ddn.Model;
using System.Collections.Generic;
using Xunit;

namespace Frame3ddn.Test
{
    /// <summary>
    /// Tests for the mass-matrix building blocks (lumped + consistent element matrices and
    /// the system-level <see cref="Frame3ddn.Frame3dd.AssembleM"/>). Each test pins a
    /// hand-computable invariant so a future regression in the assembly path produces a
    /// targeted failure rather than a vague comparison-test diff.
    /// </summary>
    public class MassMatrixTest
    {
        // For a horizontal element along global X with no extras, the lumped mass matrix
        // puts d*Ax*L/2 on each translational diagonal at each end.
        [Fact]
        public void LumpedMatrix_HorizontalBeam_PutsHalfMassOnEachTranslationalDiagonal()
        {
            List<Vec3> xyz = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(10, 0, 0) };
            double[,] m = new double[12, 12];
            const double L = 10.0;
            const double Ax = 2.0;
            const double d = 0.5;

            Frame3ddn.Frame3dd.LumpedM(m, xyz, L, n1: 0, n2: 1,
                Ax, J: 0, Iy: 0, Iz: 0, p: 0, d: d, EMs: 0);

            double half = d * Ax * L / 2.0;
            int[] transDofs = { 0, 1, 2, 6, 7, 8 };
            foreach (int i in transDofs)
                Assert.Equal(half, m[i, i], 12);
        }

        // Consistent mass conserves total mass: summing the (n1, n2) translational block
        // along one axis must equal d*Ax*L.
        [Fact]
        public void ConsistentMatrix_HorizontalBeam_ConservesTranslationalMass()
        {
            List<Vec3> xyz = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(10, 0, 0) };
            float[] r = new float[2];
            double[,] m = new double[12, 12];
            const double L = 10.0;
            const double Ax = 2.0;
            const double d = 0.5;

            Frame3ddn.Frame3dd.ConsistentM(m, xyz, r, L, n1: 0, n2: 1,
                Ax, J: 0, Iy: 0, Iz: 0, p: 0, d: d, EMs: 0);

            // For a horizontal-X element with Iy=Iz=0 the consistent matrix in global coords
            // matches the local-form translational block: m[0,0]=mt/3, m[0,6]=m[6,0]=mt/6, m[6,6]=mt/3.
            double mt = d * Ax * L;
            Assert.Equal(mt / 3.0, m[0, 0], 10);
            Assert.Equal(mt / 6.0, m[0, 6], 10);
            Assert.Equal(mt / 6.0, m[6, 0], 10);
            Assert.Equal(mt / 3.0, m[6, 6], 10);
            Assert.Equal(mt, m[0, 0] + m[0, 6] + m[6, 0] + m[6, 6], 10);
        }

        // For a vertical-Z element (n1 at origin, n2 at +Z), the local x-axis is global z,
        // local y stays as global y, local z becomes -global x. The rotational diagonal of
        // the lumped matrix should therefore project as (rz, ry, po) onto global (x, y, z).
        [Fact]
        public void LumpedMatrix_VerticalBeam_ProjectsRotationalInertiaOntoGlobalAxes()
        {
            List<Vec3> xyz = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(0, 0, 5) };
            double[,] m = new double[12, 12];
            const double L = 5.0;
            const double Ax = 1.0;
            const double Iy = 10.0;
            const double Iz = 20.0;
            const double J = 30.0;
            const double d = 1.0;

            Frame3ddn.Frame3dd.LumpedM(m, xyz, L, n1: 0, n2: 1,
                Ax, J: J, Iy: Iy, Iz: Iz, p: 0, d: d, EMs: 0);

            double ry = d * Iy * L / 2.0;
            double rz = d * Iz * L / 2.0;
            double po = d * L * J / 2.0;

            // Rotational diagonal at node 1 (DoFs 3,4,5 = global xx,yy,zz):
            //   xx ← local-z axis (rotated to global -x) → contribution rz
            //   yy ← local-y axis (still global y)        → contribution ry
            //   zz ← local-x axis (rotated to global z)   → contribution po
            Assert.Equal(rz, m[3, 3], 10);
            Assert.Equal(ry, m[4, 4], 10);
            Assert.Equal(po, m[5, 5], 10);

            // Same projection at node 2.
            Assert.Equal(rz, m[9, 9], 10);
            Assert.Equal(ry, m[10, 10], 10);
            Assert.Equal(po, m[11, 11], 10);
        }

        // Consistent mass on a tilted element should still be symmetric (the symmetry
        // enforcement in ConsistentM mops up Atma rounding, but the underlying structure
        // must already be near-symmetric).
        [Fact]
        public void ConsistentMatrix_TiltedBeam_IsSymmetric()
        {
            List<Vec3> xyz = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(3, 4, 0) }; // 3-4-5 triangle, L=5
            float[] r = new float[2];
            double[,] m = new double[12, 12];

            Frame3ddn.Frame3dd.ConsistentM(m, xyz, r, L: 5.0, n1: 0, n2: 1,
                Ax: 1.0, J: 5.0, Iy: 2.0, Iz: 3.0, p: 0, d: 0.5, EMs: 0);

            for (int i = 0; i < 12; i++)
                for (int j = 0; j < 12; j++)
                    Assert.Equal(m[i, j], m[j, i], 10);
        }

        // AssembleM should add NMs/NMx/NMy/NMz onto the per-node diagonal.
        [Fact]
        public void AssembleM_AddsExtraNodeInertiaToDiagonal()
        {
            List<Vec3> xyz = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(10, 0, 0) };
            int nN = 2, nE = 1, DoF = 6 * nN;
            float[] r = new float[nN];

            List<float> NMs = new List<float> { 0.5f, 0.0f };
            List<float> NMx = new List<float> { 1.5f, 0.0f };
            List<float> NMy = new List<float> { 2.5f, 0.0f };
            List<float> NMz = new List<float> { 3.5f, 0.0f };

            double[,] M = Frame3ddn.Frame3dd.AssembleM(
                DoF, nN, nE,
                xyz, r,
                L: new List<double> { 10.0 },
                N1: new List<int> { 0 }, N2: new List<int> { 1 },
                Ax: new List<float> { 1.0f }, Jx: new List<float> { 0 },
                Iy: new List<float> { 0 }, Iz: new List<float> { 0 },
                p: new List<float> { 0 },
                density: new List<float> { 0.1f },
                EMs: new List<float> { 0.0f },
                NMs: NMs, NMx: NMx, NMy: NMy, NMz: NMz,
                lump: true);

            // Translational diagonal at node 0: half element mass (0.1f*1*10/2 ≈ 0.5) + NMs[0] (0.5) ≈ 1.0
            // Single-precision 0.1f means we're working at ~1e-7 precision, not double precision.
            Assert.Equal(1.0, M[0, 0], 6);
            Assert.Equal(1.0, M[1, 1], 6);
            Assert.Equal(1.0, M[2, 2], 6);

            // Rotational diagonals at node 0 receive the inertia extras (rotational inertia
            // of the element itself is zero because Iy=Iz=Jx=0).
            Assert.Equal(1.5, M[3, 3], 6);
            Assert.Equal(2.5, M[4, 4], 6);
            Assert.Equal(3.5, M[5, 5], 6);

            // Node 1 has no extras → diagonal is just the half element mass.
            Assert.Equal(0.5, M[6, 6], 6);
        }
    }
}
