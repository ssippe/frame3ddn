using System;
using System.Collections.Generic;
using System.Linq;

namespace Frame3ddn
{
    public class Coordtrans
    {
        public static double[] coordTrans(List<Vec3> xyz, double L, int n1, int n2, double p)
        {
            double[] t = new double[9];
            double Cx, Cy, Cz, den, Cp, Sp;
            for (int i = 0; i < t.Length; i++)
            {
                t[i] = 0.0;
            }

            Cx = (xyz[n2].X - xyz[n1].X) / L;
            Cy = (xyz[n2].Y - xyz[n1].Y) / L;
            Cz = (xyz[n2].Z - xyz[n1].Z) / L;

            Cp = Math.Cos(p);
            Sp = Math.Sin(p);

            if (Common.Zvert == 1)
            {
                if (Math.Abs(Cz) == Math.Abs(1))
                {
                    t[2] = Cz;//><
                    t[3] = -Cz * Sp;//><
                    t[4] = Cp;//><
                    t[6] = -Cz * Cp;//><
                    t[7] = -Sp;//><
                }
                else
                {
                    den = Math.Sqrt(1.0 - Cz * Cz);

                    t[0] = Cx;//><
                    t[1] = Cy;//><
                    t[2] = Cz;//><

                    t[3] = (-Cx * Cz * Sp - Cy * Cp) / den;//><
                    t[4] = (-Cy * Cz * Sp + Cx * Cp) / den;//><
                    t[5] = Sp * den;//><

                    t[6] = (-Cx * Cz * Cp + Cy * Sp) / den;//><
                    t[7] = (-Cy * Cz * Cp - Cx * Sp) / den;//><
                    t[8] = Cp * den;//><
                }
            }

            return t;
        }

        public static double CalculateSQDistance(Vec3 nodePosition1, Vec3 nodePosition2)
        {
            return Math.Sqrt(Math.Pow(nodePosition1.X - nodePosition2.X, 2) + Math.Pow(nodePosition1.Y - nodePosition2.Y, 2) + Math.Pow(
                       nodePosition1.Z - nodePosition2.Z, 2));
        }

        public static double[,] Atma(double[] t, double[,] m, double r1, double r2)
        {
            double[,] a = new double[12, 12];
            double[,] ma = new double[12, 12];
            int i, j, k;

            for (i = 0; i < 4; i++)
            {
                a[3 * i + 0, 3 * i + 0] = t[0];
                a[3 * i + 0, 3 * i + 1] = t[1];
                a[3 * i + 0, 3 * i + 2] = t[2];
                a[3 * i + 1, 3 * i + 0] = t[3];
                a[3 * i + 1, 3 * i + 1] = t[4];
                a[3 * i + 1, 3 * i + 2] = t[5];
                a[3 * i + 2, 3 * i + 0] = t[6];
                a[3 * i + 2, 3 * i + 1] = t[7];
                a[3 * i + 2, 3 * i + 2] = t[8];
            }

            for (j = 0; j < 12; j++)            /*  MT = M T     */
                for (i = 0; i < 12; i++)
                    for (k = 0; k < 12; k++)
                        ma[i, j] += m[i, k] * a[k, j];

            for (i = 0; i < 12; i++)
                for (j = 0; j < 12; j++)
                    m[j, i] = m[i, j] = 0.0;
            return m;

        }
    }
}