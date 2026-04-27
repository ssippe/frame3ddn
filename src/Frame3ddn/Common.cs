using System;

namespace Frame3ddn
{
    public static class Common
    {
        public const int Zvert = 1;


        /// <summary>
        /// Strict equality with 0.0, mirroring upstream frame3dd's <c>A[i][j] == 0.0</c>
        /// and <c>q[i]</c>/<c>r[i]</c> tests. Earlier the C# port rounded to one decimal
        /// place, which false-positived non-zero stiffness-matrix entries during the
        /// LDL skyline scan and produced wrong reactions on near-truss geometries.
        /// </summary>
        public static bool IsDoubleZero(double num) => num == 0.0;

        public static T[] GetRow<T>(T[,] matrix, int row)
        {
            int columns = matrix.GetLength(1);
            var array = new T[columns];
            for (int i = 0; i < columns; i++)
                array[i] = matrix[row, i];
            return array;
        }

        public static T[,] GetArray<T>(T[,,] matrix, int row)
        {
            int columns1 = matrix.GetLength(1);
            int columns2 = matrix.GetLength(2);
            var array = new T[columns1, columns2];
            for (int i = 0; i < columns1; i++)
                for (int j = 0; j < columns2; j++)
                    array[i, j] = matrix[row, i, j];
            return array;
        }

        public static string Format(this int num, int padding)
        {
            return num.ToString().Replace(",", "").PadLeft(padding, ' ');
        }

        public static string Format(this float num, double padding)
        {
            string[] count = padding.ToString().Split('.');
            int c0 = int.Parse(count[0]);
            int c1 = 0;
            if (count.Length == 2)
            {
                c1 = int.Parse(count[1]);
            }
            return num.ToString("N" + c1).Replace(",", "").PadLeft(c0);
        }

        public static string EFormat(this float num, double padding)
        {
            string[] count = padding.ToString().Split('.');
            int c0 = int.Parse(count[0]);
            int c1 = 0;
            if (count.Length == 2)
            {
                c1 = int.Parse(count[1]);
            }

            string mid = "";
            for (int i = 0; i < c1; i++)
            {
                mid += "#";
            }
            return num.ToString("0." + mid + "E+000").Replace(",", "").PadLeft(c0);
        }

        public static string Format(this double num, double padding)
        {
            string[] count = padding.ToString().Split('.');
            int c0 = int.Parse(count[0]);
            int c1 = 0;
            if (count.Length == 2)
            {
                c1 = int.Parse(count[1]);
            }
            return num.ToString("N" + c1).Replace(",", "").PadLeft(c0);
        }

        /// <summary>
        /// to double without adding to the number
        /// </summary>
        /// <example>
        /// (double)26974.05F==26974.05078125
        /// 26974.05F.ToDoubleExt()==26974.05
        /// </example>
        public static double ToDoubleExt(this float f) => (double)(decimal)f;
    }
}