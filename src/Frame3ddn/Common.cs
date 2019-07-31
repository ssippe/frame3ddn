using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace Frame3ddn
{
    public static class Common
    {
        public const int Zvert = 1;

        public static bool isZeroVector(Vec3Float vec3)
        {
            if (isDoubleZero(vec3.X) && isDoubleZero(vec3.Y) && isDoubleZero(vec3.Z))
                return true;
            return false;
        }

        public static bool isDoubleZero(double num)
        {
            if (Math.Round(num, 1) != 0.0)
                return false;
            return true;
        }

        public static T[] GetRow<T>(T[,] matrix, int row)
        {
            var columns = matrix.GetLength(1);
            var array = new T[columns];
            for (int i = 0; i < columns; i++)
                array[i] = matrix[row, i];
            return array;
        }

        public static T[,] GetArray<T>(T[,,] matrix, int row)
        {
            var columns1 = matrix.GetLength(1);
            var columns2 = matrix.GetLength(2);
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
        public static double ToDoubleExt(this float f) => (double) (decimal) f;
    }
}