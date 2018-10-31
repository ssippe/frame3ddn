using System;
using System.Runtime.CompilerServices;

namespace Frame3ddn
{
    public class Common
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
            for (int i = 0; i < columns; ++i)
                array[i] = matrix[row, i];
            return array;
        }
    }
}