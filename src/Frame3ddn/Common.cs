namespace Frame3ddn
{
    public class Common
    {
        public const int Zvert = 1;

        public static bool isZeroVector(Vec3 vec3)
        {
            if (vec3.X == 0 && vec3.Y == 0 && vec3.Z == 0)
                return true;
            return false;
        }
    }
}