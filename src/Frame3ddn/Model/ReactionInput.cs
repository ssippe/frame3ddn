using System.Linq;

namespace Frame3ddn.Model
{
    public class ReactionInput
    {
        public ReactionInput(int num, Vec3Float position, Vec3Float r)
        {
            Number = num;
            Position = position;
            R = r;
        }

        public Vec3Float Position { get; }
        public Vec3Float R { get; }
        public int Number { get; set; }//This can't be replaced by index

        public static ReactionInput Parse(string inputString)
        {
            int[] data = System.Text.RegularExpressions.Regex.Split(inputString, @"\s{1,}").Select(int.Parse).ToArray();
            return new ReactionInput(data[0] - 1, new Vec3Float(data[1], data[2], data[3]), new Vec3Float(data[4], data[5], data[6]));
        }
    }
}