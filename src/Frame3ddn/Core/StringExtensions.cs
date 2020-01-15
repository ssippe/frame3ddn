using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn.Core
{
    public static class StringExtensions
    {
        public static string JoinString(this IEnumerable<string> stringList, string separator = ", ")
        {
            if (stringList == null)
                return null;
            return string.Join(separator, stringList);
        }

        public static string TrimEndExt(this string target, string trim)
        {
            if (string.IsNullOrEmpty(trim))
            {
                return target;
            }
            while (target.EndsWith(trim))
            {
                target = target.Substring(0, target.Length - trim.Length);
            }
            return target;
        }
    }
}
