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
    }
}
