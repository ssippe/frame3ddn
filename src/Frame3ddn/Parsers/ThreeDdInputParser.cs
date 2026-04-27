using Frame3ddn.Model;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses the legacy whitespace-delimited frame3dd input format (typically <c>.3dd</c>
    /// files). Records carry the same data as the CSV format — only the field separator
    /// differs (whitespace instead of commas) — so the per-record parsing reuses
    /// <see cref="CsvInputParser.ParseLines"/>.
    /// </summary>
    public static class ThreeDdInputParser
    {
        public static Input Parse(StreamReader sr) => CsvInputParser.ParseLines(GetNoCommentInput(sr));

        private static List<string> GetNoCommentInput(StreamReader sr)
        {
            // Collapse runs of whitespace, strip everything after a '#' comment marker, drop
            // blank lines. The output is a list of one-record-per-line, single-space-separated
            // strings — exactly the shape CsvParser.ParseLines expects.
            List<string> result = new List<string>();
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                int hash = line.IndexOf('#');
                if (hash >= 0) line = line.Substring(0, hash);
                line = Regex.Replace(line, @"\s+", " ").Trim();
                if (line.Length > 0) result.Add(line);
            }
            return result;
        }
    }
}
