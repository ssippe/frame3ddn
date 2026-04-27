using Frame3ddn.Model;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses the upstream frame3dd <c>_out.CSV</c> result file format. Structurally similar
    /// to the plain <c>.out</c> format that <see cref="OutOutputParser"/> handles — same section
    /// titles, same column order, same per-LC layout — but with comma separators and quoted
    /// section headers. We preprocess to <c>.out</c>-compatible form (strip <c>"</c>, replace
    /// <c>,</c> with space) and delegate to <see cref="OutOutputParser.Parse"/>.
    /// </summary>
    public static class CsvOutputParser
    {
        public static List<LoadCaseOutput> Parse(string text)
        {
            // Some upstream _out.CSV files duplicate the PEAK section per load case (exC,
            // exE, exF, exG, exI all contain the same PEAK rows twice for a single LC).
            // De-duplicate by (ElementIdx, IsMin) so we get one row per element-extremum.
            return OutOutputParser.Parse(NormalizeCsv(text))
                .Select(lc => new LoadCaseOutput(
                    lc.RmsRelativeEquilibriumError,
                    lc.NodeDisplacements,
                    lc.FrameElementEndForces,
                    lc.ReactionOutputs,
                    lc.PeakFrameElementInternalForces
                        .GroupBy(p => (p.ElementIdx, p.IsMin))
                        .Select(g => g.First())
                        .ToList()))
                .ToList();
        }

        private static string NormalizeCsv(string text)
        {
            // Quotes wrap section titles (e.g. `"L O A D   C A S E ..."`) and per-cell strings
            // ("max"/"min" in the PEAK rows); commas are field separators. Strip quotes
            // outright (collapsing them to nothing rather than spaces so leading-quote section
            // titles still TrimStart correctly), and turn commas into spaces. We then trim
            // each line so OutParser's StartsWith section-title checks pass.
            StringBuilder sb = new StringBuilder(text.Length);
            using (StringReader r = new StringReader(text))
            {
                string line;
                while ((line = r.ReadLine()) != null)
                {
                    string cleaned = line.Replace("\"", string.Empty).Replace(',', ' ').TrimStart();
                    sb.AppendLine(cleaned);
                }
            }
            return sb.ToString();
        }
    }
}
