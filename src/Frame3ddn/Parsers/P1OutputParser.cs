using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text.RegularExpressions;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses Microstran <c>.p1</c> analysis reports into per-load-case displacement and
    /// support-reaction tables. The sections of interest are
    /// <c>== N O D E   D I S P L A C E M E N T S ==</c> and
    /// <c>== S U P P O R T   R E A C T I O N S ==</c>; other sections (member forces,
    /// loading summaries, etc.) are ignored.
    /// Note: this is distinct from <see cref="OutOutputParser"/>, which parses the upstream
    /// frame3dd C program's <c>.out</c> output.
    /// </summary>
    public static class P1OutputParser
    {
        public static List<P1LoadCase> Parse(string text)
        {
            SortedDictionary<int, Accumulator> cases = new SortedDictionary<int, Accumulator>();
            Section section = Section.Unknown;
            int currentCaseId = -1;

            foreach (string raw in text.Split('\n'))
            {
                string line = raw.TrimEnd('\r');
                string trimmed = line.Trim();

                if (trimmed.StartsWith("=="))
                {
                    section = ClassifySection(trimmed);
                    currentCaseId = -1;
                    continue;
                }

                if (trimmed.StartsWith("CASE"))
                {
                    Match m = Regex.Match(trimmed, @"^CASE\s+(\d+)\s*:?\s*(.*)$");
                    if (m.Success)
                    {
                        currentCaseId = int.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture);
                        string title = m.Groups[2].Value.Trim();
                        Accumulator acc = GetOrAdd(cases, currentCaseId);
                        if (string.IsNullOrEmpty(acc.Title) && title.Length > 0) acc.Title = title;
                    }
                    continue;
                }

                if (section == Section.Unknown || currentCaseId < 0) continue;
                if (trimmed.Length == 0) continue;

                string[] tokens = trimmed.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                // Disp / Reaction rows: 7 tokens (nodeId + 6 floats). Skip header/unit rows.
                if (tokens.Length != 7) continue;
                if (!int.TryParse(tokens[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out int nodeId))
                    continue;

                double[] vals = new double[6];
                bool allParsed = true;
                for (int i = 0; i < 6; i++)
                {
                    if (!double.TryParse(tokens[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out vals[i]))
                    {
                        allParsed = false;
                        break;
                    }
                }
                if (!allParsed) continue;

                P1NodeRow row = new P1NodeRow(nodeId,
                    new Vec3(vals[0], vals[1], vals[2]),
                    new Vec3(vals[3], vals[4], vals[5]));

                Accumulator caseAcc = GetOrAdd(cases, currentCaseId);
                if (section == Section.Displacements) caseAcc.Displacements.Add(row);
                else caseAcc.Reactions.Add(row);
            }

            return cases.Values
                .Select(a => new P1LoadCase(a.Id, a.Title, a.Displacements, a.Reactions))
                .ToList();
        }

        private static Section ClassifySection(string headerLine)
        {
            if (headerLine.Contains("D I S P L A C E M E N T S")) return Section.Displacements;
            if (headerLine.Contains("S U P P O R T   R E A C T I O N S")) return Section.Reactions;
            return Section.Unknown;
        }

        private static Accumulator GetOrAdd(SortedDictionary<int, Accumulator> map, int id)
        {
            if (!map.TryGetValue(id, out Accumulator acc))
            {
                acc = new Accumulator(id);
                map[id] = acc;
            }
            return acc;
        }

        private enum Section { Unknown, Displacements, Reactions }

        private sealed class Accumulator
        {
            public int Id { get; }
            public string Title { get; set; } = "";
            public List<P1NodeRow> Displacements { get; } = new List<P1NodeRow>();
            public List<P1NodeRow> Reactions { get; } = new List<P1NodeRow>();
            public Accumulator(int id) { Id = id; }
        }
    }

    /// <summary>One load case parsed from a Microstran <c>.p1</c> report.</summary>
    public sealed class P1LoadCase
    {
        public int CaseId { get; }
        public string Title { get; }
        public IReadOnlyList<P1NodeRow> Displacements { get; }
        public IReadOnlyList<P1NodeRow> Reactions { get; }

        public P1LoadCase(int caseId, string title,
            IReadOnlyList<P1NodeRow> displacements, IReadOnlyList<P1NodeRow> reactions)
        {
            CaseId = caseId;
            Title = title;
            Displacements = displacements;
            Reactions = reactions;
        }
    }

    /// <summary>
    /// One row of a Microstran tabular section (displacements or reactions). Node IDs are
    /// 1-based as they appear in the file.
    /// </summary>
    public sealed class P1NodeRow
    {
        public int NodeId { get; }
        public Vec3 Linear { get; }
        public Vec3 Angular { get; }

        public P1NodeRow(int nodeId, Vec3 linear, Vec3 angular)
        {
            NodeId = nodeId;
            Linear = linear;
            Angular = angular;
        }
    }
}
