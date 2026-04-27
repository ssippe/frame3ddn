using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses the text output produced by the upstream <c>frame3dd</c> C program (the same
    /// format <see cref="Writers.OutWriter.OutputDataToString"/> emits — typically <c>.out</c> or
    /// <c>.txt</c> files) into a list of <see cref="LoadCaseOutput"/>. Used by tests to
    /// compare solver runs against reference output files.
    /// Note: this is distinct from <see cref="P1OutputParser"/>, which parses Microstran <c>.p1</c>
    /// reports.
    /// </summary>
    public static class OutOutputParser
    {
        public static List<LoadCaseOutput> Parse(string text)
        {
            // Skip past the input-data preamble (which has its own per-LC sub-headers
            // duplicating "L O A D   C A S E N O F  M") to the analysis-result section,
            // marked by the "E L A S T I C   S T I F F N E S S   A N A L Y S I S" banner
            // OutWriter emits. If absent, scan the whole file.
            int sentinelOffset = text.IndexOf("E L A S T I C   S T I F F N E S S   A N A L Y S I S", StringComparison.Ordinal);
            string analysisText = sentinelOffset >= 0 ? text.Substring(sentinelOffset) : text;
            List<string> lines = ReadAllLines(analysisText);

            List<LoadCaseOutput> outputLines = new List<LoadCaseOutput>();

            int i = 0;
            // Find first L O A D   C A S E.
            while (i < lines.Count && !IsLoadCaseHeader(lines[i])) i++;

            while (i < lines.Count)
            {
                int loadCaseIdx = ParseLoadCaseIdx(lines[i]);
                i++;

                List<NodeDisplacement> displacements = new List<NodeDisplacement>();
                List<FrameElementEndForce> elementEndForces = new List<FrameElementEndForce>();
                List<ReactionOutput> reactions = new List<ReactionOutput>();
                List<PeakFrameElementInternalForce> peakForces = new List<PeakFrameElementInternalForce>();

                // Walk forward consuming whichever sections appear, until either the next
                // L O A D   C A S E header or EOF terminates this load case. Missing sections
                // are silently treated as empty.
                while (i < lines.Count && !IsLoadCaseHeader(lines[i]))
                {
                    string line = lines[i];

                    // Match full section titles only (the trailing "(global)/(local)" tag varies
                    // by section so we test on the leading words). Input-data sections like
                    // "T E M P E R A T U R E   C H A N G E S" or "N O D A L   L O A D S" are
                    // intentionally skipped — we only collect computed-output rows.
                    if (line.StartsWith("N O D E   D I S P L A C E M E N T S"))
                    {
                        i += 2;     // skip section header + column header
                        i = ConsumeRows(lines, i, ln => NodeDisplacement.FromLine(ln, loadCaseIdx), displacements);
                    }
                    else if (line.StartsWith("F R A M E   E L E M E N T   E N D   F O R C E S"))
                    {
                        i += 2;     // skip section header + column header
                        i = ConsumeRows(lines, i, ln => FrameElementEndForce.FromLine(ln, loadCaseIdx), elementEndForces);
                    }
                    else if (line.StartsWith("R E A C T I O N S"))
                    {
                        i += 2;     // skip section header + column header
                        i = ConsumeRows(lines, i, ln => ReactionOutput.FromLine(ln, loadCaseIdx), reactions);
                    }
                    else if (line.StartsWith("P E A K   F R A M E   E L E M E N T   I N T E R N A L   F O R C E S"))
                    {
                        i += 2;     // skip section header + column header
                        i = ConsumeRows(lines, i, ln => PeakFrameElementInternalForce.FromLine(ln, loadCaseIdx), peakForces);
                    }
                    else
                    {
                        i++;        // R M S, blank, or anything else
                    }
                }

                outputLines.Add(new LoadCaseOutput(0, displacements, elementEndForces, reactions, peakForces));
            }

            return outputLines;
        }

        /// <summary>
        /// Parses the modal-analysis section of the upstream <c>.out</c> format (or the
        /// section emitted by <see cref="Writers.OutWriter.ModalResultsToString"/>) into
        /// a list of <see cref="ModalResult"/>. Empty if no modal section is present.
        /// </summary>
        /// <remarks>
        /// Tolerates upstream's "modal participation factor" lines (tab-indented under each
        /// MODE header) and the "Total Mass / Structural Mass / N O D A L   M A S S E S"
        /// preamble. Eigenvalue is reconstructed from the frequency: ω² = (2π·f)².
        /// </remarks>
        public static List<ModalResult> ParseModalResults(string text)
        {
            List<ModalResult> results = new List<ModalResult>();

            int sentinelOffset = text.IndexOf("M O D A L   A N A L Y S I S", StringComparison.Ordinal);
            if (sentinelOffset < 0) return results;

            List<string> lines = ReadAllLines(text.Substring(sentinelOffset));

            // The MODE block is: "  MODE %5d:   f= %f Hz,  T= %f sec".
            Regex modeHeader = new Regex(
                @"^\s*MODE\s+(\d+):\s+f=\s*([0-9.eE+\-]+)\s*Hz");

            int i = 0;
            while (i < lines.Count)
            {
                Match m = modeHeader.Match(lines[i]);
                if (!m.Success) { i++; continue; }

                int modeIdx = int.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture) - 1;
                double freqHz = double.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
                double omegaSq = (2.0 * Math.PI * freqHz) * (2.0 * Math.PI * freqHz);
                i++;

                // Skip everything up to and including the "Node X-dsp ..." column header
                // (participation-factor lines, blank lines).
                while (i < lines.Count && !lines[i].TrimStart().StartsWith("Node "))
                {
                    if (modeHeader.IsMatch(lines[i])) break;  // empty mode block
                    i++;
                }
                if (i < lines.Count && lines[i].TrimStart().StartsWith("Node ")) i++;

                List<double> shape = new List<double>();
                while (i < lines.Count)
                {
                    if (modeHeader.IsMatch(lines[i])) break;
                    // Mode-shape row: " %5d %11.3e %11.3e %11.3e %11.3e %11.3e %11.3e".
                    // Recognise by "leading whitespace + integer + 6 floats".
                    string[] tokens = lines[i].Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tokens.Length < 7
                        || !int.TryParse(tokens[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out _))
                    {
                        i++;
                        // Stop when we hit a non-data line that isn't a continuation.
                        if (lines[i - 1].StartsWith("L O A D") || lines[i - 1].StartsWith("M O D A L")) break;
                        continue;
                    }
                    for (int k = 1; k <= 6; k++)
                        shape.Add(double.Parse(tokens[k], NumberStyles.Float, CultureInfo.InvariantCulture));
                    i++;
                }

                results.Add(new ModalResult(modeIdx, freqHz, omegaSq, shape));
            }

            return results;
        }

        private static bool IsLoadCaseHeader(string line) => line.StartsWith("L O A D   C A S E");

        private static int ParseLoadCaseIdx(string headerLine) =>
            int.Parse(Regex.Match(headerLine, @"L O A D   C A S E\s*(\d+)\s*O F\s*(\d+)").Groups[1].Value) - 1;

        /// <summary>
        /// Calls <paramref name="parse"/> on consecutive lines starting at <paramref name="start"/>,
        /// appending non-null results to <paramref name="output"/>. Stops at the first line that
        /// fails to parse (returns null) or that begins a new section / load case — without
        /// consuming that terminator line, so the caller can dispatch on it.
        /// </summary>
        private static int ConsumeRows<T>(List<string> lines, int start, Func<string, T> parse, List<T> output)
            where T : class
        {
            int i = start;
            while (i < lines.Count)
            {
                string line = lines[i];
                if (IsLoadCaseHeader(line) || IsSectionHeader(line)) return i;
                T row = parse(line);
                if (row == null) return i + 1;  // skip past the (probably blank) terminator
                output.Add(row);
                i++;
            }
            return i;
        }

        private static bool IsSectionHeader(string line) =>
            line.StartsWith("N O D E   D I S P L A C E M E N T S") ||
            line.StartsWith("F R A M E   E L E M E N T   E N D   F O R C E S") ||
            line.StartsWith("R E A C T I O N S") ||
            line.StartsWith("P E A K   F R A M E   E L E M E N T   I N T E R N A L   F O R C E S");

        private static List<string> ReadAllLines(string text)
        {
            List<string> result = new List<string>();
            using (StringReader r = new StringReader(text))
            {
                string line;
                while ((line = r.ReadLine()) != null) result.Add(line);
            }
            return result;
        }
    }
}
