using Frame3ddn.Model;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;

namespace Frame3ddn.Parsers
{
    /// <summary>
    /// Parses the text output produced by the upstream <c>frame3dd</c> C program (the same
    /// format <see cref="Writers.OutWriter.OutputDataToString"/> emits — typically <c>.out</c> or
    /// <c>.txt</c> files) into a list of <see cref="LoadCaseOutput"/>. Used by tests to
    /// compare solver runs against reference output files.
    /// Note: this is distinct from <see cref="P1Parser"/>, which parses Microstran <c>.p1</c>
    /// reports.
    /// </summary>
    public static class OutParser
    {
        public static List<LoadCaseOutput> Parse(string text)
        {
            List<LoadCaseOutput> outputLines = new List<LoadCaseOutput>();
            StringReader reader = new StringReader(text);
            while (true)
            {
                string currentLine = ReadUntil(reader, s => s.StartsWith("L O A D   C A S E"));
                List<PeakFrameElementInternalForce> forceLines = new List<PeakFrameElementInternalForce>();
                List<NodeDisplacement> displacementLines = new List<NodeDisplacement>();
                List<ReactionOutput> reactionLines = new List<ReactionOutput>();
                List<FrameElementEndForce> frameElementEndForceLines = new List<FrameElementEndForce>();

                if (currentLine == null)
                    return outputLines;
                int loadCaseIdx =
                    int.Parse(Regex.Match(currentLine, @"L O A D   C A S E\s*(\d+)\s*O F\s*(\d+)").Groups[1].Value) - 1;

                // node displacements
                if (ReadUntil(reader,
                    s => s.StartsWith("N O D E   D I S P L A C E M E N T S  					(global)")) == null)
                    break;
                if (reader.ReadLine() == null) //skip headers
                    break;
                while (true)
                {
                    NodeDisplacement resultLine = NodeDisplacement.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    displacementLines.Add(resultLine);
                }

                // frame element end forces
                if (ReadUntil(reader,
                        s => s.Contains("Elmnt")) == null)
                    break;
                while (true)
                {
                    FrameElementEndForce resultLine = FrameElementEndForce.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    frameElementEndForceLines.Add(resultLine);
                }

                // reactions
                if (ReadUntil(reader,
                    s => s.Contains("Node")) == null)
                    break;
                while (true)
                {
                    ReactionOutput resultLine = ReactionOutput.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    reactionLines.Add(resultLine);
                }

                // internal forces
                if (ReadUntil(reader,
                    s => s.StartsWith("P E A K   F R A M E   E L E M E N T   I N T E R N A L   F O R C E S")) == null)
                    break;
                if (reader.ReadLine() == null) //skip headers
                    break;
                while (true)
                {
                    PeakFrameElementInternalForce resultLine = PeakFrameElementInternalForce.FromLine(reader.ReadLine(), loadCaseIdx);
                    if (resultLine == null)
                        break;
                    forceLines.Add(resultLine);
                }

                outputLines.Add(new LoadCaseOutput(0, displacementLines, frameElementEndForceLines, reactionLines,
                    forceLines));
            }
            return outputLines;
        }

        private static string ReadUntil(StringReader reader, Func<string, bool> stop)
        {
            string currentLine;
            while (true)
            {
                currentLine = reader.ReadLine();
                if (currentLine == null)
                    return null;
                if (stop(currentLine))
                    return currentLine;
            }
        }
    }
}
