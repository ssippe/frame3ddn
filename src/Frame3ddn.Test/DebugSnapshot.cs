using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;

namespace Frame3ddn.Test
{
    /// <summary>
    /// When a debugger is attached, snapshot test inputs/outputs to disk under
    /// <c>TestResults/yyyyMMdd-HHmmss/{TestClass}/</c> for hand inspection.
    /// One run directory is shared across the whole test process; a Windows Explorer
    /// window is opened the first time any snapshot is written. Older run directories
    /// (beyond the most recent <see cref="KeepRuns"/>) are pruned on first use.
    /// All operations are no-ops when no debugger is attached.
    /// </summary>
    internal static class DebugSnapshot
    {
        private const int KeepRuns = 4;
        private static readonly Lazy<string> RunDir = new Lazy<string>(InitRunDir);
        private static readonly object ExplorerLock = new object();
        private static bool _explorerOpened;

        public static bool Enabled => Debugger.IsAttached;

        /// <summary>
        /// Returns (and on first call creates) the per-test-class subdirectory under the
        /// current run's timestamp dir, e.g. <c>TestResults/20260427-132613/SolverTest/</c>.
        /// Returns null if no debugger is attached.
        /// </summary>
        public static string GetClassDir(string testClassName)
        {
            if (!Enabled) return null;
            string dir = Path.Combine(RunDir.Value, testClassName);
            Directory.CreateDirectory(dir);
            return dir;
        }

        /// <summary>
        /// Writes <paramref name="content"/> to <c>{classDir}/{name}</c> and opens Windows
        /// Explorer at the run directory the first time any file is written this run.
        /// </summary>
        public static void WriteText(string classDir, string name, string content)
        {
            if (classDir == null) return;
            File.WriteAllText(Path.Combine(classDir, name), content);
            OpenExplorerOnce();
        }

        private static string InitRunDir()
        {
            string testResultsDir = ResolveTestResultsDir();
            Directory.CreateDirectory(testResultsDir);
            PruneOldRunDirs(testResultsDir);
            string stamp = Path.Combine(testResultsDir, DateTime.Now.ToString("yyyyMMdd-HHmmss"));
            Directory.CreateDirectory(stamp);
            return stamp;
        }

        private static string ResolveTestResultsDir()
        {
            // cwd at runtime is bin/Debug/net9.0; walk up three levels to the test project root.
            string projectDir = Directory.GetParent(Directory.GetParent(Directory.GetParent(
                Directory.GetCurrentDirectory()).ToString()).ToString()).ToString();
            return Path.Combine(projectDir, "TestResults");
        }

        private static void PruneOldRunDirs(string testResultsDir)
        {
            Regex pattern = new Regex(@"^\d{8}-\d{6}$");
            string[] runs;
            try
            {
                runs = Directory.GetDirectories(testResultsDir)
                    .Where(d => pattern.IsMatch(Path.GetFileName(d)))
                    .OrderByDescending(d => Path.GetFileName(d), StringComparer.Ordinal)
                    .ToArray();
            }
            catch (DirectoryNotFoundException)
            {
                return;
            }

            // We're about to create a new one, so keep (KeepRuns - 1) of the existing dirs.
            foreach (string old in runs.Skip(KeepRuns - 1))
            {
                try { Directory.Delete(old, recursive: true); }
                catch { /* leave behind anything that's locked */ }
            }
        }

        private static void OpenExplorerOnce()
        {
            if (_explorerOpened) return;
            lock (ExplorerLock)
            {
                if (_explorerOpened) return;
                _explorerOpened = true;
                try
                {
                    Process.Start(new ProcessStartInfo("explorer.exe", $"\"{RunDir.Value}\"")
                    {
                        UseShellExecute = true
                    });
                }
                catch { /* best-effort; non-Windows or no GUI shouldn't fail the test */ }
            }
        }
    }
}
