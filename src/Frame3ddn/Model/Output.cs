using System.Collections.Generic;

namespace Frame3ddn.Model
{
    public class Output
    {
        public string TextOutput { get; set; }
        public IReadOnlyList<LoadCaseOutput> LoadCaseOutputs { get; }

        /// <summary>
        /// Modal-analysis results, one entry per requested mode (lowest first). Empty when
        /// the input did not request modal analysis (<c>DynamicAnalysisInput.ModesCount == 0</c>).
        /// </summary>
        public IReadOnlyList<ModalResult> ModalResults { get; }

        public Output(string textOutput, IReadOnlyList<LoadCaseOutput> loadCaseOutputs,
            IReadOnlyList<ModalResult> modalResults = null)
        {
            TextOutput = textOutput;
            LoadCaseOutputs = loadCaseOutputs;
            ModalResults = modalResults ?? new List<ModalResult>();
        }
    }
}

