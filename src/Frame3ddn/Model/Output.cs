using System.Collections.Generic;

namespace Frame3ddn.Model
{
    public class Output
    {
        public string TextOutput { get; set; }
        public IReadOnlyList<LoadCaseOutput> LoadCaseOutputs { get; }
        public Output(string textOutput, IReadOnlyList<LoadCaseOutput> loadCaseOutputs)
        {
            TextOutput = textOutput;
            LoadCaseOutputs = loadCaseOutputs;
        }
    }
}

