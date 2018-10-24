using System;
using System.Collections.Generic;
using System.Text;

namespace Frame3ddn
{
    public class Solver
    {
        public Output Solve(Input input)
        {
            return null;
            
            
        }

        // ref main.c:265
        static int Dof(Input input) => input.Nodes.Count * 6;

        /// <summary>
        /// global stiffness matrix
        /// </summary>
        // ref main.c:343
        static double[,] K(Input input)
        {
            var dof = Dof(input);
            var k = new double[dof,dof];
            return k;
        }
    }
}
