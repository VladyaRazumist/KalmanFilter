using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Filter
{
    public class MatrixObject
    {
        private int rows;
        private int colomns;
        public Matrix<double> matrix { get; private set; }

        public MatrixObject(int rows, int colomns)
        {
            this.rows = rows;
            this.colomns = colomns;
            matrix = Matrix<double>.Build.Dense(rows, colomns);
        }

        public static MatrixObject GetIdentityMatrix(int dim)
        {
            var identityMatrix = new MatrixObject(rows: dim, colomns: dim);

            for (var i = 0; i < dim; i++)
            {
                for (var j = 0; j < dim; j++)
                {
                    identityMatrix.matrix[i, j] = 1.0;
                }
            }

            return identityMatrix;
        }

        public static MatrixObject GetEmptyMatrix(int dim)
        {
            var emptyMatrix = new MatrixObject(dim, dim);

            for (var i = 0; i < dim; i++)
            {
                for (var j = 0; j < dim; j++)
                {
                    emptyMatrix.matrix[i, j] = 0.0;
                }
            }

            return emptyMatrix;
        }

        public void AddElement(int i, int j, double value)
        {
            if (matrix.RowCount > i && matrix.ColumnCount > j)
            {
                matrix[i, j] += value;
            }
        }

        public void SetMatrix(double[][] matrix)
        {
            var array = matrix.SelectMany(inner => inner).ToArray();

            if (this.matrix.RowCount > 0)
            {
                if (matrix.Length == this.matrix.ColumnCount && matrix[0].Length == this.matrix.RowCount)
                {
                    this.matrix = new DenseMatrix(matrix.Length, matrix[0].Length, array);
                }
            }
        }

        public double? GetElement(int i, int j)
        {
            if (matrix.RowCount > i && matrix.ColumnCount > j)
            {
                return matrix[i, j];
            }
            else
            {
                return null;
            }
        }

        public MatrixObject Transpose()
        {
            var result = new MatrixObject(rows, colomns)
            {
                matrix = matrix.Transpose()
            };

            return result;
        }

        public MatrixObject Inverse()
        {
            var result = new MatrixObject(rows, colomns)
            {
                matrix = matrix.Inverse()
            };

            return result;
        }

        public static MatrixObject operator +(MatrixObject mo1, MatrixObject mo2)
        {

            var result = new MatrixObject(mo1.rows, mo1.colomns)
            {
                matrix = mo1.matrix.Add(mo2.matrix)
            };

            return result;
        }

        public static MatrixObject operator -(MatrixObject mo1, MatrixObject mo2)
        {
            var result = new MatrixObject(mo1.rows, mo1.colomns)
            {
                matrix = mo1.matrix.Subtract(mo2.matrix)
            };

            return result;
        }

        public static MatrixObject operator *(MatrixObject mo1, MatrixObject mo2)
        {
            var result = new MatrixObject(mo1.rows, mo1.colomns)
            {
                matrix = mo1.matrix.Multiply(mo2.matrix)
            };

            return result;
        }

        public override string ToString()
        {
            var builder = new StringBuilder();

            for (var i = 0; i < matrix.RowCount; i++)
            {
                for (var j = 0; j < matrix.RowCount; j++)
                {
                    builder.Append(matrix[i, j]);
                }

                builder.Append("\n---");
            }

            return builder.ToString();
        }
    }
}
