﻿// <copyright file="SparseMatrixTests.cs" company="Math.NET">
// Math.NET Numerics, part of the Math.NET Project
// http://numerics.mathdotnet.com
// http://github.com/mathnet/mathnet-numerics
// http://mathnetnumerics.codeplex.com
// Copyright (c) 2009-2010 Math.NET
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// </copyright>

namespace MathNet.Numerics.UnitTests.LinearAlgebraTests.Complex32
{
    using System;
    using System.Collections.Generic;
    using LinearAlgebra.Complex32;
    using NUnit.Framework;
    using Complex32 = Numerics.Complex32;

    /// <summary>
    /// Sparse matrix tests.
    /// </summary>
    public class SparseMatrixTests : MatrixTests
    {
        /// <summary>
        /// Creates a matrix for the given number of rows and columns.
        /// </summary>
        /// <param name="rows">The number of rows.</param>
        /// <param name="columns">The number of columns.</param>
        /// <returns>A matrix with the given dimensions.</returns>
        protected override Matrix CreateMatrix(int rows, int columns)
        {
            return new SparseMatrix(rows, columns);
        }

        /// <summary>
        /// Creates a matrix from a 2D array.
        /// </summary>
        /// <param name="data">The 2D array to create this matrix from.</param>
        /// <returns>A matrix with the given values.</returns>
        protected override Matrix CreateMatrix(Complex32[,] data)
        {
            return new SparseMatrix(data);
        }

        /// <summary>
        /// Creates a vector of the given size.
        /// </summary>
        /// <param name="size">The size of the vector to create.
        /// </param>
        /// <returns>The new vector. </returns>
        protected override Vector CreateVector(int size)
        {
            return new SparseVector(size);
        }

        /// <summary>
        /// Creates a vector from an array.
        /// </summary>
        /// <param name="data">The array to create this vector from.</param>
        /// <returns>The new vector. </returns>
        protected override Vector CreateVector(Complex32[] data)
        {
            return new SparseVector(data);
        }

        /// <summary>
        /// Can create a matrix form array.
        /// </summary>
        [Test]
        public void CanCreateMatrixFrom1DArray()
        {
            var testData = new Dictionary<string, Matrix>
                           {
                               { "Singular3x3", new SparseMatrix(3, 3, new[] { new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(2.0f, 1), new Complex32(2.0f, 1), new Complex32(2.0f, 1) }) }, 
                               { "Square3x3", new SparseMatrix(3, 3, new[] { new Complex32(-1.1f, 1), Complex32.Zero, new Complex32(-4.4f, 1), new Complex32(-2.2f, 1), new Complex32(1.1f, 1), new Complex32(5.5f, 1), new Complex32(-3.3f, 1), new Complex32(2.2f, 1), new Complex32(6.6f, 1) }) }, 
                               { "Square4x4", new SparseMatrix(4, 4, new[] { new Complex32(-1.1f, 1), Complex32.Zero, new Complex32(1.0f, 1), new Complex32(-4.4f, 1), new Complex32(-2.2f, 1), new Complex32(1.1f, 1), new Complex32(2.1f, 1), new Complex32(5.5f, 1), new Complex32(-3.3f, 1), new Complex32(2.2f, 1), new Complex32(6.2f, 1), new Complex32(6.6f, 1), new Complex32(-4.4f, 1), new Complex32(3.3f, 1), new Complex32(4.3f, 1), new Complex32(-7.7f, 1) }) }, 
                               { "Tall3x2", new SparseMatrix(3, 2, new[] { new Complex32(-1.1f, 1), Complex32.Zero, new Complex32(-4.4f, 1), new Complex32(-2.2f, 1), new Complex32(1.1f, 1), new Complex32(5.5f, 1) }) }, 
                               { "Wide2x3", new SparseMatrix(2, 3, new[] { new Complex32(-1.1f, 1), Complex32.Zero, new Complex32(-2.2f, 1), new Complex32(1.1f, 1), new Complex32(-3.3f, 1), new Complex32(2.2f, 1) }) }
                           };

            foreach (var name in testData.Keys)
            {
                Assert.AreEqual(TestMatrices[name], testData[name]);
            }
        }

        /// <summary>
        /// Matrix from array is a copy.
        /// </summary>
        [Test]
        public void MatrixFrom1DArrayIsCopy()
        {
            // Sparse Matrix copies values from Complex32[], but no remember reference. 
            var data = new[] { new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(1.0f, 1), new Complex32(2.0f, 1), new Complex32(2.0f, 1), new Complex32(2.0f, 1) };
            var matrix = new SparseMatrix(3, 3, data);
            matrix[0, 0] = new Complex32(10.0f, 1);
            Assert.AreNotEqual(new Complex32(10.0f, 1), data[0]);
        }

        /// <summary>
        /// Matrix from two-dimensional array is a copy.
        /// </summary>
        [Test]
        public void MatrixFrom2DArrayIsCopy()
        {
            var matrix = new SparseMatrix(TestData2D["Singular3x3"]);
            matrix[0, 0] = new Complex32(10.0f, 1);
            Assert.AreEqual(new Complex32(1.0f, 1), TestData2D["Singular3x3"][0, 0]);
        }

        /// <summary>
        /// Can create a matrix from two-dimensional array.
        /// </summary>
        /// <param name="name">Matrix name.</param>
        [Test]
        public void CanCreateMatrixFrom2DArray([Values("Singular3x3", "Singular4x4", "Square3x3", "Square4x4", "Tall3x2", "Wide2x3")] string name)
        {
            var matrix = new SparseMatrix(TestData2D[name]);
            for (var i = 0; i < TestData2D[name].GetLength(0); i++)
            {
                for (var j = 0; j < TestData2D[name].GetLength(1); j++)
                {
                    Assert.AreEqual(TestData2D[name][i, j], matrix[i, j]);
                }
            }
        }

        /// <summary>
        /// Can create a matrix with uniform values.
        /// </summary>
        [Test]
        public void CanCreateMatrixWithUniformValues()
        {
            var matrix = new SparseMatrix(10, 10, new Complex32(10.0f, 1));
            for (var i = 0; i < matrix.RowCount; i++)
            {
                for (var j = 0; j < matrix.ColumnCount; j++)
                {
                    Assert.AreEqual(matrix[i, j], new Complex32(10.0f, 1));
                }
            }
        }

        /// <summary>
        /// Can create an identity matrix.
        /// </summary>
        [Test]
        public void CanCreateIdentity()
        {
            var matrix = SparseMatrix.Identity(5);
            for (var i = 0; i < matrix.RowCount; i++)
            {
                for (var j = 0; j < matrix.ColumnCount; j++)
                {
                    Assert.AreEqual(i == j ? Complex32.One : Complex32.Zero, matrix[i, j]);
                }
            }
        }

        /// <summary>
        /// Identity with wrong order throws <c>ArgumentOutOfRangeException</c>.
        /// </summary>
        /// <param name="order">The size of the square matrix</param>
        [Test]
        public void IdentityWithWrongOrderThrowsArgumentOutOfRangeException([Values(0, -1)] int order)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => SparseMatrix.Identity(order));
        }

        /// <summary>
        /// Can create a large sparse matrix
        /// </summary>
        [Test]
        public void CanCreateLargeSparseMatrix()
        {
            var matrix = new SparseMatrix(500, 1000);
            var nonzero = 0;
            var rnd = new Random();

            for (var i = 0; i < matrix.RowCount; i++)
            {
                for (var j = 0; j < matrix.ColumnCount; j++)
                {
                    var value = rnd.Next(10) * rnd.Next(10) * rnd.Next(10) * rnd.Next(10) * rnd.Next(10);
                    if (value != 0)
                    {
                        nonzero++;
                    }

                    matrix[i, j] = value;
                }
            }

            Assert.AreEqual(matrix.NonZerosCount, nonzero);
        }

        /// <summary>
        /// Test whether order matters when adding sparse matrices.
        /// </summary>
        [Test]
        public void CanAddSparseMatricesBothWays()
        {
            var m1 = new SparseMatrix(1, 3);
            var m2 = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            var sum1 = m1 + m2;
            var sum2 = m2 + m1;
            Assert.IsTrue(sum1.Equals(m2));
            Assert.IsTrue(sum1.Equals(sum2));

            var sparseResult = new SparseMatrix(1, 3);
            sparseResult.Add(m2, sparseResult);
            Assert.IsTrue(sparseResult.Equals(sum1));

            sparseResult = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            sparseResult.Add(m1, sparseResult);
            Assert.IsTrue(sparseResult.Equals(sum1));

            sparseResult = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            m1.Add(sparseResult, sparseResult);
            Assert.IsTrue(sparseResult.Equals(sum1));

            sparseResult = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            sparseResult.Add(sparseResult, sparseResult);
            Assert.IsTrue(sparseResult.Equals(2 * sum1));

            var denseResult = new DenseMatrix(1, 3);
            denseResult.Add(m2, denseResult);
            Assert.IsTrue(denseResult.Equals(sum1));

            denseResult = new DenseMatrix(new Complex32[,] { { 0, 1, 1 } });
            denseResult.Add(m1, denseResult);
            Assert.IsTrue(denseResult.Equals(sum1));

            var m3 = new DenseMatrix(new Complex32[,] { { 0, 1, 1 } });
            var sum3 = m1 + m3;
            var sum4 = m3 + m1;
            Assert.IsTrue(sum3.Equals(m3));
            Assert.IsTrue(sum3.Equals(sum4));
        }

        /// <summary>
        /// Test whether order matters when subtracting sparse matrices.
        /// </summary>
        [Test]
        public void CanSubtractSparseMatricesBothWays()
        {
            var m1 = new SparseMatrix(1, 3);
            var m2 = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            var diff1 = m1 - m2;
            var diff2 = m2 - m1;
            Assert.IsTrue(diff1.Equals(m2.Negate()));
            Assert.IsTrue(diff1.Equals(diff2.Negate()));

            var sparseResult = new SparseMatrix(1, 3);
            sparseResult.Subtract(m2, sparseResult);
            Assert.IsTrue(sparseResult.Equals(diff1));

            sparseResult = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            sparseResult.Subtract(m1, sparseResult);
            Assert.IsTrue(sparseResult.Equals(diff2));

            sparseResult = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            m1.Subtract(sparseResult, sparseResult);
            Assert.IsTrue(sparseResult.Equals(diff1));

            sparseResult = new SparseMatrix(new Complex32[,] { { 0, 1, 1 } });
            sparseResult.Subtract(sparseResult, sparseResult);
            Assert.IsTrue(sparseResult.Equals(0 * diff1));

            var denseResult = new DenseMatrix(1, 3);
            denseResult.Subtract(m2, denseResult);
            Assert.IsTrue(denseResult.Equals(diff1));

            denseResult = new DenseMatrix(new Complex32[,] { { 0, 1, 1 } });
            denseResult.Subtract(m1, denseResult);
            Assert.IsTrue(denseResult.Equals(diff2));

            var m3 = new DenseMatrix(new Complex32[,] { { 0, 1, 1 } });
            var diff3 = m1 - m3;
            var diff4 = m3 - m1;
            Assert.IsTrue(diff3.Equals(m3.Negate()));
            Assert.IsTrue(diff3.Equals(diff4.Negate()));
        }
    }
}
