﻿// <copyright file="DenseVectorTests.cs" company="Math.NET">
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

namespace MathNet.Numerics.UnitTests.LinearAlgebraTests.Single
{
    using System;
    using System.Collections.Generic;
    using LinearAlgebra.Generic;
    using LinearAlgebra.Single;
    using NUnit.Framework;

    /// <summary>
    /// Dense vector tests.
    /// </summary>
    public class DenseVectorTests : VectorTests
    {
        /// <summary>
        /// Creates a new instance of the Vector class.
        /// </summary>
        /// <param name="size">The size of the <strong>Vector</strong> to construct.</param>
        /// <returns>The new <c>Vector</c>.</returns>
        protected override Vector<float> CreateVector(int size)
        {
            return new DenseVector(size);
        }

        /// <summary>
        /// Creates a new instance of the Vector class.
        /// </summary>
        /// <param name="data">The array to create this vector from.</param>
        /// <returns>The new <c>Vector</c>.</returns>
        protected override Vector<float> CreateVector(IList<float> data)
        {
            var vector = new DenseVector(data.Count);
            for (var index = 0; index < data.Count; index++)
            {
                vector[index] = data[index];
            }

            return vector;
        }

        /// <summary>
        /// Can create a dense vector form array.
        /// </summary>
        [Test]
        public void CanCreateDenseVectorFromArray()
        {
            var data = new float[Data.Length];
            Array.Copy(Data, data, Data.Length);
            var vector = new DenseVector(data);

            for (var i = 0; i < data.Length; i++)
            {
                Assert.AreEqual(data[i], vector[i]);
            }

            vector[0] = 100.0f;
            Assert.AreEqual(100.0f, data[0]);
        }

        /// <summary>
        /// Can create a dense vector from another dense vector.
        /// </summary>
        [Test]
        public void CanCreateDenseVectorFromAnotherDenseVector()
        {
            var vector = new DenseVector(Data);
            var other = new DenseVector(vector);

            Assert.AreNotSame(vector, other);
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(vector[i], other[i]);
            }
        }

        /// <summary>
        /// Can create a dense vector from another vector.
        /// </summary>
        [Test]
        public void CanCreateDenseVectorFromAnotherVector()
        {
            var vector = (Vector<float>)new DenseVector(Data);
            var other = new DenseVector(vector);

            Assert.AreNotSame(vector, other);
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(vector[i], other[i]);
            }
        }

        /// <summary>
        /// Can create a dense vector from user defined vector.
        /// </summary>
        [Test]
        public void CanCreateDenseVectorFromUserDefinedVector()
        {
            var vector = new UserDefinedVector(Data);
            var other = new DenseVector(vector);

            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(vector[i], other[i]);
            }
        }

        /// <summary>
        /// Can create a dense vector with constant values.
        /// </summary>
        [Test]
        public void CanCreateDenseVectorWithConstantValues()
        {
            var vector = new DenseVector(5, 5);
            foreach (var t in vector)
            {
                Assert.AreEqual(t, 5);
            }
        }

        /// <summary>
        /// Can create a dense matrix.
        /// </summary>
        [Test]
        public void CanCreateDenseMatrix()
        {
            var vector = new DenseVector(3);
            var matrix = vector.CreateMatrix(2, 3);
            Assert.AreEqual(2, matrix.RowCount);
            Assert.AreEqual(3, matrix.ColumnCount);
        }

        /// <summary>
        /// Can convert a dense vector to an array.
        /// </summary>
        [Test]
        public void CanConvertDenseVectorToArray()
        {
            var vector = new DenseVector(Data);
            var array = (float[])vector;
            Assert.IsInstanceOf(typeof(float[]), array);
            CollectionAssert.AreEqual(vector, array);
        }

        /// <summary>
        /// Can convert an array to a dense vector.
        /// </summary>
        [Test]
        public void CanConvertArrayToDenseVector()
        {
            var array = new[] { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f };
            var vector = (DenseVector)array;
            Assert.IsInstanceOf(typeof(DenseVector), vector);
            CollectionAssert.AreEqual(array, array);
        }

        /// <summary>
        /// Can call unary plus operator on a vector.
        /// </summary>
        [Test]
        public void CanCallUnaryPlusOperatorOnDenseVector()
        {
            var vector = new DenseVector(Data);
            var other = +vector;
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(vector[i], other[i]);
            }
        }

        /// <summary>
        /// Can add two dense vectors using "+" operator.
        /// </summary>
        [Test]
        public void CanAddTwoDenseVectorsUsingOperator()
        {
            var vector = new DenseVector(Data);
            var other = new DenseVector(Data);
            var result = vector + other;
            CollectionAssert.AreEqual(Data, vector, "Making sure the original vector wasn't modified.");
            CollectionAssert.AreEqual(Data, other, "Making sure the original vector wasn't modified.");

            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] * 2.0f, result[i]);
            }
        }

        /// <summary>
        /// Can call unary negate operator on a dense vector.
        /// </summary>
        [Test]
        public void CanCallUnaryNegationOperatorOnDenseVector()
        {
            var vector = new DenseVector(Data);
            var other = -vector;
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(-Data[i], other[i]);
            }
        }

        /// <summary>
        /// Can subtract two dense vectors using "-" operator.
        /// </summary>
        [Test]
        public void CanSubtractTwoDenseVectorsUsingOperator()
        {
            var vector = new DenseVector(Data);
            var other = new DenseVector(Data);
            var result = vector - other;
            CollectionAssert.AreEqual(Data, vector, "Making sure the original vector wasn't modified.");
            CollectionAssert.AreEqual(Data, other, "Making sure the original vector wasn't modified.");

            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(0.0f, result[i]);
            }
        }

        /// <summary>
        /// Can multiply a dense vector by a scalar using "*" operator.
        /// </summary>
        [Test]
        public void CanMultiplyDenseVectorByScalarUsingOperators()
        {
            var vector = new DenseVector(Data);
            vector = vector * 2.0f;

            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] * 2.0f, vector[i]);
            }

            vector = vector * 1.0f;
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] * 2.0f, vector[i]);
            }

            vector = new DenseVector(Data);
            vector = 2.0f * vector;

            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] * 2.0f, vector[i]);
            }

            vector = 1.0f * vector;
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] * 2.0f, vector[i]);
            }
        }

        /// <summary>
        /// Can divide a dense vector by a scalar using "/" operator.
        /// </summary>
        [Test]
        public void CanDivideDenseVectorByScalarUsingOperators()
        {
            var vector = new DenseVector(Data);
            vector = vector / 2.0f;

            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] / 2.0f, vector[i]);
            }

            vector = vector / 1.0f;
            for (var i = 0; i < Data.Length; i++)
            {
                Assert.AreEqual(Data[i] / 2.0f, vector[i]);
            }
        }

        /// <summary>
        /// Can calculate an outer product for a dense vector.
        /// </summary>
        [Test]
        public void CanCalculateOuterProductForDenseVector()
        {
            var vector1 = CreateVector(Data);
            var vector2 = CreateVector(Data);
            var m = Vector<float>.OuterProduct(vector1, vector2);
            for (var i = 0; i < vector1.Count; i++)
            {
                for (var j = 0; j < vector2.Count; j++)
                {
                    Assert.AreEqual(m[i, j], vector1[i] * vector2[j]);
                }
            }
        }

        /// <summary>
        /// Outer product for <c>null</c> dense vectors throws <c>ArgumentNullException</c>.
        /// </summary>
        [Test]
        public void OuterProductForNullDenseVectorsThrowsArgumentNullException()
        {
            DenseVector vector1 = null;
            var vector2 = CreateVector(Data);
            Assert.Throws<ArgumentNullException>(() => Vector<float>.OuterProduct(vector1, vector2));
            Assert.Throws<ArgumentNullException>(() => Vector<float>.OuterProduct(vector2, vector1));
        }
    }
}
