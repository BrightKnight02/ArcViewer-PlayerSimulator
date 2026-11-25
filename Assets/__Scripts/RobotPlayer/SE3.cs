using System;
using NUnit.Framework.Constraints;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;


public class SE3
{
    public static double[,] ScrewMatrixExponential(double[] screwAxis, double theta)


    {
        double[] omega = new double[] { screwAxis[0], screwAxis[1], screwAxis[2] };
        double[] v = new double[] { screwAxis[3], screwAxis[4], screwAxis[5] };

        double[,] omegaSkewed = SkewedVector(omega);
        double[,] rotationMatrix = MatAdd(IdentityMatrix(3), MatAdd(MatMulScalar(omegaSkewed, math.sin(theta)), MatMulScalar(MatMul(omegaSkewed, omegaSkewed), 1.0-math.cos(theta))));

        double[,] G_theta = MatAdd(MatMulScalar(IdentityMatrix(3), theta), MatAdd(MatMulScalar(omegaSkewed, 1.0-math.cos(theta)), MatMulScalar(MatMul(omegaSkewed, omegaSkewed), theta - math.sin(theta))));

        double[] position = MatMul(G_theta, v);

        return PoseFromRotAndPos(rotationMatrix, position);

    }

    public static double[,] SkewedVector(double[] vector)
    {
        // converts a vector [a, b, c]^T into a skew-symmetric 3x3 matrix of the form
        // [[ 0, -c,  b]
        //  [ c,  0, -a]
        //  [-b,  a,  0]] 
        // or converts a vector [w^t, v^T]^T into a 4x4 matrix of the form
        // [[skewed(w), v]
        //  [0        , 0]]
        if(vector.Length == 3)
        {
            return new double[,] { {0, -vector[2], vector[1] },
                                   { vector[2], 0, -vector[0]},
                                   {-vector[1], vector[0], 0 } };
        }
        else if(vector.Length == 6)
        {
            double[] tempvec = new double[] { vector[0], vector[1], vector[2] };

            double[,] tempmat = SkewedVector(tempvec);
            return new double[,] { { tempmat[0, 0], tempmat[0, 1], tempmat[0, 2], vector[3] },
                                   { tempmat[1, 0], tempmat[1, 1], tempmat[1, 2], vector[4] },
                                   { tempmat[2, 0], tempmat[2, 1], tempmat[2, 2], vector[5] },
                                   {            0 ,            0 ,            0 ,        0  } };
        }
        else
        {
            throw new ArgumentOutOfRangeException($"Cannot skew a {vector.Length} long vector");
        }
    }

    public static double[,] RotationInverse(double[,] rot)
    {
        double[,] holder = new double[3, 3];
        alglib.rmatrixtranspose(3, 3, rot, 0, 0, holder, 0, 0);
        return holder;
    }

    public static double[,] PoseInverse(double[,] pose)
    {
        // calculates the inverse of a matrix in SE(3)
        double[,] rotInv = RotationInverse(ExtractPoseRotation(pose));
        double[] pos = ExtractPosePos(pose);

        return PoseFromRotAndPos(rotInv, MatMulScalar(MatMul(rotInv, ColFromVect(pos)), -1.0));
    }


    public static double[] ExtractPosePos(double[,] pose)
    {
        return new double[] { pose[0, 3], pose[1, 3], pose[1, 3] };
    }



    public static double[,] AdjointBigSE3(double[,] pose)
    {
        // returns [Ad_T] for a pose
        double[][][,] holder = new double[][][,] { new double[][,]{ ExtractPoseRotation(pose), null },
                                                   new double[][,]{ MatMul(SkewedVector(ExtractPosePos(pose)), ExtractPoseRotation(pose)), ExtractPoseRotation(pose)}};
        return BlockMatrix(holder);
    }


    public static double[,] AdjointTransposeBigSE3(double[,] pose)
    {
        double[,] holder = new double[6, 6];
        alglib.rmatrixtranspose(6, 6, AdjointBigSE3(pose), 0, 0, holder, 0, 0);
        return holder;
    }

    public static double[,] AdjointLittleSE3(double[] screw)
    {
        //returns [ad_v] = [[ [w],  0  ],
        //                  [ [v], [w] ]]
        double[] omega = new double[] { screw[0], screw[1], screw[2] };
        double[] v = new double[] { screw[3], screw[4], screw[5] };

        double[][][,] holder = new double[][][,] { new double[][,]{ SkewedVector(omega), null },
                                               new double[][,]{ SkewedVector(v), SkewedVector(omega) } };

        return BlockMatrix(holder);
    }

    public static double[,] AdjointTransposeLittleSE3(double[] screw)
    {
        double[,] holder = new double[6, 6];
        alglib.rmatrixtranspose(6, 6, AdjointLittleSE3(screw), 0, 0, holder, 0, 0);
        return holder;
    }

    public static double[,] PoseFromPos(double[] pos)
    {
        return PoseFromRotAndPos(IdentityMatrix(3), pos);
    }

    public static double[,] PoseFromRotAndPos(double[,] rot, double[] pos)
    {
        return BlockMatrix(new double[][][,] { new double[][,]{ rot, ColFromVect(pos) },
                                               new double[][,]{ null, new double[,] {{ 1 }} } });
    }

    public static double[,] PoseFromRotAndPos(double[,] rot, double[,] pos)
    {
        if(pos.GetLength(0) != 3 || pos.GetLength(1) != 1)
        {
            throw new ArgumentException("Position vector of wrong shape");
        }
        return BlockMatrix(new double[][][,] { new double[][,]{ rot, pos },
                                               new double[][,]{ null, new double[,] {{ 1 }} } });
    }



    public static double[,] ExtractPoseRotation(double[,] pose)
    {
        return new double[,] { { pose[0, 0], pose[0, 1], pose[0, 2] },
                               { pose[1, 0], pose[1, 1], pose[1, 2] },
                               { pose[2, 0], pose[2, 1], pose[2, 2] },};
    }

    public static double Dot(double[] x, double[] y)
    {
        if(x.Length != y.Length)
        {
            throw new FormatException("Dot product vectors must have same length!");
        }
        else
        {
            double total = 0;
            for(int i = 0; i < x.Length; i++)
            {
                total += x[i] * y[i];
            }
            return total;
        }
    }

    public static double[,] MatMul(double[,] A, double[,] B)
    {
        int ACols = A.GetLength(0);
        int ARows = A.GetLength(1);
        int BCols = B.GetLength(0);
        int BRows = B.GetLength(1);

        if(ACols != BRows)
        {
            throw new ArgumentException($"A {ARows}x{ACols} matrix cannot be multiplied by a {BRows}x{BCols} matrix!");
        }
        else
        {
            double[,] output = new double[ARows, BCols];
            alglib.rmatrixgemm(ARows, BCols, ACols, 1, A, 0, 0, 0, B, 0, 0, 0, 0, output, 0, 0);
            return output;
        }
    }
    public static double[] MatMul(double[,] A, double[] b)
    {
        int ACols = A.GetLength(0);
        int ARows = A.GetLength(1);
        int bLength = b.Length;

        if(ACols != bLength)
        {
            throw new ArgumentException($"A {ARows}x{ACols} matrix cannot be multiplied by a {bLength} column vector!");
        }
        else
        {
            double[,] bvector = new double[bLength, 1];
            for(int i = 0; i < bLength; i++) bvector[i, 0] = b[i];
            double[,] outputRaw = new double[ARows, 1];
            alglib.rmatrixgemm(ARows, 1, ACols, 1, A, 0, 0, 0, bvector, 0, 0, 0, 0, outputRaw, 0, 0);

            double[] output = new double[ARows];
            for(int i = 0; i < ARows; i++) output[i] = outputRaw[i, 0];
            return output;
        }
    }
    public static double[] MatMul(double[] a, double[,] B)
    {
        int BCols = B.GetLength(0);
        int BRows = B.GetLength(1);
        int aLength = a.Length;

        if(BRows != aLength)
        {
            throw new ArgumentException($"A {aLength} row vector cannot be mutliplied by a {BRows}x{BCols} matrix!");
        }
        else
        {
            double[,] avector = new double[1, aLength];
            for(int i = 0; i < aLength; i++) avector[0, i] = a[i];
            double[,] outputRaw = new double[1, BCols];
            alglib.rmatrixgemm(1, BCols, aLength, 1, avector, 0, 0, 0, B, 0, 0, 0, 0, outputRaw, 0, 0);

            double[] output = new double[BCols];
            for(int i = 0; i < BCols; i++) output[i] = outputRaw[0, i];
            return output;
        }
    }
    public static double[,] MatMulScalar(double[,] A, double b)
    {
        int ACols = A.GetLength(0);
        int ARows = A.GetLength(1);

        double[,] output = (double[,])A.Clone();


        for(int i = 0; i < ARows; i++)
        {
            for(int j = 0; j < ACols; j++)
            {
                output[i, j] *= b;
            }
        }
        return output;
    }

    public static double[,] BlockMatrix(double[][][,] blocks)
    {
        // AI generated because I really didn't want to do this myself
        int R = blocks.Length;
        if(R == 0) return new double[0, 0];
        int C = blocks[0].Length;

        int[] h = new int[R];   // height of each block row
        int[] w = new int[C];   // width of each block column
        for(int i = 0; i < R; i++) h[i] = -1;
        for(int j = 0; j < C; j++) w[j] = -1;

        // Determine and validate block row heights / column widths
        for(int i = 0; i < R; i++)
        {
            if(blocks[i].Length != C)
                throw new ArgumentException($"Block row {i} has {blocks[i].Length} blocks, expected {C}.");

            for(int j = 0; j < C; j++)
            {
                double[,] b = blocks[i][j];
                if(b == null) continue;

                int bh = b.GetLength(0), bw = b.GetLength(1);

                if(h[i] == -1) h[i] = bh;
                else if(h[i] != bh)
                    throw new ArgumentException($"Block ({i},{j}) has height {bh}, but block row {i} has height {h[i]}.");

                if(w[j] == -1) w[j] = bw;
                else if(w[j] != bw)
                    throw new ArgumentException($"Block ({i},{j}) has width {bw}, but block column {j} has width {w[j]}.");
            }
        }

        for(int i = 0; i < R; i++)
            if(h[i] == -1) throw new ArgumentException($"Block row {i} is all null; its height can't be inferred.");
        for(int j = 0; j < C; j++)
            if(w[j] == -1) throw new ArgumentException($"Block column {j} is all null; its width can't be inferred.");

        // Offsets of each block row/column in the result
        int[] rowOff = new int[R + 1];
        int[] colOff = new int[C + 1];
        for(int i = 0; i < R; i++) rowOff[i + 1] = rowOff[i] + h[i];
        for(int j = 0; j < C; j++) colOff[j + 1] = colOff[j] + w[j];

        // The new array is zero-initialized, so null blocks need no work
        double[,] result = new double[rowOff[R], colOff[C]];

        for(int i = 0; i < R; i++)
            for(int j = 0; j < C; j++)
            {
                double[,] b = blocks[i][j];
                if(b == null) continue;
                for(int r = 0; r < h[i]; r++)
                    for(int c = 0; c < w[j]; c++)
                        result[rowOff[i] + r, colOff[j] + c] = b[r, c];
            }

        return result;
    }

    public static double[,] ColFromVect(double[] vector)
    {
        double[,] output = new double[vector.Length, 1];
        for(int i = 0; i < vector.Length; i++)
        {
            output[i, 0] = vector[i];
        }
        return output;
    }
    public static double[,] RowFromVect(double[] vector)
    {
        double[,] output = new double[1, vector.Length];
        for(int i = 0; i < vector.Length; i++)
        {
            output[0, i] = vector[i];
        }
        return output;
    }

    public static double[,] IdentityMatrix(int size)
    {
        double[,] output = new double[size, size];
        for(int i = 0; i < size; i++)
        {
            output[i, i] = 1.0;
        }
        return output;
    }

    public static double[,] MatAdd(double[,] A, double[,] B)
    {
        int ACols = A.GetLength(0);
        int ARows = A.GetLength(1);
        int BCols = B.GetLength(0);
        int BRows = B.GetLength(1);

        if(ACols != BCols || ARows != BRows)
        {
            throw new ArgumentException($"A {ARows}x{ACols} matrix cannot be added to a {BRows}x{BCols} matrix!");
        }
        double[,] output = new double[ARows, ACols];
        for(int i = 0; i < ARows; i++)
        {
            for(int j = 0; j < ACols; j++)
            {
                output[i, j] = A[i, j] = B[i, j];
            }
        }
        return output;
    }
}
