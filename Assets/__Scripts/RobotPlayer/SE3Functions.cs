using Unity.Mathematics;
using UnityEngine;
using System;

public class SE3Functions
{
    public static double4x4 ScrewMatrixExponential(double3 screwRotAxis, double3 screwTransAxis, double theta)
    {
        // calculates exp([A] * theta) where A = [w^T v^T]^T is a screw axis
        
        double3x3 omegaSkewed = SkewedVector(screwRotAxis);
        double3x3 rotationMatrix = double3x3.identity + Math.Sin(theta) * omegaSkewed + (1 - Math.Cos(theta)) * math.mul(omegaSkewed, omegaSkewed);

        if(screwTransAxis.Equals(double3.zero))
        {
            return PoseFromRotPos(rotationMatrix, double3.zero);
        } 
        else 
        {
            double3x3 vectorRotation = theta * double3x3.identity + (1 - math.cos(theta)) * omegaSkewed + (theta - math.sin(theta)) * math.mul(omegaSkewed, omegaSkewed);
            return PoseFromRotPos(rotationMatrix, math.mul(vectorRotation, screwTransAxis));
        }
    }

    public static double3x3 SkewedVector(double3 vector)
    {
        // converts a vector [a, b, c]^T into a skew-symmetric 3x3 matrix of the form
        // [[ 0, -c,  b]
        //  [ c,  0, -a]
        //  [-b,  a,  0]]
        return new double3x3(0, -vector[2], vector[1],
                            vector[2], 0, -vector[0],
                            -vector[1], vector[0], 0);
    }

    public static double3x3 RotationInverse(double3x3 rot)
    {
        // returns the inverse of a SO(3) matrix
        return math.transpose(rot);
    }

    public static double4x4 PoseInverse(double4x4 pose)
    {
        // calculates the inverse of a matrix in SE(3)
        double3x3 rotInv = RotationInverse(ExtractPoseRotation(pose));
        double3 pos = PosePos(pose);
        return PoseFromRotPos(rotInv, -math.mul(rotInv, pos));
    }

    public static double3 PosePos(double4x4 pose)
    {
        return new double3(pose.c3[0], pose.c3[1], pose.c3[2]);
    }


    public static (double3, double3) AdjointBigSE3(double4x4 pose, double3 rotation, double3 translation)
    {
        // calculates [Ad_T] * V for a pose T and a twist/screw V of form [rotation^T translation^T]^T
        double3x3 rot = ExtractPoseRotation(pose);
        double3 pos = PosePos(pose);
        double3 upper = math.mul(rot, rotation);
        double3 lower = math.mul(SkewedVector(pos), math.mul(rot, rotation)) + math.mul(rot, translation);

        return (upper, lower);
    }
    public static (double3, double3) AdjointTransposeBigSE3(double4x4 pose, double3 rotation, double3 translation)
    {
        // calculates [Ad_T]^T * V for a pose T and a twist/screw V of form [rotation^T translation^T]^T
        double3x3 rot = math.transpose(ExtractPoseRotation(pose));
        double3 pos = PosePos(pose);
        double3 upper = math.mul(rot, rotation) + math.mul(math.mul(rot, -SkewedVector(pos)), translation);
        double3 lower = math.mul(rot, translation);

        return (upper, lower);
    }

    public static (double3, double3) AdjointLittleSE3(double3 upper1, double3 lower1, double3 upper2, double3 lower2)
    {
        // calculates [ad_V1]V2 for a pair of screw/twist/wrench structures V1 and V2
        double3 upper = math.mul(SkewedVector(upper1), upper2);
        double3 lower = math.mul(SkewedVector(lower1), upper2) + math.mul(SkewedVector(upper1), lower2);

        return (upper, lower);
    }
    public static (double3, double3) AdjointTransposeLittleSE3(double3 upper1, double3 lower1, double3 upper2, double3 lower2)
    {
        // calculates [ad_V1]V2 for a pair of screw/twist/wrench structures V1 and V2
        double3 upper = -math.mul(SkewedVector(upper1), upper2) - math.mul(SkewedVector(lower1), lower2);
        double3 lower = -math.mul(SkewedVector(upper1), lower2);

        return (upper, lower);
    }
    public static double4x4 PoseFromPos(double3 pos)
    {
        // returns a SE(3) matrix with R=I and p=pos
        return PoseFromRotPos(double3x3.identity, pos);
    }

    public static double4x4 PoseFromRotPos(double3x3 rot, double3 pos)
    {
        //used just to convert double4x4 values. Use float4x4 for floats
        double4 c0 = new double4(rot.c0, 0);
        double4 c1 = new double4(rot.c1, 0);
        double4 c2 = new double4(rot.c2, 0);

        return new double4x4(c0, c1, c2, new double4(pos, 1));
    }

    public static double3x3 ExtractPoseRotation(double4x4 pose)
    {
        double3 c0 = new double3(pose.c0[0], pose.c0[1], pose.c0[2]);
        double3 c1 = new double3(pose.c1[0], pose.c1[1], pose.c1[2]);
        double3 c2 = new double3(pose.c2[0], pose.c2[1], pose.c2[2]);

        return new double3x3(c0, c1, c2);
    }

    public static double Dot(double[] x, double[] y)
    {
        if (x.Length != y.Length)
        {
            Debug.LogException(new FormatException("Dot product arrays must have same length"));
            return Mathf.Infinity;
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
    public static double Dot(double3 x, double3 y)
    {
        double total = 0;
        for(int i = 0; i < 3; i++)
        {
            total += x[i] * y[i];
        }
        return total;
    }
}
