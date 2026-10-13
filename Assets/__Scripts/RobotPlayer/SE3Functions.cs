using Unity.Mathematics;
using UnityEngine;

public class SE3Functions
{
    public static float4x4 ScrewMatrixExponential(float3 screwRotAxis, float3 screwTransAxis, float theta)
    {
        // calculates exp([A] * theta) where A = [w^T v^T]^T is a screw axis

        float3x3 omegaSkewed = SkewedVector(screwRotAxis);
        float3x3 rotationMatrix = float3x3.identity + Mathf.Sin(theta) * omegaSkewed + (1 - Mathf.Cos(theta)) * math.mul(omegaSkewed, omegaSkewed);

        if(screwTransAxis.Equals(float3.zero))
        {
            return new float4x4(rotationMatrix, float3.zero);
        } 
        else 
        {
            float3x3 vectorRotation = theta * float3x3.identity + (1 - math.cos(theta)) * omegaSkewed + (theta - math.sin(theta)) * math.mul(omegaSkewed, omegaSkewed);
            return new float4x4(rotationMatrix, math.mul(vectorRotation, screwTransAxis));
        }
    }

    public static float3x3 SkewedVector(float3 vector)
    {
        // converts a vector [a, b, c]^T into a skew-symmetric 3x3 matrix of the form
        // [[ 0, -c,  b]
        //  [ c,  0, -a]
        //  [-b,  a,  0]]
        return new float3x3(0, -vector[2], vector[1],
                            vector[2], 0, -vector[0],
                            -vector[1], vector[0], 0);
    }

    public static float3x3 RotationInverse(float3x3 rot)
    {
        // returns the inverse of a SO(3) matrix
        return math.transpose(rot);
    }

    public static float4x4 PoseInverse(float4x4 pose)
    {
        // calculates the inverse of a matrix in SE(3)
        float3x3 rotInv = RotationInverse(new float3x3(pose));
        float3 pos = PosePos(pose);
        return new float4x4(rotInv, -math.mul(rotInv, pos));
    }

    public static float3 PosePos(float4x4 pose)
    {
        return new float3(pose.c3[0], pose.c3[1], pose.c3[2]);
    }

    public static (float3, float3) AdjointBigSE3(float4x4 pose, float3 rotation, float3 translation)
    {
        // calculates [Ad_T] * V for a pose T and a twist/screw V of form [rotation^T translation^T]^T
        float3x3 rot = new float3x3(pose);
        float3 pos = PosePos(pose);
        float3 upper = math.mul(rot, rotation);
        float3 lower = math.mul(SkewedVector(pos), math.mul(rot, rotation)) + math.mul(rot, translation);

        return (upper, lower);
    }
    public static (float3, float3) AdjointTransposeBigSE3(float4x4 pose, float3 rotation, float3 translation)
    {
        // calculates [Ad_T]^T * V for a pose T and a twist/screw V of form [rotation^T translation^T]^T
        float3x3 rot = math.transpose(new float3x3(pose));
        float3 pos = PosePos(pose);
        float3 upper = math.mul(rot, rotation) + math.mul(math.mul(rot, -SkewedVector(pos)), translation);
        float3 lower = math.mul(rot, translation);

        return (upper, lower);
    }

    public static (float3, float3) AdjointLittleSE3(float3 upper1, float3 lower1, float3 upper2, float3 lower2)
    {
        // calculates [ad_V1]V2 for a pair of screw/twist/wrench structures V1 and V2
        float3 upper = math.mul(SkewedVector(upper1), upper2);
        float3 lower = math.mul(SkewedVector(lower1), upper2) + math.mul(SkewedVector(upper1), lower2);

        return (upper, lower);
    }
    public static (float3, float3) AdjointTransposeLittleSE3(float3 upper1, float3 lower1, float3 upper2, float3 lower2)
    {
        // calculates [ad_V1]V2 for a pair of screw/twist/wrench structures V1 and V2
        float3 upper = -math.mul(SkewedVector(upper1), upper2) - math.mul(SkewedVector(lower1), lower2);
        float3 lower = -math.mul(SkewedVector(upper1), lower2);

        return (upper, lower);
    }

}
