using Unity.Mathematics;
using UnityEngine;

public class ExtendedMathFunctions
{
    public float4x4 ScrewMatrixExponential(float3 screwRotAxis, float3 screwTransAxis, float theta)
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

    public float3x3 SkewedVector(float3 vector)
    {
        // converts a vector [a, b, c]^T into a skew-symmetric 3x3 matrix of the form
        // [[ 0, -c,  b]
        //  [ c,  0, -a]
        //  [-b,  a,  0]]
        return new float3x3(0, -vector[2], vector[1],
                            vector[2], 0, -vector[0],
                            -vector[1], vector[0], 0);
    }

    public float3x3 RotationInverse(float3x3 rot)
    {
        // returns the inverse of a SO(3) matrix
        return math.transpose(rot);
    }

    public float4x4 PoseInverse(float4x4 pose)
    {
        // calculates the inverse of a matrix in SE(3)
        float3x3 rotInv = RotationInverse(new float3x3(pose));
        float3 pos = new float3(pose.c3[0], pose.c3[1], pose.c3[2]);
        return new float4x4(rotInv, -math.mul(rotInv, pos));
    }

}
