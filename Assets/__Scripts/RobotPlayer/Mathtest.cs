using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

public class Mathtest : MonoBehaviour
{
    [Header("Test Inputs")]
    public float3 w1;
    public float3 v1;
    public float3 w2;
    public float3 v2;
    public float theta;

    [Header("Test Outputs")]
    public float3x3 skewed;
    public float4x4 matrixexp;
    public float3x3 rot;
    public float3x3 rotInv;
    public float4x4 poseInv;
    public float3 bigAdjointW;
    public float3 bigAdjointV;
    public float3 smallAdjointW;
    public float3 smallAdjointV;

    void Start()
    {
        List<int> testing = new List<int>();
        Debug.Log(testing.ToArray().Length);
    }

    // Update is called once per frame
    void Update()
    {
        skewed = SE3Functions.SkewedVector(w1);
        matrixexp = SE3Functions.ScrewMatrixExponential(w1, v1, theta);
        rot = new float3x3(matrixexp);
        rotInv = SE3Functions.RotationInverse(rot);
        poseInv = SE3Functions.PoseInverse(matrixexp);
        (bigAdjointW, bigAdjointW) = SE3Functions.AdjointBigSE3(matrixexp, w2, v2);
        (smallAdjointW, smallAdjointW) = SE3Functions.AdjointLittleSE3(w1, v1, w2, v2);

    }
}
