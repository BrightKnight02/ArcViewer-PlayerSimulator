using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

public class Mathtest : MonoBehaviour
{
    [Header("Test Inputs")]
    public double3 w1;
    public double3 v1;
    public double3 w2;
    public double3 v2;
    public double theta;

    [Header("Test Outputs")]
    public double3x3 skewed;
    public double4x4 matrixexp;
    public double3x3 rot;
    public double3x3 rotInv;
    public double4x4 poseInv;
    public double3 bigAdjointW;
    public double3 bigAdjointV;
    public double3 smallAdjointW;
    public double3 smallAdjointV;

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
        rot = SE3Functions.ExtractPoseRotation(matrixexp);
        rotInv = SE3Functions.RotationInverse(rot);
        poseInv = SE3Functions.PoseInverse(matrixexp);
        (bigAdjointW, bigAdjointW) = SE3Functions.AdjointBigSE3(matrixexp, w2, v2);
        (smallAdjointW, smallAdjointW) = SE3Functions.AdjointLittleSE3(w1, v1, w2, v2);

    }
}
