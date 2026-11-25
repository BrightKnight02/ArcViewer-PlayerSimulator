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
        skewed = SE3.SkewedVector(w1);
        matrixexp = SE3.ScrewMatrixExponential(w1, v1, theta);
        rot = SE3.ExtractPoseRotation(matrixexp);
        rotInv = SE3.RotationInverse(rot);
        poseInv = SE3.PoseInverse(matrixexp);
        (bigAdjointW, bigAdjointW) = SE3.AdjointBigSE3(matrixexp, w2, v2);
        (smallAdjointW, smallAdjointW) = SE3.AdjointLittleSE3(w1, v1, w2, v2);

    }
}
