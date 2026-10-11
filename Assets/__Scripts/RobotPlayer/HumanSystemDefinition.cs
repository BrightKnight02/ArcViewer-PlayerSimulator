using JetBrains.Annotations;
using UnityEngine;
using Unity.Mathematics;

public class HumanSystemDefinition
{
    
}

public class Limb
{
    public int numInputs;
    public Limb prevLimb;
    public Limb[] nextLimbs;
    public float4x4 homePose; // SE(3) matrix representing the link COM frame in base frame with all joint angles = 0
    public float3[] screwRotAxes; // vectors in base frame representing the rotation axes for inputs
    public float3[] screwTransAxes; // vectors in base frame representing the translation axes for the inputs

    // class to compactly represent limbs as they have many inputs and axes
    public Limb(int numInputs, Limb prevLimb, Limb[] nextLimbs, float4x4 homePose, float3[] screwRotAxes, float3[] screwTransAxes = null)
    {
        this.numInputs = numInputs;
        this.prevLimb = prevLimb;
        this.nextLimbs = nextLimbs;
        this.homePose = homePose;
        this.screwRotAxes = screwRotAxes;
        if (screwTransAxes == null)
        {
            this.screwTransAxes = new float3[numInputs];
            for(int i = 0; i < numInputs; i++)
            {
                this.screwTransAxes[i] = float3.zero;
            }
        }
    }


}

