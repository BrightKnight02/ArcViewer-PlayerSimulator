using JetBrains.Annotations;
using UnityEngine;
using Unity.Mathematics;
using NUnit.Framework;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;

public class ConnectedLimbSystem
{
    public Limb root;
    public int totalInputs;

    public float4x4 debugPose1;
    public float4x4 debugPose2;

    public ConnectedLimbSystem(Limb[] limbs)
    {
        Debug.Log("Reaching System Construction");
        this.totalInputs = 0;

        // Set up limb tree to have links to other limb objects
        for(int i = 0; i < limbs.Length; i++)
        {
            this.totalInputs += limbs[i].numInputs;

            //Debug.Log("Limb "+ limbs[i].name + " nextLimb Count: " + limbs[i].nextLimbNames.Length.ToString());
            Limb[] nextLimbs = new Limb[limbs[i].nextLimbNames.Length];
            int nextIndexTracker = 0;

            for(int j = 0; j < limbs.Length; j++)
            {
                if(limbs[j].name == limbs[i].prevLimbName)
                {
                    limbs[i].prevLimb = limbs[j];
                }
                if(limbs[i].nextLimbNames.Contains(limbs[j].name))
                {
                    nextLimbs[nextIndexTracker] = (limbs[j]);
                    nextIndexTracker++;
                }
            }
            limbs[i].nextLimbs = nextLimbs;

            if(limbs[i].prevLimbName == null)
            {
                this.root = limbs[i];
                limbs[i].relLinkPose = SE3Functions.PoseInverse(limbs[i].homePose);
            }
            else
            {
                limbs[i].relLinkPose = math.mul(SE3Functions.PoseInverse(limbs[i].homePose), limbs[i].prevLimb.homePose);
            }
        }
        DetermineInputOffset(root, 0); // I think this should return the total number of inputs
    }

    private int DetermineInputOffset(Limb limb, int currentOffset)
    {
        // recursively step through the limb tree to find the start index of the inputs
        // to all the limbs for a flat input array
        limb.inputOffset = currentOffset;
        int offsetChange = limb.numInputs;

        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            offsetChange += DetermineInputOffset(limb.nextLimbs[i], currentOffset + offsetChange);
        }
        return offsetChange;
    }

    public void OrientGameObjects(float[] positionVector)
    {
        //Debug.Log(positionVector.ToString());
        PoseWalker(positionVector, root, float4x4.identity);
    }
    private void PoseWalker(float[] positionVector, Limb limb, float4x4 prevPose)
    {
        float4x4 limbPose = math.mul(limb.RelativePoseFromInput(positionVector), prevPose);
        float4x4 worldPose = SE3Functions.PoseInverse(limbPose);
        limb.visObject.transform.position = SE3Functions.PosePos(worldPose);
        limb.visObject.transform.rotation = math.quaternion(new float3x3(worldPose));
        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            PoseWalker(positionVector, limb.nextLimbs[i], limbPose);
        }
        if(limb == this.root)
        {
            debugPose1 = worldPose;
            //Debug.Log(limb.visObject.transform.position);
        } 
        else if(limb != this.root)
        {
            debugPose2 = worldPose;
        }
    }
}

public class Limb
{
    // class to represent limbs as they have many inputs and controlled degrees of freedom

    public string name;
    public GameObject visObject;
    public float limbMass;
    public float3x3 limbInertia; // inertia tensor of the limb
    public int numInputs;
    public int inputOffset;

    public string prevLimbName;
    public string[] nextLimbNames;
    // these are instantiated by an instance of ConnectedLimbSystem
    public Limb prevLimb;
    public Limb[] nextLimbs;

    public float4x4 homePose; // SE(3) matrix representing the link COM frame in base frame with all joint angles = 0

    // SE(3) matrix representing the previous link frame in the current link frame with all joint angles = 0
    // represents M_{i. i-1} from "Modern Robotics: Mechanics, Planning, and Control" by  Kevin M. Lynch and Frank Chongwoo Park
    public float4x4 relLinkPose;
    public float3[] BodyScrewRotAxes; // vectors in base frame representing the rotation axes for inputs at home
    public float3[] BodyScrewTransAxes; // vectors in base frame representing the translation axes for the inputs at home
    private float3[] LinkScrewRotAxes; // vectors in link frame representing the rotation axes for inputs at home
    private float3[] LinkScrewTransAxes; // vectors in link frame representing the translation axes for the inputs at home

    public Limb(string name, GameObject visObject, float limbMass, float3 limbInertia, int numInputs, string prevLimbName, string[] nextLimbNames, float4x4 homePose, float3[] screwRotAxes, float3[] screwPivotPoints)
    {
        this.name = name;
        this.visObject = visObject;
        this.limbMass = limbMass;
        this.limbInertia = new float3x3(limbInertia[0], 0, 0,
                                        0, limbInertia[1], 0,
                                        0, 0, limbInertia[2]);
        this.numInputs = numInputs;
        this.prevLimbName = prevLimbName;
        if(nextLimbNames == null)
        {
            Debug.Log("Null nextlimbs");
            this.nextLimbNames = new string[] { };
        }
        else
        {
            this.nextLimbNames = nextLimbNames;
        }
        this.homePose = homePose;


        this.BodyScrewRotAxes = screwRotAxes;
        this.BodyScrewTransAxes = screwPivotPoints;
        for(int i = 0; i < numInputs; i++)
        {
            this.BodyScrewTransAxes[i] = math.mul(-SE3Functions.SkewedVector(screwRotAxes[i]), screwPivotPoints[i]);
        }

        this.LinkScrewRotAxes = new float3[numInputs];
        this.LinkScrewTransAxes = new float3[numInputs];


        for(int i = 0; i < numInputs; i++)
        {
            (this.LinkScrewRotAxes[i], this.LinkScrewTransAxes[i]) = SE3Functions.AdjointBigSE3(SE3Functions.PoseInverse(homePose), this.BodyScrewRotAxes[i], this.BodyScrewTransAxes[i]);
        }

    }
    public float4x4 RelativePoseFromInput(float[] positionsVector)
    {
        // gives the transform from the this limb to the previous limb with rotations
        float4x4 output = float4x4.identity; 
        for(int i = 0; i < this.numInputs; i++)
        {
            output = math.mul(SE3Functions.ScrewMatrixExponential(-1*this.LinkScrewRotAxes[i], -1*this.LinkScrewTransAxes[i], positionsVector[this.inputOffset + i]), output);
        }
        return math.mul(output, this.relLinkPose);
    }
}

