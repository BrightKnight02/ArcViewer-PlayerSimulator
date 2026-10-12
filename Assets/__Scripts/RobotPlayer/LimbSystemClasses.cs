using JetBrains.Annotations;
using UnityEngine;
using Unity.Mathematics;
using NUnit.Framework;
using System.Collections.Generic;
using System.Linq;

public class ConnectedLimbSystem
{
    public Limb root;
    public int totalInputs;

    public ConnectedLimbSystem(Limb[] limbs)
    {
        Debug.Log("Reaching System Construction");
        this.totalInputs = 0;

        // Set up limb tree to have links to other limb objects
        for(int i = 0; i < limbs.Length; i++)
        {
            this.totalInputs += limbs[i].numInputs;
            List<Limb> nextLimbs = new List<Limb>();

            for(int j = 0; j < limbs.Length; j++)
            {
                if(limbs[j].name == limbs[i].prevLimbName)
                {
                    limbs[i].prevLimb = limbs[j];
                }
                if(limbs[i].nextLimbNames.Contains(limbs[j].name))
                {
                    nextLimbs.Add(limbs[j]);
                }
            }
            limbs[i].nextLimbs = nextLimbs.ToArray();

            if(limbs[i].prevLimbName == null)
            {
                this.root = limbs[i];
                limbs[i].relLinkPose = SE3Functions.PoseInverse(limbs[i].homePose);
            }
            else
            {
                limbs[i].relLinkPose = math.mul(SE3Functions.PoseInverse(limbs[i].homePose), limbs[i].prevLimb.homePose);
            }
            DetermineInputOffset(root, 0); // I think this should return the total number of inputs
        }
    }

    private int DetermineInputOffset(Limb root, int currentOffset)
    {
        // recursively step through the limb tree to find the start index of the inputs
        // to all the limbs for a flat input array
        root.inputOffset = currentOffset;
        int offsetChange = root.numInputs;
        //Debug.Log(root.name);

        for(int i = 0; i < root.nextLimbs.Length; i++)
        {
            offsetChange += DetermineInputOffset(root.nextLimbs[i], currentOffset + offsetChange);
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

    public Limb(string name, GameObject visObject, float limbMass, float3 limbInertia, int numInputs, string prevLimbName, string[] nextLimbsNames, float4x4 homePose, float3[] screwRotAxes, float3[] screwTransAxes = null)
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
            this.nextLimbNames = new string[] { };
        }
        else
        {
            this.nextLimbNames = nextLimbsNames;
        }
        this.homePose = homePose;


        this.BodyScrewRotAxes = screwRotAxes;
        if(screwTransAxes == null)
        {
            this.BodyScrewTransAxes = new float3[numInputs];
            for(int i = 0; i < numInputs; i++)
            {
                this.BodyScrewTransAxes[i] = float3.zero;
            }
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
        // gives the transform from the previous limb to this limb with rotations
        float4x4 output = math.mul(SE3Functions.ScrewMatrixExponential(this.LinkScrewRotAxes[0], this.LinkScrewTransAxes[0], positionsVector[this.inputOffset]), this.relLinkPose);
        for(int i = 1; i < this.numInputs; i++)
        {
            output = math.mul(SE3Functions.ScrewMatrixExponential(this.LinkScrewRotAxes[i], this.LinkScrewTransAxes[i], positionsVector[this.inputOffset + i]), output);
        }
        return output;
    }
}

