using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEditor.VersionControl.Asset;

public class ConnectedLimbSystem
{
    public Limb root;
    public int totalInputs;


    public float4x4 debugPose1;
    public float4x4 debugPose2;

    public float4x4[] relativePosesdb;

    public float3[] twistRotdb;
    public float3[] twistTransdb;

    public float3[] dTwistRotdb;
    public float3[] dTwistTransdb;

    public float3[] wrenchMomentsdb;
    public float3[] wrenchForcesdb;

    public ConnectedLimbSystem(Limb[] limbs)
    {
        Debug.Log("Constructing Limb System");
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
        Debug.Log("Before calculating offsets");
        DetermineInputOffset(root, 0); // I think this should return the total number of inputs
        Debug.Log("Finished limb system construction");
    }

    private int DetermineInputOffset(Limb limb, int currentOffset)
    {
        // recursively step through the limb tree to find the start index of the inputs
        // to all the limbs for a flat input array
        Debug.Log("Finding " + limb.ToString() + " offset");
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
    }

    public float[] InverseDynamicsInputs(float[] states, float[] velocities, float[] accelerations)
    {
        // calculates the inverse dynamics of a Limb System based on the current joint states and derivatives.
        // impliments the Newton-Euler Inverse Dynamics Algorithm from Modern Robotics by Lynch and Park
        // as seen in section 8.3.2 with modifications for connected open chains.
        // The tree structure caused by connected open chains makes this annoying.

        float4x4[] relativePoses = new float4x4[states.Length];

        float3[] twistRot = new float3[states.Length];
        float3[] twistTrans = new float3[states.Length];

        float3[] dTwistRot = new float3[states.Length];
        float3[] dTwistTrans = new float3[states.Length];

        InverseDynamicsForwardStep(this.root, states, velocities, accelerations, ref relativePoses, ref twistRot, ref twistTrans, ref dTwistRot, ref dTwistTrans);

        float3[] wrenchMoments = new float3[states.Length];
        float3[] wrenchForces = new float3[states.Length];

        InverseDynamicsBackwardStep(this.root, states, velocities, accelerations,
                                                   ref relativePoses, ref twistRot, ref twistTrans, ref dTwistRot,
                                                   ref dTwistTrans, ref wrenchMoments, ref wrenchForces);

        float[] jointInputs = new float[states.Length];
        CalcJointForces(this.root, ref wrenchMoments, ref wrenchForces, ref jointInputs);

        relativePosesdb = relativePoses;

        twistRotdb = twistRot;
        twistTransdb = twistTrans;

        dTwistRotdb = dTwistRot;
        dTwistTransdb = dTwistTrans;

        wrenchMomentsdb = wrenchMoments;
        wrenchForcesdb = wrenchForces;

        return jointInputs;

    }
    private void InverseDynamicsForwardStep(Limb limb, float[] states, float[] velocities, float[] accelerations,
                                            ref float4x4[] relativePoses,
                                            ref float3[] twistRot, ref float3[] twistTrans,
                                            ref float3[] dTwistRot, ref float3[] dTwistTrans)
    {
        // used for unpacking
        float3 tempRot;
        float3 tempTrans;

        float3 prevTwistRot;
        float3 prevTwistTrans;
        float3 prevDTwistRot;
        float3 prevDTwistTrans;

        for(int inputIndex = 0; inputIndex < limb.numInputs; inputIndex++)
        {
            int writeIndex = inputIndex + limb.inputOffset;
            // T_{i, i-1} = exp(-[A_i] \theta_i) * M_{i, i-1}
            relativePoses[writeIndex] = SE3Functions.ScrewMatrixExponential(-limb.LinkScrewRotAxes[inputIndex], -limb.LinkScrewTransAxes[inputIndex], states[writeIndex]);
            if(inputIndex == 0)
            {
                relativePoses[writeIndex] = math.mul(relativePoses[writeIndex], limb.relLinkPose);
            }
            // the pose calculations is technically repeated code from Limb but I don't care. I can't get all the poses easily from how that function is defined.

            // selecting the correct V_{i-1} and \dot V_{i-1} values
            if(inputIndex == 0)
            {
                if(writeIndex == 0)
                {
                    prevTwistRot = float3.zero;
                    prevTwistTrans = float3.zero;
                    prevDTwistRot = float3.zero;
                    prevDTwistTrans = new float3(0, -9.81f, 0);
                }
                else
                {
                    int prevLimbIndex = limb.prevLimb.inputOffset + limb.prevLimb.numInputs - 1;
                    prevTwistRot = twistRot[prevLimbIndex];
                    prevTwistTrans = twistTrans[prevLimbIndex];
                    prevDTwistRot = dTwistRot[prevLimbIndex];
                    prevDTwistTrans = dTwistTrans[prevLimbIndex];
                }
            }
            else
            {
                int prevWrite = writeIndex - 1;
                prevTwistRot = twistRot[prevWrite];
                prevTwistTrans = twistTrans[prevWrite];
                prevDTwistRot = dTwistRot[prevWrite];
                prevDTwistTrans = dTwistTrans[prevWrite];
            }

            // V_i = Ad_{T_{i, i-1}}(V_{i-1}) + A_i * \dot\theta
            (tempRot, tempTrans) = SE3Functions.AdjointBigSE3(relativePoses[writeIndex], prevTwistRot, prevTwistTrans);
            twistRot[writeIndex] = limb.LinkScrewRotAxes[inputIndex] * velocities[writeIndex] + tempRot;
            twistTrans[writeIndex] = limb.LinkScrewTransAxes[inputIndex] * velocities[writeIndex] + tempTrans;

            // \dot V_i = Ad_{T_{i, i-1}}(\dot V_{i-1}) + ad_{V_i}(A_i) * \dot\theta + A_i * \ddot\theta
            (dTwistRot[writeIndex], dTwistTrans[writeIndex]) = SE3Functions.AdjointBigSE3(relativePoses[writeIndex], prevDTwistRot, prevDTwistTrans);
            (tempRot, tempTrans) = SE3Functions.AdjointLittleSE3(twistRot[writeIndex], twistTrans[writeIndex], limb.LinkScrewRotAxes[inputIndex], limb.LinkScrewTransAxes[inputIndex]);
            dTwistRot[writeIndex] += tempRot * velocities[writeIndex] + limb.LinkScrewRotAxes[inputIndex] * accelerations[writeIndex];
            dTwistTrans[writeIndex] += tempTrans * velocities[writeIndex] + limb.LinkScrewTransAxes[inputIndex] * accelerations[writeIndex];
        }
        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            InverseDynamicsForwardStep(limb.nextLimbs[i], states, velocities, accelerations, ref relativePoses, ref twistRot, ref twistTrans, ref dTwistRot, ref dTwistTrans);
        }
    }

    private void InverseDynamicsBackwardStep(Limb limb, float[] states, float[] velocities, float[] accelerations,
                                            ref float4x4[] relativePoses,
                                            ref float3[] twistRot, ref float3[] twistTrans,
                                            ref float3[] dTwistRot, ref float3[] dTwistTrans,
                                            ref float3[] wrenchMoments, ref float3[] wrenchForces)
    {
        // sets the elements of wrenchMoments and wrenchForces corresponding to limb


        // used for unpacking
        float3 tempMoment;
        float3 tempForce;

        for(int inputIndex = limb.numInputs - 1; inputIndex > -1; inputIndex--)
        {
            int writeIndex = inputIndex + limb.inputOffset;

            // Only do inertia calculations
            if(inputIndex == limb.numInputs - 1)
            {
                // F_i = sum(Ad_{T_{next, i}^T F_{next}) + G_i \dot V_i - ad_{V_i}^T (G_i V_i)
                wrenchMoments[writeIndex] = math.mul(limb.inertia, dTwistRot[writeIndex]);
                wrenchForces[writeIndex] = limb.mass * dTwistTrans[writeIndex];

                (tempMoment, tempForce) = SE3Functions.AdjointTransposeLittleSE3(twistRot[writeIndex], twistTrans[writeIndex],
                    math.mul(limb.inertia, dTwistRot[writeIndex]), limb.mass * twistTrans[writeIndex]);
                wrenchMoments[writeIndex] -= tempMoment;
                wrenchForces[writeIndex] -= tempForce;
                for(int outputIndex = 0; outputIndex < limb.nextLimbs.Length; outputIndex++)
                {
                    if(limb.nextLimbs.Length == 0)
                    {
                        // contact force calc to add to end limbs goes here

                        wrenchMoments[writeIndex] += float3.zero;
                        wrenchForces[writeIndex] += float3.zero;
                    }
                    else
                    {

                        int nextLimbIndex = limb.nextLimbs[outputIndex].inputOffset;
                        // calculates the wrench of nextLimb[outputIndex] before adding
                        // its contribution to the wrench of this limb
                        InverseDynamicsBackwardStep(limb.nextLimbs[outputIndex], states, velocities, accelerations,
                                                ref relativePoses, ref twistRot, ref twistTrans, ref dTwistRot, ref dTwistTrans, ref wrenchMoments, ref wrenchForces);

                        (tempMoment, tempForce) = SE3Functions.AdjointTransposeBigSE3(relativePoses[nextLimbIndex], wrenchMoments[nextLimbIndex], wrenchForces[nextLimbIndex]);
                        wrenchMoments[writeIndex] += tempMoment;
                        wrenchForces[writeIndex] += tempForce;
                    }
                }
            }
            else
            {
                // sublinks for a limb have no inertial components, so only the component from the link upstream contributes
                (wrenchMoments[writeIndex], wrenchForces[writeIndex]) = SE3Functions.AdjointTransposeBigSE3(relativePoses[writeIndex + 1],
                                                                                wrenchMoments[writeIndex + 1], wrenchForces[writeIndex + 1]);
            }
        }
    }
    private void CalcJointForces(Limb limb, ref float3[] wrenchMoments, ref float3[] wrenchForces, ref float[] jointInputs)
    {
        for(int i = 0; i < limb.numInputs; i++)
        {
            int writeIndex = limb.inputOffset + i;
            jointInputs[writeIndex] = math.mul(limb.LinkScrewRotAxes[i], wrenchMoments[writeIndex]) + math.mul(limb.LinkScrewTransAxes[i], wrenchForces[writeIndex]);
        }
        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            CalcJointForces(limb.nextLimbs[i], ref wrenchMoments, ref wrenchForces, ref jointInputs);
        }
    }
}

public class Limb
{
    // class to represent limbs as they have many inputs and controlled degrees of freedom

    public string name;
    public GameObject visObject;
    public float mass;
    public float3x3 inertia; // inertia tensor of the limb
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
    public float3[] LinkScrewRotAxes; // vectors in link frame representing the rotation axes for inputs at home
    public float3[] LinkScrewTransAxes; // vectors in link frame representing the translation axes for the inputs at home

    public Limb(string name, GameObject visObject, float limbMass, float3 limbInertia, string prevLimbName, string[] nextLimbNames, float4x4 homePose, float3[] screwRotAxes, float3[] screwPivotPoints)
    {
        this.name = name;
        this.numInputs = screwRotAxes.Length;
        this.visObject = visObject;
        this.mass = limbMass;
        this.inertia = new float3x3(limbInertia[0], 0, 0,
                                        0, limbInertia[1], 0,
                                        0, 0, limbInertia[2]);
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
            output = math.mul(SE3Functions.ScrewMatrixExponential(-1 * this.LinkScrewRotAxes[i], -1 * this.LinkScrewTransAxes[i], positionsVector[this.inputOffset + i]), output);
        }
        return math.mul(output, this.relLinkPose);
    }

    public override string ToString()
    {
        return base.ToString() + ": " + this.name + "(" + this.numInputs.ToString() + ")";
    }
}

