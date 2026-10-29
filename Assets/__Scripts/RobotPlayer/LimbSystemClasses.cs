using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class ConnectedLimbSystem
{
    public Limb root;
    public int numInputs;
    public double[] jointLowerLimits;
    public double[] jointUpperLimits;

    public double4x4 debugPose1;
    public double4x4 debugPose2;

    public double4x4[] relativePosesdb;

    public double3[] twistRotdb;
    public double3[] twistTransdb;

    public double3[] dTwistRotdb;
    public double3[] dTwistTransdb;

    public double3[] wrenchMomentsdb;
    public double3[] wrenchForcesdb;

    public GameObject sysObject;

    public ConnectedLimbSystem(Limb[] limbs, string name)
    {
        Debug.Log("Constructing Limb System");
        this.numInputs = 0;
        this.sysObject = new GameObject(name);

        // Set up limb tree to have links to other limb objects
        for(int i = 0; i < limbs.Length; i++)
        {
            this.numInputs += limbs[i].numInputs;
            limbs[i].visObject.transform.parent = this.sysObject.transform;

            //Debug.Log("Limb "+ limbs[i].name + " nextLimb Count: " + limbs[i].nextLimbNames.Length.ToString());
            Limb[] nextLimbs = new Limb[limbs[i].nextLimbNames.Length];
            int nextIndexTracker = 0;

            for(int j = 0; j < limbs.Length; j++)
            {
                if(limbs[j].name == limbs[i].prevLimbName)
                {
                    limbs[i].prevLimb = limbs[j];
                }
                else if(limbs[i].nextLimbNames.Contains(limbs[j].name))
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
        DetermineInputOffset(this.root, 0); // I think this should return the total number of inputs
        this.jointUpperLimits = new double[this.numInputs];
        this.jointLowerLimits = new double[this.numInputs];
        SetupArrays(this.root);
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

    private void SetupArrays(Limb limb)
    {
        Debug.Log("Setting " + limb.ToString() + " limits");
        for(int i = 0; i < limb.numInputs; i++)
        {
            int writeIndex = limb.inputOffset + i;
            this.jointLowerLimits[writeIndex] = limb.lowerJointLimits[i];
            this.jointUpperLimits[writeIndex] = limb.upperJointLimits[i];
        }
        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            SetupArrays(limb.nextLimbs[i]);
        }
    }

    public void OrientGameObjects(double[] positionVector)
    {
        //Debug.Log(positionVector.ToString());
        PoseWalker(positionVector, root, double4x4.identity);
    }
    private void PoseWalker(double[] positionVector, Limb limb, double4x4 prevPose)
    {
        double4x4 limbPose = math.mul(limb.RelativePoseFromInput(positionVector), prevPose);
        double4x4 worldPose = SE3Functions.PoseInverse(limbPose);

        limb.visObject.transform.position = new float3(SE3Functions.PosePos(worldPose));
        limb.visObject.transform.rotation = math.quaternion(new float3x3(SE3Functions.ExtractPoseRotation(worldPose)));

        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            PoseWalker(positionVector, limb.nextLimbs[i], limbPose);
        }
    }

    public (double[], double4x4[]) InverseDynamicsInputs(double[] states, double[] velocities, double[] accelerations)
    {
        // calculates the inverse dynamics of a Limb System based on the current joint states and derivatives.
        // impliments the Newton-Euler Inverse Dynamics Algorithm from Modern Robotics by Lynch and Park
        // as seen in section 8.3.2 with modifications for connected open chains.
        // The tree structure caused by connected open chains makes this annoying.

        double4x4[] relativePoses = new double4x4[states.Length];
        double4x4[] worldPoses = new double4x4[states.Length];

        double3[] twistRot = new double3[states.Length];
        double3[] twistTrans = new double3[states.Length];

        double3[] dTwistRot = new double3[states.Length];
        double3[] dTwistTrans = new double3[states.Length];

        InverseDynamicsForwardStep(this.root, states, velocities, accelerations, ref relativePoses, ref twistRot, ref twistTrans, ref dTwistRot, ref dTwistTrans);
        CalcWorldPoses(this.root, ref relativePoses, ref worldPoses);

        double3[] wrenchMoments = new double3[states.Length];
        double3[] wrenchForces = new double3[states.Length];

        InverseDynamicsBackwardStep(this.root, states, velocities, accelerations,
                                                   ref relativePoses, ref twistRot, ref twistTrans, ref dTwistRot,
                                                   ref dTwistTrans, ref wrenchMoments, ref wrenchForces);

        double[] jointInputs = new double[states.Length];
        CalcJointForces(this.root, ref wrenchMoments, ref wrenchForces, ref jointInputs);

        this.relativePosesdb = relativePoses;

        this.twistRotdb = twistRot;
        this.twistTransdb = twistTrans;

        this.dTwistRotdb = dTwistRot;
        this.dTwistTransdb = dTwistTrans;

        this.wrenchMomentsdb = wrenchMoments;
        this.wrenchForcesdb = wrenchForces;

        return (jointInputs, worldPoses);

    }
    private void InverseDynamicsForwardStep(Limb limb, double[] states, double[] velocities, double[] accelerations,
                                            ref double4x4[] relativePoses,
                                            ref double3[] twistRot, ref double3[] twistTrans,
                                            ref double3[] dTwistRot, ref double3[] dTwistTrans)
    {
        // used for unpacking
        double3 tempRot;
        double3 tempTrans;

        double3 prevTwistRot;
        double3 prevTwistTrans;
        double3 prevDTwistRot;
        double3 prevDTwistTrans;

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
                    prevTwistRot = double3.zero;
                    prevTwistTrans = double3.zero;
                    prevDTwistRot = double3.zero;
                    prevDTwistTrans = new double3(0, -9.81f, 0);
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

    private void InverseDynamicsBackwardStep(Limb limb, double[] states, double[] velocities, double[] accelerations,
                                            ref double4x4[] relativePoses,
                                            ref double3[] twistRot, ref double3[] twistTrans,
                                            ref double3[] dTwistRot, ref double3[] dTwistTrans,
                                            ref double3[] wrenchMoments, ref double3[] wrenchForces)
    {
        // sets the elements of wrenchMoments and wrenchForces corresponding to limb


        // used for unpacking
        double3 tempMoment;
        double3 tempForce;

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

                        wrenchMoments[writeIndex] += double3.zero;
                        wrenchForces[writeIndex] += double3.zero;
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
    
    private void CalcJointForces(Limb limb, ref double3[] wrenchMoments, ref double3[] wrenchForces, ref double[] jointInputs)
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
    private void CalcWorldPoses(Limb limb, ref double4x4[] relativePoses, ref double4x4[] worldPoses)
    {
        if(limb.inputOffset == 0)
        {
            worldPoses[0] = SE3Functions.PoseInverse(relativePoses[0]);
        }
        else
        {
            worldPoses[limb.inputOffset] = math.mul(worldPoses[limb.prevLimb.inputOffset + limb.prevLimb.numInputs - 1], SE3Functions.PoseInverse(relativePoses[limb.inputOffset]));
        }
        for(int i = 1; i < limb.numInputs; i++)
        {
            worldPoses[limb.inputOffset + i] = math.mul(worldPoses[limb.inputOffset + i - 1], SE3Functions.PoseInverse(relativePoses[limb.inputOffset + i]));
        }
        for(int i = 0; i < limb.nextLimbs.Length; i++)
        {
            CalcWorldPoses(limb.nextLimbs[i], ref relativePoses, ref worldPoses);
        }
    }
    public Limb FindLimbByName(string target)
    {
        Limb found = FindLimbByName(this.root, target);
        if(!found.IsUnityNull())
        {
            return found;
        }
        else
        {
            Debug.LogException(new NullReferenceException($"Limb by name {target} not found"));
            return null;
        }
    }
    private Limb FindLimbByName(Limb limb, string target)
    {
        //Debug.Log($"recursing to {limb.name}");
        if(target == limb.name)
        {
            return limb;
        }
        else if(limb.nextLimbs.Length == 0)
        {
            //Debug.Log($"exiting search down {limb.name} path");
            return null;
        }
        else
        {
            Limb holder;
            for(int i = 0; i < limb.nextLimbs.Length; i++)
            {
                holder = FindLimbByName(limb.nextLimbs[i], target);
                if(!holder.IsUnityNull())
                {
                    if(holder.name == target)
                    {
                        Debug.Log($"{target} found");
                        return holder;
                    }
                }
            }
            return null;
        }

    }
}

public class Limb
{
    // class to represent limbs as they have many inputs and controlled degrees of freedom

    public string name;
    public GameObject visObject;
    public double mass;
    public double3x3 inertia; // inertia tensor of the limb
    public int numInputs;
    public double[] lowerJointLimits;
    public double[] upperJointLimits;
    public int inputOffset;

    public string prevLimbName;
    public string[] nextLimbNames;
    // these are instantiated by an instance of ConnectedLimbSystem
    public Limb prevLimb;
    public Limb[] nextLimbs;

    public double4x4 homePose; // SE(3) matrix representing the link COM frame in base frame with all joint angles = 0

    // SE(3) matrix representing the previous link frame in the current link frame with all joint angles = 0
    // represents M_{i. i-1} from "Modern Robotics: Mechanics, Planning, and Control" by  Kevin M. Lynch and Frank Chongwoo Park
    public double4x4 relLinkPose;
    public double3[] BodyScrewRotAxes; // vectors in base frame representing the rotation axes for inputs at home
    public double3[] BodyScrewTransAxes; // vectors in base frame representing the translation axes for the inputs at home
    public double3[] LinkScrewRotAxes; // vectors in link frame representing the rotation axes for inputs at home
    public double3[] LinkScrewTransAxes; // vectors in link frame representing the translation axes for the inputs at home

    public Limb(string name, GameObject visObject, double limbMass, double3 limbInertia,
        string prevLimbName, string[] nextLimbNames, double4x4 homePose,
        double3[] screwRotAxes, double3[] screwPivotPoints,
        double[] lowerJointLimits = null, double[] upperJointLimits = null)
    {
        this.name = name;
        this.numInputs = screwRotAxes.Length;
        this.lowerJointLimits = new double[this.numInputs];
        if(lowerJointLimits == null)
        {
            for(int i = 0; i < this.numInputs; i++)
            {
                this.lowerJointLimits[i] = -math.INFINITY;
            }
        }
        else
        {
            for(int i = 0; i < this.numInputs; i++)
            {
                this.lowerJointLimits[i] = lowerJointLimits[i];
            }
        }
        this.upperJointLimits = new double[this.numInputs];
        if(upperJointLimits == null)
        {
            for(int i = 0; i < this.numInputs; i++)
            {
                this.upperJointLimits[i] = -math.INFINITY;
            }
        }
        else
        {
            for(int i = 0; i < this.numInputs; i++)
            {
                this.upperJointLimits[i] = upperJointLimits[i];
            }
        }
        this.visObject = visObject;
        this.mass = limbMass;
        this.inertia = new double3x3(limbInertia[0], 0, 0,
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
            if(screwRotAxes[i].Equals(double3.zero))
            {
                this.BodyScrewTransAxes[i] = screwPivotPoints[i];
            }
            else
            {
                this.BodyScrewTransAxes[i] = math.mul(-SE3Functions.SkewedVector(screwRotAxes[i]), screwPivotPoints[i]);
            }
        }

        this.LinkScrewRotAxes = new double3[numInputs];
        this.LinkScrewTransAxes = new double3[numInputs];


        for(int i = 0; i < numInputs; i++)
        {
            (this.LinkScrewRotAxes[i], this.LinkScrewTransAxes[i]) = SE3Functions.AdjointBigSE3(SE3Functions.PoseInverse(homePose), this.BodyScrewRotAxes[i], this.BodyScrewTransAxes[i]);
        }

    }
    public double4x4 RelativePoseFromInput(double[] positionsVector)
    {
        // gives the transform from the this limb to the previous limb with rotations
        double4x4 output = double4x4.identity;
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

