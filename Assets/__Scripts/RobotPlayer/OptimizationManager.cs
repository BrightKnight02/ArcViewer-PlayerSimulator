using UnityEngine;
using Unity.Mathematics;
using NUnit.Framework;
using System;


public class OptimizationManager
{
    private static float simDT = 1e-2f;
    private static float[] theta0;
    private static ConnectedLimbSystem robotSys;
    private static int numTimeSteps;
    private static int totalNumStates;

    private static (float[], float[]) StatesFromAccelerations(float[] accelerations)
    {
        float[] states = new float[totalNumStates];
        float[] velocities = new float[totalNumStates];

        //set the initial positions
        for(int i = 0; i < theta0.Length; i++)
        {
            states[i] = theta0[i];
        }


        for(int i = 1; i < numTimeSteps; i++)
        {
            //outer loop to get time index, inner loop to step through joint index
            for(int j = 0; j < theta0.Length; j++)
            {
                //forward euler of accelerations and velocities
                velocities[j + i * theta0.Length] = velocities[j + (i - 1) * theta0.Length] + accelerations[j + (i - 1) * theta0.Length] * simDT;
                states[j + i * theta0.Length] = states[j + (i - 1) * theta0.Length] + velocities[j + (i - 1) * theta0.Length] * simDT;
            }
        }

        return (states, velocities);
    }

    private static (float[], float[]) PositonsInBounds(float[] states)
    {
        // constraints for keeping all the joint positions within specified bounds

        float[] uppers = new float[totalNumStates];
        float[] lowers = new float[totalNumStates];
        for(int i = 1; i < numTimeSteps; i++)
        {
            //outer loop to get time index, inner loop to step through joint index
            for(int j = 0; j < theta0.Length; j++)
            {
                //two lists for lowerbound <= theta <= upper bound
                // same as lowerbound - theta <= 0 and theta - upperbound <= 0
                uppers[j + i * theta0.Length] = states[j + i * theta0.Length] - robotSys.jointUpperLimits[j];
                lowers[j + i * theta0.Length] = robotSys.jointLowerLimits[j] - states[j + i * theta0.Length];
            }
        }

        return (uppers, lowers);
    }

    public static void FindOptimalTrajectory()
    {
        robotSys = RobotManager.robotSystem;
        float songDuration = SongManager.GetSongLength();
        numTimeSteps = (int)(songDuration / simDT);

        theta0 = new float[robotSys.totalInputs];
        for(int i = 3; i < theta0.Length; i++)
        {
            theta0[i] = (robotSys.jointUpperLimits[i] + robotSys.jointLowerLimits[i]) / 2;
        }
        theta0[1] = 0.81f;
        totalNumStates = numTimeSteps * robotSys.totalInputs;

        double[] dthetas0 = new double[totalNumStates];
        double[] s = new double[totalNumStates];
        Array.Fill(s, 1);

        double epsx = 0.000001;
        int maxits = 0;
        alglib.minnlcstate state;
        alglib.minnlcreport rep;
        double[] x1;

        //
        // Create optimizer object and tune its settings:
        // * epsx=0.000001  stopping condition for inner iterations
        // * s=[1,1]        all variables have unit scale
        // * set ORBIT solver
        //
        alglib.minnlccreate(totalNumStates, dthetas0, out state);
        alglib.minnlcsetcond(state, epsx, maxits);
        alglib.minnlcsetscale(state, s);
        alglib.minnlcsetalgoorbit(state, 0, 0);
    }
}
