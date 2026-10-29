using UnityEngine;
using Unity.Mathematics;
using NUnit.Framework;
using System;
using System.Runtime.CompilerServices;
using CsvHelper;


public class OptimizationManager
{
    private static double simDT = 1e-2;
    private static double timeOffset = 0.25;
    private static int timeIndexChange = (int)(timeOffset / simDT);
    static int numNoteConstraints = 1;
    static int numWallConstraints = 0;
    static int numBombConstraints = 0;


    private static double[] theta0;
    private static ConnectedLimbSystem robotSys;
    public static bool optRunning = false;


    private static (double[], double[]) StateDerivatives(double[] states, OptiInfoContainer optiInfo)
    {
        double[] velocities = new double[optiInfo.totalNumStates];
        double[] acceleraitons = new double[optiInfo.totalNumStates];

        //set the initial positions

        for(int i = 0; i < optiInfo.numTimeSteps - 1; i++)
        {
            //outer loop to get time index, inner loop to step through joint index
            for(int j = 0; j < robotSys.numInputs; j++)
            {
                velocities[j + i * robotSys.numInputs] = (states[j + (i + 1) * robotSys.numInputs] - states[j + i * robotSys.numInputs]) / simDT;
            }
        }
        for(int j = 0; j < robotSys.numInputs; j++)
        {
            velocities[j + (optiInfo.numTimeSteps - 1) * robotSys.numInputs] = velocities[j + (optiInfo.numTimeSteps - 2) * robotSys.numInputs];
        }

        for(int i = 0; i < optiInfo.numTimeSteps - 2; i++)
        {
            //outer loop to get time index, inner loop to step through joint index
            for(int j = 0; j < robotSys.numInputs; j++)
            {
                acceleraitons[j + i * robotSys.numInputs] = (velocities[j + (i + 1) * robotSys.numInputs] - velocities[j + i * robotSys.numInputs]) / simDT;
            }
        }

        return (velocities, acceleraitons);
    }


    public static void FindOptimalTrajectory(double startTime, double endTime)
    {
        Debug.Log("marker");
        optRunning = true;

        //robot setup
        robotSys = RobotManager.robotSystem;


        float songDuration = SongManager.GetSongLength();


        theta0 = new double[robotSys.numInputs];
        for(int i = 3; i < theta0.Length; i++)
        {
            theta0[i] = (robotSys.jointUpperLimits[i] + robotSys.jointLowerLimits[i]) / 2;
        }
        theta0[1] = 0.81f;

        OptiInfoContainer optiInfo = new OptiInfoContainer(ObjectManager.Instance, startTime, endTime);

        //initialize optimization guess and long bounds vectors
        double[] optiInitialStates = new double[optiInfo.totalNumStates];
        double[] optiLowerBounds = new double[optiInfo.totalNumStates];
        double[] optiUpperBounds = new double[optiInfo.totalNumStates];
        for(int i = 0; i < optiInfo.totalNumStates; i++)
        {
            optiInitialStates[i] = theta0[i % robotSys.numInputs];
            optiLowerBounds[i] = robotSys.jointLowerLimits[i % robotSys.numInputs];
            optiUpperBounds[i] = robotSys.jointUpperLimits[i % robotSys.numInputs];
        }
        double[] s = new double[optiInfo.totalNumStates];
        Array.Fill(s, 1);


        // ObjectManager.Instance.noteManager.Objects contains the notes when a map is loaded

        double[] constraintLowerBounds;
        double[] constraintUpperBounds;
        (constraintLowerBounds, constraintUpperBounds) = SetupConstraints(optiInfo);


        double epsx = 0.000001;
        int maxits = 0;
        alglib.minnlcstate state;
        alglib.minnlcreport rep;
        double[] optimalTraj = new double[optiInfo.totalNumStates];

        //
        // Create optimizer object and tune its settings:
        // * epsx=0.000001  stopping condition for inner iterations
        // * s=[1,1]        all variables have unit scale
        // * set ORBIT solver
        //
        alglib.minnlccreatef(optiInfo.totalNumStates, optiInitialStates, 1e-4, out state);
        alglib.minnlcsetcond(state, epsx, maxits);
        alglib.minnlcsetscale(state, s);
        alglib.minnlcsetalgosqp(state);
        alglib.minnlcsetbc(state, optiLowerBounds, optiUpperBounds);
        alglib.trace_file("SQP", "C:\\Program Files (x86)\\Steam\\steamapps\\common\\Beat Saber\\Beat Saber_Data\\CustomWIPLevels\\Mr. Capgras short\\trace.log");

        alglib.minnlcsetnlc2(state, constraintLowerBounds, constraintUpperBounds);

        Debug.Log("Starting Optimization");
        try
        {
            alglib.minnlcoptimize(state, OptimizerFunction, null, (object)optiInfo);
        }
        catch(alglib.alglibexception e)
        {
            Debug.LogError($"ALGLIB: {e.msg}");
            throw;
        }
        alglib.minnlcresults(state, out optimalTraj, out rep);
        Debug.Log("Finished Optimization");

        using var writer = CsvWriter.ToFile("F:\\OneDrive - purdue.edu\\Documents\\Beat Saber\\CustomWIPLevels\\Mr. Capgras short\\EasyOpt.csv");
        writer.WriteHeader("time", "State");
        for(int i = 0; i < optiInfo.numTimeSteps; i++)
        {
            writer.WriteRecord(optiInfo.times[i].ToString(), string.Join(", ", optimalTraj[(i * robotSys.numInputs)..((i + 1) * robotSys.numInputs - 1)]));
        }

    }
    private static (double[], double[]) SetupConstraints(OptiInfoContainer optiInfo)
    {
        //[note constraints, wall constraints, bomb constraints]
        optiInfo.totalConstraints = numNoteConstraints * optiInfo.noteObjects.Length + numWallConstraints * optiInfo.wallObjects.Length + numBombConstraints * optiInfo.bombObjects.Length;

        double[] constraintLowerBounds = new double[optiInfo.totalConstraints];
        double[] constraintUpperBounds = new double[optiInfo.totalConstraints];


        for(int i = 0; i < optiInfo.noteObjects.Length; i++)
        {
            constraintUpperBounds[numNoteConstraints * i] = 0.01 * 0.01; //115 scoring radius squared 
        }

        return (constraintLowerBounds, constraintUpperBounds);
    }
    private static double[] EvaluateConstraints(OptiInfoContainer optiInfo, double[] states, 
                                                double4x4[] lHandPoses, double4x4[] rhandPoses, 
                                                double4x4[] neckPoses)
    {
        double[] constraints = new double[optiInfo.totalConstraints];
        double njs = BeatmapManager.CurrentDifficulty.noteJumpSpeed;
        for(int noteIndex = 0; noteIndex < optiInfo.noteObjects.Length; noteIndex++)
        {
            Note curNote = optiInfo.noteObjects[noteIndex];
            int timeIndex = NearestTimeIndex(curNote.Time, optiInfo.times);

            //Debug.Log(timeIndex);
            //Debug.Log(optiInfo.times[timeIndex]);

            double4x4[] handPoses = rhandPoses;
            if(curNote.Color == 0)
            {
                handPoses = lHandPoses;
            }
            double positionErrorMagnitude = Mathf.Infinity;
            double3 positionError;
            for(int j = math.max(0, timeIndex - timeIndexChange); j <= timeIndex; j++)
            {
                positionError = SE3Functions.PosePos(handPoses[j]) - new double3(curNote.Position[0], curNote.Position[1], (curNote.Time - optiInfo.times[j]) * njs * 0.5);
                positionErrorMagnitude = math.min(positionErrorMagnitude, SE3Functions.Dot(positionError, positionError));
            }
            constraints[noteIndex * numNoteConstraints] = positionErrorMagnitude;
        }

        return constraints;
    }

    private static int NearestTimeIndex(float objectTime, double[] times)
    {
        int index = -1;
        for(int i = 0; i < times.Length; i++)
        {
            if(objectTime < times[i])
            {
                index = i;
                break;
            }
        }
        if(index == -1)
        {
            Debug.Log(new ArgumentOutOfRangeException($"{objectTime} not in time list"));
        }
        if(math.abs(objectTime - times[index]) > objectTime - times[index - 1])
        {
            index -= 1;
        }
        return index;
    }

    private static double EvaluateCost(double[] torques, double4x4[] pelvisPoses, OptiInfoContainer optiInfo)
    {
        double3 pelvisPosHolder;
        double springCost = 0;
        for(int i = 0; i < optiInfo.numTimeSteps; i++)
        {
            pelvisPosHolder = SE3Functions.PosePos(pelvisPoses[i]);
            springCost += SE3Functions.Dot(pelvisPosHolder, pelvisPosHolder);
        }
        return SE3Functions.Dot(torques, torques) + springCost;
    }

    public static void OptimizerFunction(double[] states, double[] foutput, object obj)
    {
        OptiInfoContainer optiInfo = (OptiInfoContainer)obj;
        double[] velocities;
        double[] accelerations;
        (velocities, accelerations) = StateDerivatives(states, optiInfo);

        double[] allTorques = new double[optiInfo.totalNumStates];
        double4x4[] lHandPoses = new double4x4[optiInfo.numTimeSteps];
        double4x4[] rhandPoses = new double4x4[optiInfo.numTimeSteps];
        double4x4[] neckPoses = new double4x4[optiInfo.numTimeSteps];
        double4x4[] pelvisPoses = new double4x4[optiInfo.numTimeSteps];

        double[] torqueHolder = new double[robotSys.numInputs];
        double4x4[] poseHolder = new double4x4[robotSys.numInputs];

        for(int i = 0; i < optiInfo.numTimeSteps; i++)
        {
            int startIndex = i * robotSys.numInputs;
            int stopIndex = startIndex + robotSys.numInputs;
            (torqueHolder, poseHolder) = robotSys.InverseDynamicsInputs(states[startIndex..stopIndex],
                                                                        velocities[startIndex..stopIndex],
                                                                        accelerations[startIndex..stopIndex]);
            lHandPoses[i] = poseHolder[optiInfo.lHandIndex];
            rhandPoses[i] = poseHolder[optiInfo.rHandIndex];
            neckPoses[i] = poseHolder[optiInfo.headIndex];
            pelvisPoses[i] = poseHolder[optiInfo.pelvisIndex];
            for(int j = 0; j < robotSys.numInputs; j++)
            {
                allTorques[i * robotSys.numInputs + j] = torqueHolder[j];
            }
        }


        double[] constraints = EvaluateConstraints(optiInfo, states, lHandPoses, rhandPoses, neckPoses);

        foutput[0] = EvaluateCost(allTorques, pelvisPoses, optiInfo);
        for(int i = 0; i < optiInfo.totalConstraints; i++)
        {
            foutput[i + 1] = constraints[i];
        }

        optiInfo.iterationCounter++;
        //Debug.Log($"Function call: {optiInfo.iterationCounter}");
    }
    public class OptiInfoContainer
    {
        public Note[] noteObjects;
        public Wall[] wallObjects;
        public Bomb[] bombObjects;
        public double[] times;
        public int totalConstraints;
        public int numTimeSteps;
        public int totalNumStates;
        public int headIndex;
        public int lHandIndex;
        public int rHandIndex;
        public int pelvisIndex;

        public int iterationCounter = 0;

        public OptiInfoContainer(ObjectManager manager, double startTime, double endTime)
        {
            this.times = new double[(int)((endTime - startTime + 2 * timeOffset) / simDT)];
            this.times[0] = startTime - timeOffset;
            for(int i = 1; i < times.Length; i++)
            {
                times[i] = times[i - 1] + simDT;
            }

            this.headIndex = robotSys.FindLimbByName("neck").inputOffset;
            this.lHandIndex = robotSys.FindLimbByName("leftHand").inputOffset;
            this.rHandIndex = robotSys.FindLimbByName("rightHand").inputOffset;
            this.pelvisIndex = robotSys.FindLimbByName("pelvis").inputOffset;
            this.numTimeSteps = times.Length;
            this.totalNumStates = numTimeSteps * robotSys.numInputs;


            MapElementList<Note> noteObjects = manager.noteManager.Objects;
            MapElementList<Wall> wallObjects = manager.wallManager.Objects;
            MapElementList<Bomb> bombObjects = manager.bombManager.Objects;

            int noteStart = noteObjects.GetFirstIndex((float)startTime, x => x.Time > (float)startTime);
            int noteEnd = noteObjects.GetLastIndex((float)startTime, x => x.Time < (float)endTime);
            this.noteObjects = new Note[noteEnd - noteStart];
            for(int i = 0; i < this.noteObjects.Length; i++)
            {
                this.noteObjects[i] = noteObjects[noteStart + i];
            }

            int wallStart = wallObjects.GetFirstIndex((float)startTime, x => x.Time > (float)startTime);
            int wallEnd = wallObjects.GetLastIndex((float)startTime, x => x.Time < (float)endTime);
            this.wallObjects = new Wall[wallEnd - wallStart];
            for(int i = 0; i < this.wallObjects.Length; i++)
            {
                this.wallObjects[i] = wallObjects[wallStart + i];
            }

            int bombStart = bombObjects.GetFirstIndex((float)startTime, x => x.Time > (float)startTime);
            int bombEnd = bombObjects.GetLastIndex((float)startTime, x => x.Time < (float)endTime);
            this.bombObjects = new Bomb[bombEnd - bombStart];
            for(int i = 0; i < this.bombObjects.Length; i++)
            {
                this.bombObjects[i] = bombObjects[bombStart + i];
            }
        }

    }
}

