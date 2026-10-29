using UnityEngine;
using Unity.Mathematics;

public class TestLimbSystem : MonoBehaviour
{

    public GameObject limbPrefab;


    public double[] theta;
    public double[] dtheta;
    public double[] ddtheta;
    public double[] tau;
    public double4x4[] worldPoses;

    public double4x4 debugPose1;
    public double4x4 debugPose2;

    public double4x4[] relativePosesdb;

    public double3[] twistRotdb;
    public double3[] twistTransdb;

    public double3[] dTwistRotdb;
    public double3[] dTwistTransdb;

    public double3[] wrenchMomentsdb;
    public double3[] wrenchForcesdb;

    private double[] thetaMod;

    //limb1 definitions
    private static double4x4 limb1Home = new double4x4(1, 0, 0, 0,
                                              0, 1, 0, 0.5f,
                                              0, 0, 1, 1,
                                              0, 0, 0, 1);
    private static double3[] limb1ScrewRot = { new double3(1, 0, 0),
                                              new double3(0, 1, 0) };
    private static double3[] limb1ScrewPos = { new double3(0, 0, 1),
                                              new double3(0, 0, 1) };

    //limb2 definitions
    private static double4x4 limb2Home = new double4x4(1, 0, 0, 0,
                                                     0, 1, 0, 1.5f,
                                                     0, 0, 1, 1,
                                                     0, 0, 0, 1);
    private static double3[] limb2ScrewRot = { new double3(0, 0, 1) };
    private static double3[] limb2ScrewPos = { new double3(0, 1, 1) };

    //limb3 definitions
    private static double4x4 limb3Home = new double4x4(1, 0, 0, 0,
                                                     0, 1, 0, 2.5f,
                                                     0, 0, 1, 1,
                                                     0, 0, 0, 1);
    private static double3[] limb3ScrewRot = { new double3(1, 0, 0) };
    private static double3[] limb3ScrewPos = { new double3(0, 2, 1) };


    static ConnectedLimbSystem testLimbSys;


    private void Start()
    {
        Limb limb1 = new Limb("limb1", Object.Instantiate(limbPrefab), 12, new double3(1.0625f, .125f, 1.0625f), null, new string[] { "limb2" }, limb1Home, limb1ScrewRot, limb1ScrewPos);
        Limb limb2 = new Limb("limb2", Object.Instantiate(limbPrefab), 12, new double3(1.0625f, .125f, 1.0625f), "limb1", new string[] { "limb3" }, limb2Home, limb2ScrewRot, limb2ScrewPos);
        Limb limb3 = new Limb("limb3", Object.Instantiate(limbPrefab), 12, new double3(1.0625f, .125f, 1.0625f), "limb2", null, limb3Home, limb3ScrewRot, limb3ScrewPos);
        Limb[] testLimbs = { limb1, limb2, limb3 };
        testLimbSys = new ConnectedLimbSystem(testLimbs, "test");

        thetaMod = new double[testLimbSys.numInputs];
        theta = new double[testLimbSys.numInputs];
        dtheta = new double[testLimbSys.numInputs];
        ddtheta = new double[testLimbSys.numInputs];
        tau = new double[testLimbSys.numInputs];
    }

    void Update()
    {
        //double theta1 = SettingsManager.Getdouble("usertheta1") * math.PI;
        //double theta2 = SettingsManager.Getdouble("usertheta2") * math.PI;
        //double theta3 = SettingsManager.Getdouble("usertheta3") * math.PI;

        //double[] theta = { theta1, theta2, theta3 };

        //Debug.Log("Theta: " + theta.ToString());
        for(int i = 0; i < theta.Length; i++)
        {
            thetaMod[i] = theta[i] * math.PI;
        }
        testLimbSys.OrientGameObjects(thetaMod);
        debugPose1 = testLimbSys.debugPose1;
        debugPose2 = testLimbSys.debugPose2;
        (tau, worldPoses) = testLimbSys.InverseDynamicsInputs(thetaMod, dtheta, ddtheta);

        relativePosesdb = testLimbSys.relativePosesdb;

        twistRotdb = testLimbSys.twistRotdb;
        twistTransdb = testLimbSys.twistTransdb;

        dTwistRotdb = testLimbSys.dTwistRotdb;
        dTwistTransdb = testLimbSys.dTwistTransdb;

        wrenchMomentsdb = testLimbSys.wrenchMomentsdb;
        wrenchForcesdb = testLimbSys.wrenchForcesdb;
    }

}
