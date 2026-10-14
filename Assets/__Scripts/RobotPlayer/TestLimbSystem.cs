using UnityEngine;
using Unity.Mathematics;

public class TestLimbSystem : MonoBehaviour
{

    public GameObject limbPrefab;


    public float[] theta;
    public float[] dtheta;
    public float[] ddtheta;
    public float[] tau;
    public float4x4[] worldPoses;

    public float4x4 debugPose1;
    public float4x4 debugPose2;

    public float4x4[] relativePosesdb;

    public float3[] twistRotdb;
    public float3[] twistTransdb;

    public float3[] dTwistRotdb;
    public float3[] dTwistTransdb;

    public float3[] wrenchMomentsdb;
    public float3[] wrenchForcesdb;

    private float[] thetaMod;

    //limb1 definitions
    private static float4x4 limb1Home = new float4x4(1, 0, 0, 0,
                                              0, 1, 0, 0.5f,
                                              0, 0, 1, 1,
                                              0, 0, 0, 1);
    private static float3[] limb1ScrewRot = { new float3(1, 0, 0),
                                              new float3(0, 1, 0) };
    private static float3[] limb1ScrewPos = { new float3(0, 0, 1),
                                              new float3(0, 0, 1) };

    //limb2 definitions
    private static float4x4 limb2Home = new float4x4(1, 0, 0, 0,
                                                     0, 1, 0, 1.5f,
                                                     0, 0, 1, 1,
                                                     0, 0, 0, 1);
    private static float3[] limb2ScrewRot = { new float3(0, 0, 1) };
    private static float3[] limb2ScrewPos = { new float3(0, 1, 1) };

    //limb3 definitions
    private static float4x4 limb3Home = new float4x4(1, 0, 0, 0,
                                                     0, 1, 0, 2.5f,
                                                     0, 0, 1, 1,
                                                     0, 0, 0, 1);
    private static float3[] limb3ScrewRot = { new float3(1, 0, 0) };
    private static float3[] limb3ScrewPos = { new float3(0, 2, 1) };


    static ConnectedLimbSystem testLimbSys;


    private void Start()
    {
        Limb limb1 = new Limb("limb1", Object.Instantiate(limbPrefab), 12, new float3(1.0625f, .125f, 1.0625f), null, new string[] { "limb2" }, limb1Home, limb1ScrewRot, limb1ScrewPos);
        Limb limb2 = new Limb("limb2", Object.Instantiate(limbPrefab), 12, new float3(1.0625f, .125f, 1.0625f), "limb1", new string[] { "limb3" }, limb2Home, limb2ScrewRot, limb2ScrewPos);
        Limb limb3 = new Limb("limb3", Object.Instantiate(limbPrefab), 12, new float3(1.0625f, .125f, 1.0625f), "limb2", null, limb3Home, limb3ScrewRot, limb3ScrewPos);
        Limb[] testLimbs = { limb1, limb2, limb3 };
        testLimbSys = new ConnectedLimbSystem(testLimbs);

        thetaMod = new float[testLimbSys.totalInputs];
        theta = new float[testLimbSys.totalInputs];
        dtheta = new float[testLimbSys.totalInputs];
        ddtheta = new float[testLimbSys.totalInputs];
        tau = new float[testLimbSys.totalInputs];
    }

    void Update()
    {
        //float theta1 = SettingsManager.GetFloat("usertheta1") * math.PI;
        //float theta2 = SettingsManager.GetFloat("usertheta2") * math.PI;
        //float theta3 = SettingsManager.GetFloat("usertheta3") * math.PI;

        //float[] theta = { theta1, theta2, theta3 };

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
