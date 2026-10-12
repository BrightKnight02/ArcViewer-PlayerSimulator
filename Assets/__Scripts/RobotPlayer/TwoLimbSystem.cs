using UnityEngine;
using Unity.Mathematics;

public class TwoLimbSystem : MonoBehaviour
{

    public GameObject limb1game;
    public GameObject limb2game;

    private static float4x4 limb1Home = new float4x4(1, 0, 0, 0,
                                              0, 1, 0, 0.5f,
                                              0, 0, 1, 1,
                                              0, 0, 0, 1);
    private static float3[] limb1ScrewRot = { new float3(1, 0, 0), new float3(0, 1, 0) };

    //private static Limb limb1;


    private static float4x4 limb2Home = new float4x4(1, 0, 0, 0,
                                              0, 1, 0, 1.5f,
                                              0, 0, 1, 1,
                                              0, 0, 0, 1);
    private static float3[] limb2ScrewRot = { new float3(0, 0, 1) };

    //private static Limb limb2;

    static ConnectedLimbSystem twoLimbSys;

    private void Start()
    {
        Limb limb1 = new Limb("limb1", limb1game, 1, float3.zero, 2, null, new string[] { "limb2" }, limb1Home, limb1ScrewRot);
        Limb limb2 = new Limb("limb2", limb2game, 1, float3.zero, 1, "limb1", null, limb2Home, limb2ScrewRot);
        Limb[] twoLimbLimbs = { limb1, limb2 };
        twoLimbSys = new ConnectedLimbSystem(twoLimbLimbs);
    }

    void Update()
    {
        float theta1 = SettingsManager.GetFloat("usertheta1") * math.PI;
        float theta2 = SettingsManager.GetFloat("usertheta2") * math.PI;
        float theta3 = SettingsManager.GetFloat("usertheta3") * math.PI;

        float[] theta = { theta1, theta2, theta3 };

        //Debug.Log("Theta: " + theta.ToString());
        twoLimbSys.OrientGameObjects(theta);
    }

}
