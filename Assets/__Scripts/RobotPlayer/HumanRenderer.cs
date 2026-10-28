using System;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class HumanRenderer : MonoBehaviour
{
    public static ConnectedLimbSystem robotSys;
    public GameObject saberPrefab;
    public float[] theta;

    private float prevTime = 0;

    void Start()
    {
        robotSys = HumanLimbSystem.generate();
        
        Limb leftHand = robotSys.FindLimbByName("leftHand");
        GameObject leftSaber = GameObject.Instantiate(saberPrefab);
        leftSaber.transform.rotation = Quaternion.Euler(0, -90, 0);
        leftSaber.transform.parent = leftHand.visObject.transform;
        
        Limb rightHand = robotSys.FindLimbByName("rightHand");
        GameObject rightSaber = GameObject.Instantiate(saberPrefab);
        rightSaber.transform.rotation = Quaternion.Euler(0, 90, 0);
        rightSaber.transform.parent = rightHand.visObject.transform;
        
        theta = new float[robotSys.totalInputs];
    }

    // Update is called once per frame
    void Update()
    {
        float time = Time.fixedTime;
        const float dt = 0.1f;
        float theta1 = SettingsManager.GetFloat("usertheta1") * Mathf.PI;
        float theta2 = SettingsManager.GetFloat("usertheta2") * Mathf.PI;
        float theta3 = SettingsManager.GetFloat("usertheta3") * Mathf.PI;
        float[] userTheta = { theta1, theta2, theta3 };

        if(time - prevTime > dt)
        {
            robotSys.OrientGameObjects(theta);
            prevTime = time;
            for(int i = 3; i < theta.Length; i++)
            {
                theta[i] = userTheta[3 * (i-3) / theta.Length];
            }
            //Debug.Log($"[{string.Join(",", theta)}]");
        }
    }
}
