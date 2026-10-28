
using Unity.VisualScripting;
using UnityEngine;
using Unity.Mathematics;

public class HumanLimbSystem
{
    // generate a ConnectedLimbSystem based on Subject1 of https://github.com/gbionics/human-gazebo
    // Creates a 2 link spine model using these measurements
    static private GameObject renderPrefab = Resources.Load("RobotCube") as GameObject;

    public static ConnectedLimbSystem generate()
    {

        //--------------------------------------------------------
        //torso limbs
        float3 pelvisPos = new float3(0, 0.81f, 0);
        GameObject pelvisObject = ScaledCube(.25f, 0.1f, 0.2f);

        Limb pelvis = new Limb("pelvis", pelvisObject, 4.98f, new float3(0.01f, 0.03f, 0.03f),
            null, new string[] { "lowerBack" }, SE3Functions.PoseFromPos(pelvisPos),
            new float3[] { new float3(0, 0, 0), new float3(0, 0, 0), new float3(0, 0, 0) },
            new float3[] { new float3(1, 0, 0), new float3(0, 1, 0), new float3(0, 0, 1) },
            new float[] { -0.1f, 0.45f, -0.1f }, new float[] { 0.1f, 0.93f, 0.1f });

        float3 lbRot = pelvisPos + new float3(0, 0.1f, 0);
        float3 lbCOM = lbRot + new float3(0, 0.05f + (0.1f + 0.043f) / 2f, 0);
        GameObject lowerBackObject = ScaledCube(0.2f, .5f, .1f);

        Limb lowerBack = new Limb("lowerBack", lowerBackObject, 6.344f * 2, new float3(0.019f, 2 * .024f + 2 * 6.334f * 0.25f * 0.25f, 0.037f), // combining L3 and L5 from urdf
            "pelvis", new string[] { "ribs" }, SE3Functions.PoseFromPos(lbCOM),
            new float3[] { new float3(0, 0, 1), new float3(1, 0, 0) },
            new float3[] { lbRot, lbRot },
            new float[] { -1.3f, -0.61f }, new float[] { 0.52f, 0.61f });

        float3 ribsRot = lbRot + new float3(0, 0.05f + 0.1f + 0.043f + 0.08f, 0);
        float3 ribsCOM = ribsRot + new float3(0, 0.04f + (2.488f * (0.06f + 0.08f)) / (2.488f + 6.344f), 0);
        GameObject ribsObject = ScaledCube(.2f, 0.4f, .2f);

        Limb ribs = new Limb("ribs", ribsObject, 6.344f + 2.488f, new float3(0.021f, 0.028f + 6.344f * .04f * .04f + 2.488f * 0.1f * 0.1f, 0.022f), //COMs .14 apart 
            "lowerBack", new string[] { "neck", "leftShoulder", "rightShoulder" }, SE3Functions.PoseFromPos(ribsCOM),
            new float3[] { new float3(0, 0, 1), new float3(1, 0, 0), new float3(0, 1, 0) },
            new float3[] { ribsRot, ribsRot, ribsRot },
            new float[] { -1, -2f, -0.6f }, new float[] { 1, 0.75f, 0.6f });

        //--------------------------------------------------------
        //head system
        float3 neckRot = ribsRot + new float3(0, 0.04f + 0.06f + 0.08f + 0.12f, 0);
        float3 headNeckCOM = neckRot + new float3(0, 2.24f * (0.09f + 0.09f) / (0.75f + 2.24f), 0);
        GameObject neckObject = ScaledCube(0.05f, .1f, 0.05f);
        GameObject headObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        headObject.transform.parent = neckObject.transform;
        headObject.transform.localScale = new Vector3(4f, 2f, 4f);
        headObject.transform.position = new Vector3(0, 0.05f, 0.05f);
        headObject.layer = 12;

        Limb neck = new Limb("neck", neckObject, 0.75f + 2.24f, new float3(0.0076f, 0.0074f + 2.24f * 0.4f * 0.4f + 0.75f * .13f * .13f, 0.0076f),
            "ribs", null, SE3Functions.PoseFromPos(headNeckCOM),
            new float3[] { new float3(0, 0, 1) }, new float3[] { neckRot },
            new float[] { -0.61f }, new float[] { 0.61f });

        //--------------------------------------------------------
        //Right arm
        float3 rShoulderRot = ribsRot + new float3(0.026f, 0.04f + 0.06f + 0.08f + 0.067f, 0); //actually an internal joint
        float3 rShoulderCOM = rShoulderRot + new float3(0.065f, 0, 0);
        GameObject shoulderObject = ScaledCube(0.1f, 0.1f, 0.1f);

        Limb rightShoulder = new Limb("rightShoulder", shoulderObject, 1.928f, new float3(0.0005f, 0.003f, 0.003f),
            "ribs", new string[] { "rightUpperArm" }, SE3Functions.PoseFromPos(rShoulderCOM),
            new float3[] { new float3(0, 0, 1) },
            new float3[] { rShoulderRot },
            new float[] { -0.087f }, new float[] { 0.785f });

        float3 rUArmRot = rShoulderCOM + new float3(0.13f, 0, 0);
        float3 rUArmCOM = rUArmRot + new float3(0.13f, 0, 0);
        GameObject upperArmObject = ScaledCube(0.26f, 0.1f, 0.1f);

        Limb rightUpperArm = new Limb("rightUpperArm", upperArmObject, 1.866f, new float3(0.0008f, 0.011f, 0.011f),
            "rightShoulder", new string[] {"rightLowerArm"}, SE3Functions.PoseFromPos(rUArmCOM),
            new float3[] { new float3(0, 0, 1), new float3(1, 0, 0), new float3(0, 1, 0) },
            new float3[] { rUArmRot, rUArmRot, rUArmRot },
            new float[] { -2.35f, -1.57f, -0.78f }, new float[] { 1.57f, 1.57f, 3.14f });

        float3 rLArmRot = rUArmCOM + new float3(0.27f, 0, 0);
        float3 rLArmCOM = rLArmRot + new float3(0.11f, 0, 0);
        GameObject lowerArmObject = ScaledCube(0.22f, 0.05f, 0.05f);

        Limb rightLowerArm = new Limb("rightLowerArm", lowerArmObject, 1.244f, new float3(0.0002f, 0.005f, 0.005f),
            "rightUpperArm", new string[] { "rightHand" }, SE3Functions.PoseFromPos(rLArmCOM),
            new float3[] { new float3(1, 0, 0), new float3(0, 1, 0) },
            new float3[] { rLArmRot, rLArmRot },
            new float[] { -0.78f, -1.57f }, new float[] { 3.141f, 1.48f });

        float3 rHandRot = rLArmCOM + new float3(0.22f, 0, 7.7e-5f);
        float3 rHandCOM = rHandRot + new float3(0.083f, 0, 0);
        GameObject handObject = ScaledCube(0.11f, 0.07f, 0.11f);

        Limb rightHand = new Limb("rightHand", handObject, 0.37f, new float3(0.0004f, 0.001f, 0.0009f),
            "rightLowerArm", null, SE3Functions.PoseFromPos(rHandCOM),
            new float3[] { new float3(0, 0, 1), new float3(0, 1, 0) },
            new float3[] { rHandRot, rHandRot },
            new float[] { -0.87f, -0.34f }, new float[] { 1.04f, 0.52f });

        //--------------------------------------------------------
        //Left arm
        float3 lShoulderRot = rShoulderRot;
        float3 lShoulderCOM = rShoulderCOM;
        lShoulderRot[0] *= -1;
        lShoulderCOM[0] *= -1;

        Limb leftShoulder = new Limb("leftShoulder", GameObject.Instantiate(shoulderObject), 1.928f, new float3(0.0005f, 0.003f, 0.003f),
            "ribs", new string[] { "leftUpperArm" }, SE3Functions.PoseFromPos(lShoulderCOM),
            new float3[] { new float3(0, 0, 1) },
            new float3[] { lShoulderRot },
            new float[] { -0.087f }, new float[] { 0.785f });

        float3 lUArmRot = rUArmRot;
        float3 lUArmCOM = rUArmCOM;
        lUArmRot[0] *= -1;
        lUArmCOM[0] *= -1;

        Limb leftUpperArm = new Limb("leftUpperArm", GameObject.Instantiate(upperArmObject), 1.866f, new float3(0.0008f, 0.011f, 0.011f),
            "leftShoulder", new string[] { "leftLowerArm" }, SE3Functions.PoseFromPos(lUArmCOM),
            new float3[] { new float3(0, 0, 1), new float3(1, 0, 0), new float3(0, 1, 0) },
            new float3[] { lUArmRot, lUArmRot, lUArmRot },
            new float[] { -2.35f, -1.57f, -0.78f }, new float[] { 1.57f, 1.57f, 3.14f });

        float3 lLArmRot = rLArmRot;
        float3 lLArmCOM = rLArmCOM;
        lLArmRot[0] *= -1;
        lLArmCOM[0] *= -1;

        Limb leftLowerArm = new Limb("leftLowerArm", GameObject.Instantiate(lowerArmObject), 1.244f, new float3(0.0002f, 0.005f, 0.005f),
            "leftUpperArm", new string[] { "leftHand" }, SE3Functions.PoseFromPos(lLArmCOM),
            new float3[] { new float3(1, 0, 0), new float3(0, 1, 0) },
            new float3[] { lLArmRot, lLArmRot },
            new float[] { -0.78f, -1.57f }, new float[] { 3.141f, 1.48f });

        float3 lHandRot = rHandRot;
        float3 lHandCOM = rHandCOM;
        lHandRot[0] *= -1;
        lHandCOM[0] *= -1;

        Limb leftHand = new Limb("leftHand", GameObject.Instantiate(handObject), 0.37f, new float3(0.0004f, 0.001f, 0.0009f),
            "leftLowerArm", null, SE3Functions.PoseFromPos(lHandCOM),
            new float3[] { new float3(0, 0, 1), new float3(0, 1, 0) },
            new float3[] { lHandRot, lHandRot },
            new float[] { -0.87f, -0.34f }, new float[] { 1.04f, 0.52f });


        Limb[] limbs = { pelvis, lowerBack, ribs, neck, rightShoulder, rightUpperArm, rightLowerArm, rightHand, leftShoulder, leftUpperArm, leftLowerArm, leftHand };
        return new ConnectedLimbSystem(limbs, "Robot System");
    }

    private static GameObject ScaledCube(float x, float y, float z)
    {

        GameObject output = GameObject.Instantiate(renderPrefab);
        Debug.Log(output);
        output.transform.localScale = new Vector3(x, y, z);
        return output;
    }
}
// Complex lower body probably not necessary but here if needed
/**
GameObject shinObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
shinObject.transform.localScale = new Vector3(.1f, .4f, .1f);

float FOOTX = 0.177f;
// rshin has two additional degrees of freedom for positioning the foot in x and rotation about y
Limb rshin = new Limb("rshin", GameObject.Instantiate(shinObject), 2.27f, new float3(0, 0.28f, 0),
    null, new string[] { "rthigh" }, SE3Functions.PoseFromPos(new float3(0.08f, 0.25f, 0)),
    new float3[] { new float3(0, 0, 0), new float3(0, 1, 0), new float3(1, 0, 0), new float3(0.838488745177307f, -0.291211660001699f, -0.460578324815704f) }, //ankle rotation axes from https://pmc.ncbi.nlm.nih.gov/articles/PMC4994968/ and pixel measurements
    new float3[] { new float3(1, 0, 0), new float3(0.08f, 0, 0), new float3(0.08f, 0.08f, 0), new float3(0.08f, 0.08f, 0) },
    new float[] { FOOTX, -math.PI, -math.PI / 2f, -math.PI / 6f }, new float[] { FOOTX, math.PI, math.PI / 10f, math.PI / 20f });
Limb lshin = new Limb("lshin", shinObject, 2.27f, new float3(0, 0.28f, 0),
    "lthigh", null, SE3Functions.PoseFromPos(new float3(-0.08f, 0.25f, 0)),
    new float3[] { new float3(1, 0, 0) }, new float3[] { new float3(-0.08f, 0.48f, 0) },
    new float[] { -math.PI }, new float[] { 0 });

GameObject thighObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
thighObject.transform.localScale = new Vector3(.2f, 0.4f, 0.2f);

Limb rthigh = new Limb("rthigh", thighObject, 7.78f, new float3(0.13681f, 0.01241f, 0.13681f),
    "rshin", new string[] {"pelvis"}, SE3Functions.PoseFromPos(new float3(0.08f, 0.69f, 0)),
    new float3[] { new float3(1, 0, 0) }, new float3[] { new float3(0.08f, 0.48f, 0) },
    new float[] { -math.PI }, new float[] { 0 });
Limb lthigh = new Limb("lthigh", thighObject, 7.78f, new float3(0.13681f, 0.01241f, 0.13681f),
    "pelvis", new string[] { "lshin" }, SE3Functions.PoseFromPos(new float3(-0.08f, 0.69f, 0)),
    new float3[] { new float3(1, 0, 0), new float3(0, 1, 0), new float3(0, 0, 1) }, 
    new float3[] { new float3(-0.08f, 0.91f, 0), new float3(-0.08f, 0.91f, 0), new float3(-0.08f, 0.91f, 0) },
    new float[] { -math.PI, -math.PI/4, -math.PI/10f }, new float[] { math.PI/2, math.PI/2, math.PI/2 });


GameObject pelvisObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
pelvisObject.transform.localScale = new Vector3(.25f, 0.1f, 0.2f);
Limb pelvis = new Limb("pelvis", pelvisObject, 4.98f, new float3(0.03f, 0.03f, 0.01f),
    "rthigh", new string[] { "lthigh", "lowerBack" }, SE3Functions.PoseFromPos(new float3(0, 0.91f, 0)),
    new float3[] { new float3(1, 0, 0), new float3(0, 1, 0), new float3(0, 0, 1) },
    new float3[] { new float3(0.08f, 0.91f, 0), new float3(0.08f, 0.91f, 0), new float3(0.08f, 0.91f, 0) },
    new float[] { -math.PI, -math.PI / 2, -math.PI / 2 }, new float[] { math.PI / 2, math.PI / 4, math.PI / 10f });
**/
