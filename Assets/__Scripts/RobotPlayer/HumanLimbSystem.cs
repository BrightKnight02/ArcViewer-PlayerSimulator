
using Unity.VisualScripting;
using UnityEngine;
using Unity.Mathematics;

public class HumanLimbSystem
{
    // generate a ConnectedLimbSystem based on Subject1 of https://github.com/gbionics/human-gazebo
    // Creates a 2 link spine model using these measurements
    public ConnectedLimbSystem generate()
    {
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

        GameObject pelvisObject = ScaledCube(.25f, 0.1f, 0.2f);
        
        Limb pelvis = new Limb("pelvis", pelvisObject, 4.98f, new float3(0.01f, 0.03f, 0.03f),
            null, new string[] {"lowerBack" }, SE3Functions.PoseFromPos(new float3(0, 0.91f, 0)),
            new float3[] { new float3(0, 0, 0), new float3(0, 0, 0), new float3(0, 0, 0) },
            new float3[] { new float3(1, 0, 0), new float3(0, 1, 0), new float3(0, 0, 1) },
            new float[] { -.1f, 0.45f, -0.1f }, new float[] { .1f, 0.93f, 0.1f });

        GameObject lowerBackObject = ScaledCube(0.2f, .5f, .1f);

        Limb lowerBack = new Limb("lowerBack", lowerBackObject, 6.344f * 2, new float3(0.019f, 2 * .024f + 2 * 6.334f * 0.25f * 0.25f, 0.037f), // combining L3 and L5 from urdf
            "pelvis", new string[] { "chest" }, SE3Functions.PoseFromPos(new float3(0, 1.06f, 0)),
            new float3[] { new float3(0, 0, 1), new float3(1, 0, 0) },
            new float3[] { new float3(0, 1.01f, 0), new float3(0, 1.01f, 0) },
            new float[] { -1.3f, -0.61f }, new float[] { 0.52f, 0.61f });

        Limb[] limbs = { pelvis, lowerBack };
        return new ConnectedLimbSystem(limbs);
    }

    private GameObject ScaledCube(float x, float y, float z)
    {
        GameObject output = GameObject.CreatePrimitive(PrimitiveType.Cube);
        output.transform.localScale = new Vector3(x, y, z);
        return output;
    }
}
