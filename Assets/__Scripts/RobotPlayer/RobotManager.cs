using System;
using UnityEngine;

public class RobotManager : MonoBehaviour
{
    public static ConnectedLimbSystem robotSystem;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        robotSystem = HumanRenderer.robotSys;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        bool visualizeRobot = SettingsManager.GetBool("robotvis");
        robotSystem.sysObject.SetActive(visualizeRobot);
    }
}
