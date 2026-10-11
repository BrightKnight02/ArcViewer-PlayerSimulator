using System;
using UnityEngine;

public class RobotManager : MonoBehaviour
{
    public GameObject testSphere;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        bool visualizeRobot = SettingsManager.GetBool("robotvis");
        testSphere.SetActive(visualizeRobot);
    }
}
