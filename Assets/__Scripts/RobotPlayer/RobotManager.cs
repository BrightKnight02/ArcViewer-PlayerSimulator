using System;
using UnityEngine;

public class RobotManager : MonoBehaviour
{
    public static ConnectedLimbSystem robotSystem;
    public MapElementList<Note> notes;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        robotSystem = HumanRenderer.robotSys;
        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        notes = ObjectManager.Instance.noteManager.Objects;
        bool visualizeRobot = SettingsManager.GetBool("robotvis");
        robotSystem.sysObject.SetActive(visualizeRobot);
        if(!OptimizationManager.optRunning && BeatmapManager.CurrentDifficulty.difficultyRank == DifficultyRank.Easy)
        {
            Debug.Log("Running Optimization");
            OptimizationManager.FindOptimalTrajectory();
        }
    }
}
