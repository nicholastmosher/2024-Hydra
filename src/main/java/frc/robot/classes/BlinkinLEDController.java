// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import static frc.lib.util.BlinkinPattern.BLUE_ALLIANCE_PATTERNS;
import static frc.lib.util.BlinkinPattern.RED_ALLIANCE_PATTERNS;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.lib.config.LightConfig;
import frc.lib.util.BlinkinPattern;

/** Control REV Robotics Blinkin LED controller */
public class BlinkinLEDController {

    private static BlinkinLEDController m_controller = null;
    private static Spark m_blinkin;
    private static BlinkinPattern m_currentPattern;
    private static HashMap<Alliance, BlinkinPattern[]> m_allianceColors = new HashMap<Alliance, BlinkinPattern[]>();

    public BlinkinLEDController(int blinikinid) {
        m_blinkin = new Spark(blinikinid);

        m_allianceColors.put(Alliance.Red, RED_ALLIANCE_PATTERNS);
        m_allianceColors.put(Alliance.Blue, BLUE_ALLIANCE_PATTERNS);
    }
    public void setPattern(BlinkinPattern pattern) {
        m_currentPattern = pattern;
        m_blinkin.set(m_currentPattern.value);
    }
    public BlinkinPattern getCurrentPattern() {
        return m_currentPattern;
    }
    public void off() {
        setPattern(BlinkinPattern.BLACK);
    }
}
