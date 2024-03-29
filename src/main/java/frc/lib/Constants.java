package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.config.*;

import java.awt.*;

public final class Constants {

    public static IntakeConfig intakeConfig = new IntakeConfig(50, -0.8);
    public static ArmConfig armConfig = new ArmConfig(51, 52, new PidConfig(1.0, 0.0, 0.0), 0.55, 0.45, 0.663, 1, 163.32+180, new DigitalInput(2));
    public static ShooterConfig shooterConfig = new ShooterConfig(53, 54, -0.2, 10000);
    public static IndexerConfig indexerConfig = new IndexerConfig(55, -0.5, 0.15);
    public static LightConfig lightConfig = new LightConfig(0, 1, 1.5, 0);
    public static ColorSensorConfig colorSensorConfig = new ColorSensorConfig(2047);
    public static ClimberConfig climberConfig = new ClimberConfig(59, 58, 1.0, -1.0, new DigitalInput(1), new DigitalInput(0));
    public static VisionConfig visionConfig = new VisionConfig("limelight-intake", "limelight-shoot");

    public static SwerveModuleConfig mod0frontleftConfig = new SwerveModuleConfig(11, false, 12, false, 13, Rotation2d.fromRotations(0.95));
    public static SwerveModuleConfig mod1frontrightConfig = new SwerveModuleConfig(21, false, 22, false, 23, Rotation2d.fromRotations(-0.08));
    public static SwerveModuleConfig mod2backleftConfig = new SwerveModuleConfig(31, false, 32, false,33, Rotation2d.fromRotations(0.12));//
    public static SwerveModuleConfig mod3backrightConfig = new SwerveModuleConfig(41, false, 42, false, 43, Rotation2d.fromRotations(0.19));

    public final class Debug {
        //set to false to allow compiler to identify and eliminate
        //unreachable code
        public static final boolean VisionON = false;
        }

    public static class VisionParameters {
        public static final int k_lightOff = 1;
        public static final int k_lightOn = 3;
        public static final int k_maxPipeline = 2;
        public static final int k_aprilTagPipeline = 0;
        public static final int k_conePipeline = 1;
        public static final int k_cubePipeline = 2;
        public static final int k_retrotapePipeline = 3;
        public static final double k_xTargetBounds = 2.0;  //+- degrees
        public static final double k_offset = 8.0;
        }
    public static enum AutonomousOptions {
        TWO_NOTE_CENTER, SHOOT_NOTE,SHOOT_NOTE_MOVEBACK, RIGHTSPEAKERSIDESHOOTANDMOVEBACK, THREE_NOTES_RIGHT, THREE_NOTES_LEFT;
    }
}
