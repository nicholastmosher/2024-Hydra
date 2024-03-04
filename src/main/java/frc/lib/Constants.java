package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.config.*;

public final class Constants {

    public static IntakeConfig intakeConfig = new IntakeConfig(50, -0.5, 2047);

    public static ArmConfig armConfig = new ArmConfig(51, 52, new PidConfig(1.0, 0.0, 0.0), Rotation2d.fromDegrees(2), Rotation2d.fromDegrees(2), Rotation2d.fromDegrees(232.44-162.84), Rotation2d.fromRotations(1).getDegrees(), Rotation2d.fromDegrees(162.84), new DigitalInput(2));

    public static ShooterConfig shooterConfig = new ShooterConfig(53, 54, 55, new PidConfig(1.0, 0.0, 0.0), -0.2, -0.4, 0.5, 10000);

    public static LightConfig lightConfig = new LightConfig(56, 57, 1.5, 0);

    public static ClimberConfig climberConfig = new ClimberConfig(59, 58, 1.0, -1.0, new DigitalInput(0), new DigitalInput(1));

    public static SwerveModuleConfig mod0frontleftConfig = new SwerveModuleConfig(11, 12, 13, Rotation2d.fromDegrees(-0.1+45+-90));
    public static SwerveModuleConfig mod1frontrightConfig = new SwerveModuleConfig(21, 22, 23, Rotation2d.fromDegrees(-0.2+72-90));
    public static SwerveModuleConfig mod2backleftConfig = new SwerveModuleConfig(31, 32, 33, Rotation2d.fromDegrees(-0.44-10-90-180));
    public static SwerveModuleConfig mod3backrightConfig = new SwerveModuleConfig(41, 42, 43, Rotation2d.fromDegrees(-0.10+45-
            90));
}
