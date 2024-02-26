package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.*;

public final class Constants {

    public static IntakeConfig intakeConfig = new IntakeConfig(50, -0.8);

    public static ArmConfig armConfig = new ArmConfig(51, 52, new PidConfig(1.0, 0.0, 0.0), Rotation2d.fromDegrees(160.84), Rotation2d.fromDegrees(160.84), Rotation2d.fromDegrees(232.44));

    public static ShooterConfig shooterConfig = new ShooterConfig(53, 54, 55, new PidConfig(1.0, 0.0, 0.0), -0.8, 0.6);

    public static LightConfig lightConfig = new LightConfig(56, 57, 1.5, 0);

    public static SwerveModuleConfig mod0frontleftConfig = new SwerveModuleConfig(11, 12, 13, Rotation2d.fromDegrees(-0.1+45+-90));
    public static SwerveModuleConfig mod1frontrightConfig = new SwerveModuleConfig(21, 22, 23, Rotation2d.fromDegrees(-0.2+72-90));
    public static SwerveModuleConfig mod2backleftConfig = new SwerveModuleConfig(31, 32, 33, Rotation2d.fromDegrees(-0.44-10-90));
    public static SwerveModuleConfig mod3backrightConfig = new SwerveModuleConfig(41, 42, 43, Rotation2d.fromDegrees(-0.10+45-
            90));
}
