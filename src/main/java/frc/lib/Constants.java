package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.*;
import frc.lib.krakentalon.krakenTalonConstants;

public final class Constants {

    public static IntakeConfig intakeConfig = new IntakeConfig(32, -0.8);

    public static ArmConfig armConfig = new ArmConfig(33, 34, new PidConfig(1.0, 0.0, 0.0));

    public static ShooterConfig shooterConfig = new ShooterConfig(50, 51, 37, new PidConfig(1.0, 0.0, 0.0), -0.8, 0.6);

    public static SwerveModuleConfig mod0frontleftConfig = new SwerveModuleConfig(1, 2, 3, Rotation2d.fromDegrees(142.39));
    public static SwerveModuleConfig mod1frontrightConfig = new SwerveModuleConfig(11, 12, 13, Rotation2d.fromDegrees(17.92));
    public static SwerveModuleConfig mod2backleftConfig = new SwerveModuleConfig(21, 22, 23, Rotation2d.fromDegrees(106.43));
    public static SwerveModuleConfig mod3backrightConfig = new SwerveModuleConfig(31, 32, 33, Rotation2d.fromDegrees(136.05));
}
