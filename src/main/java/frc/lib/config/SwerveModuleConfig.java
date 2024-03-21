package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
    public final int driveMotorID;
    public final boolean driveInvert;
    public final int angleMotorID;
    public final boolean angleInvert;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param driveInvert
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConfig(int driveMotorID, boolean driveInvert, int angleMotorID, boolean angleInvert, int canCoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.driveInvert = driveInvert;
        this.angleMotorID = angleMotorID;
        this.angleInvert = angleInvert;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
