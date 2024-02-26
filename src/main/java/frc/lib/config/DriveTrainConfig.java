package frc.lib.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveTrainConfig {
    public final double trackWidth;
    public final double wheelBase;
    public final double wheelCircumference;
    public final SwerveDriveKinematics kinematics;

    public DriveTrainConfig(double trackWidth, double wheelBase, double wheelCircumference) {
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.wheelCircumference = wheelCircumference;
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );
    }
}
