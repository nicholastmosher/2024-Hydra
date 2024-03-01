package frc.lib.krakentalon;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.ModuleConstants;
import frc.lib.config.CurrentLimitConfig;
import frc.lib.config.DriveTrainConfig;
import frc.lib.config.PidConfig;

public class krakenTalonConstants {
    public static double stickDeadband = 0.1;

    public static final class Swerve {
        public final static int pigeonID = 4;

        public final static ModuleConstants drivemodule =  //TODO: This must be tuned to specific robot
                ModuleConstants.SDS.MK4i.KrakenFalcon(ModuleConstants.SDS.MK4i.driveRatio.L1and16t);

        public final static ModuleConstants anglemodule =  //TODO: This must be tuned to specific robot
                ModuleConstants.SDS.MK4i.KrakenFalcon(ModuleConstants.SDS.MK4i.driveRatio.L1and16t);

        public final static DriveTrainConfig driveTrainConfig = new DriveTrainConfig(Units.inchesToMeters(20.75), Units.inchesToMeters(20.75), drivemodule.wheelCircumference);

        public final static double swerveRadius = 11.6875;

        /* Module Gear Ratios */
        public final static double driveGearRatio = drivemodule.driveGearRatio;
        public final static double angleGearRatio = anglemodule.angleGearRatio;

        /* Motor Inverts */
        public final static boolean angleMotorInvert = anglemodule.angleMotorInvert;
        public final static boolean driveMotorInvert = drivemodule.driveMotorInvert;

        /* Angle Encoder Invert */
        public final static SensorDirectionValue cancoderInvert = anglemodule.cancoderInvert;

        /* Swerve Current Limiting */
        public final static CurrentLimitConfig angleCurrentLimit = new CurrentLimitConfig(10, 40, 0.1, true);
        public final static CurrentLimitConfig driveCurrentLimit = new CurrentLimitConfig(30, 60, 0.1, true);

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public final static double openLoopRamp = 0.25;
        public final static double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public final static PidConfig anglePid = new PidConfig(anglemodule.angleKP, anglemodule.angleKI, anglemodule.angleKD);

        /* Drive Motor PID Values */
        public final static PidConfig drivePid = new PidConfig(0.12, 0.0, 0.0);

        /* Drive Motor Characterization Values From SYSID */
        public final static double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public final static double driveKV = 1.51;
        public final static double driveKA = 0.27;

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        public final static double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         */
        public final static double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public final static NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public final static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static double kPXController = 1;
        public static double kPYController = 1;
        public static double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}