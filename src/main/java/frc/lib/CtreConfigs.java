package frc.lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.krakentalon.krakenTalonConstants;

public final class CtreConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CtreConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = krakenTalonConstants.Swerve.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
//        swerveCANcoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//        swerveCANcoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        //swerveAngleFXConfig.MotorOutput.Inverted = krakenTalonConstants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = krakenTalonConstants.Swerve.angleNeutralMode;

        swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = krakenTalonConstants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = krakenTalonConstants.Swerve.angleCurrentLimit.currentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = krakenTalonConstants.Swerve.angleCurrentLimit.currentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = krakenTalonConstants.Swerve.angleCurrentLimit.currentThresholdTime;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = krakenTalonConstants.Swerve.angleCurrentLimit.enableCurrentLimit;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = krakenTalonConstants.Swerve.anglePid.kP;
        swerveAngleFXConfig.Slot0.kI = krakenTalonConstants.Swerve.anglePid.kI;
        swerveAngleFXConfig.Slot0.kD = krakenTalonConstants.Swerve.anglePid.kD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        //swerveDriveFXConfig.MotorOutput.Inverted = krakenTalonConstants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = krakenTalonConstants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = krakenTalonConstants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = krakenTalonConstants.Swerve.driveCurrentLimit.currentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = krakenTalonConstants.Swerve.driveCurrentLimit.currentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = krakenTalonConstants.Swerve.driveCurrentLimit.currentThresholdTime;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = krakenTalonConstants.Swerve.driveCurrentLimit.enableCurrentLimit;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = krakenTalonConstants.Swerve.drivePid.kP;
        swerveDriveFXConfig.Slot0.kI = krakenTalonConstants.Swerve.drivePid.kI;
        swerveDriveFXConfig.Slot0.kD = krakenTalonConstants.Swerve.drivePid.kD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = krakenTalonConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = krakenTalonConstants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = krakenTalonConstants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = krakenTalonConstants.Swerve.closedLoopRamp;
    }
}