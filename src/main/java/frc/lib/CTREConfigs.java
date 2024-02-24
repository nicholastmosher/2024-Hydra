package frc.lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.Constants;
import frc.lib.krakentalon.krakenTalonConstants;
//import com.ctre.phoenix6.sensors.CANCoderConfiguration;
//import com.ctre.phoenix6.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public static CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = krakenTalonConstants.Swerve.cancoderInvert;
//        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
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
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = krakenTalonConstants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = krakenTalonConstants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = krakenTalonConstants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = krakenTalonConstants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = krakenTalonConstants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = krakenTalonConstants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = krakenTalonConstants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        //swerveDriveFXConfig.MotorOutput.Inverted = krakenTalonConstants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = krakenTalonConstants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = krakenTalonConstants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = krakenTalonConstants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = krakenTalonConstants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = krakenTalonConstants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = krakenTalonConstants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = krakenTalonConstants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = krakenTalonConstants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = krakenTalonConstants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = krakenTalonConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = krakenTalonConstants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = krakenTalonConstants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = krakenTalonConstants.Swerve.closedLoopRamp;
    }
}