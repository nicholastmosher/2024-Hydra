package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ArmConfig;
import frc.lib.config.DashboardConfig;

import java.util.function.BooleanSupplier;

public class Arm extends SubsystemBase {
    private final CANSparkMax armLeftMotor;
    private final CANSparkMax armRightMotor;

    private final SparkPIDController armPID;

    private final AbsoluteEncoder armEncoder;

    private final DigitalInput armLimitSwitch;

    public final ArmConfig config;


    private PIDController armPIDController;

    public Arm(ArmConfig config) {
        this.config = config;



        armLeftMotor = new CANSparkMax(this.config.leftMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armPID = armLeftMotor.getPIDController();
        armEncoder = armLeftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        armEncoder.setPositionConversionFactor(config.positionScalingFactor);
        armEncoder.setZeroOffset(163.32+180);
        armPID.setFeedbackDevice(armEncoder);

        armRightMotor = new CANSparkMax(this.config.rightMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armRightMotor.follow(armLeftMotor, true);

        armLimitSwitch = config.armLimitSwitch;

        armPIDController = new PIDController(1, 0, 0);
    }

    public void init() {

    }

    public void setAngle(double angle) {
//        if (!isFullLower() ||  armEncoder.getVelocity() >0) {
//            Rotation2d motorAngle =Rotation2d.fromRotations(armLeftMotor.getEncoder().getPosition());
//            System.out.println(getPosition().getDegrees());
//            System.out.println(motorAngle.getDegrees());
//            armLeftMotor.getEncoder().setPosition(getPosition().getRotations());
//            armPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
//        } else {
//            armLeftMotor.stopMotor();
//        }

//        if (in < 0) {
//            if (isFullLower()) {
//                armLeftMotor.stopMotor();
//                return;
//            }
//
//            armPID.setReference(angle);
//            return;
//        }
//        if (in > 0) {
//            armLeftMotor.set(in);
//            return;
//        }
//        armLeftMotor.set(0);
//        double input = armPIDController.calculate(armEncoder.getPosition(), config.ampAngle);
//        SmartDashboard.putNumber("PID Loop", -input);
//
//        armLeftMotor.set(-input);

        armPID.setReference(angle, CANSparkMax.ControlType.kPosition);

//        if (input < 0) {
//            if (isFullLower()) {
//                armLeftMotor.stopMotor();
//                return;
//            }
//
//            armLeftMotor.set(input);
//            return;
//        }
//        if (input > 0) {
//            armLeftMotor.set(input);
//            return;
//        }
//        armLeftMotor.set(0);


    }

    public void moveArm(double input) {

        double inup = input *((config.ampAngle-armEncoder.getPosition()) * 4.5);
        double indown = input *((armEncoder.getPosition() - 0.45) * 4.5);

        if (input < 0) {
            if (isFullLower()) {
                armLeftMotor.stopMotor();
                return;
            }

            armLeftMotor.set(indown);
            return;
        }
        if (input > 0) {
            if (armEncoder.getPosition() > (config.ampAngle - 0.05) && armEncoder.getPosition() < (config.ampAngle + 0.05)) {
                armLeftMotor.stopMotor();
                return;
            }

            armLeftMotor.set(inup);
            return;
        }
        armLeftMotor.set(0);

    }

    public boolean endCondition(double angle) {
        return armEncoder.getPosition() > (config.ampAngle - 0.05) && armEncoder.getPosition() < (config.ampAngle + 0.05);
    }

    public void stopMotor() {
        armLeftMotor.stopMotor();
        armRightMotor.stopMotor();
        System.out.println("Stop All Arm Movement");
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(armEncoder.getPosition());
    }

    private boolean isFullLower() {
        if (!armLimitSwitch.get()) {
            //armEncoder.setZeroOffset(armEncoder.getPosition());
            return true;
        }

        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("fulllower", isFullLower());
        SmartDashboard.putNumber("armAngle", armEncoder.getPosition());
        SmartDashboard.putBoolean("ampPosition", endCondition(1));

    }
}
