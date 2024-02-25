package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ArmConfig;

public class Arm extends SubsystemBase {
    private final CANSparkMax armRightMotor;
    private final CANSparkMax armLeftMotor;

    private final SparkPIDController armPID;

    private final AbsoluteEncoder armEncoder;

    public final ArmConfig config;

    public Arm(ArmConfig config) {
        this.config = config;

        armRightMotor = new CANSparkMax(this.config.rightMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armPID = armRightMotor.getPIDController();


        armEncoder = armRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        configArmMotor();

        armLeftMotor = new CANSparkMax(this.config.leftMotorId, CANSparkLowLevel.MotorType.kBrushless);
        configFollowerMotor();



    }

    public void setAngle(Rotation2d angle) {
        Rotation2d motorAngle =Rotation2d.fromRotations(armRightMotor.getEncoder().getPosition());
        System.out.println(getPosition().getDegrees());
        System.out.println(motorAngle.getDegrees());
        armRightMotor.getEncoder().setPosition(getPosition().getRotations());
        armPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        //System.out.println(armPID.get);

    }

    public void moveArm(double input) {
        armRightMotor.set(input*0.1);
    }

    public boolean endCondition(Rotation2d angle) {
        return armEncoder.getPosition() > (angle.getDegrees() + 1) && armEncoder.getPosition() < (angle.getDegrees() - 1);
    }

    public void stopSet() {
        armRightMotor.stopMotor();
        armLeftMotor.stopMotor();
        System.out.println("Stop All");
    }

    private void configArmMotor() {
        //armPID.setOutputRange(Constants.Arm.minAngle, Constants.Arm.maxAngle);
        armPID.setFeedbackDevice(armEncoder);
        armRightMotor.getEncoder().setPosition(getPosition().getRotations());
        armPID.setP(0.1);

    }

    private void configFollowerMotor() {
        armLeftMotor.follow(armRightMotor, true);
        armLeftMotor.getPIDController().setP(0.1);
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(armEncoder.getPosition());
    }

    @Override
    public void periodic() {
        Rotation2d motorAngle =Rotation2d.fromRotations(armRightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("ArmPosition", getPosition().getDegrees());
        SmartDashboard.putNumber("InbuiltEncoderAngle", motorAngle.getDegrees());

    }
}
