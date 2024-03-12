package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;
import frc.lib.config.IntakeConfig;
import frc.robot.classes.ColorSensorController;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    public final IntakeConfig config;
    public final ColorSensorController colorSensor;

    public Intake(IntakeConfig config, ColorSensorController colorSensorController) {
        this.config = config;
        intakeMotor = new CANSparkMax(this.config.intakeMotorId, CANSparkLowLevel.MotorType.kBrushless);
        colorSensor = colorSensorController;
    }

    public void setIntakeMotor(boolean invert) {
        if (!invert) {
            intakeMotor.set(this.config.intakeMotorSpeed);
            return;
        }
        intakeMotor.set(-this.config.intakeMotorSpeed);
    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    public boolean isIntakeStopped() {
        if (intakeMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }

    public boolean isIntaked() {
        return colorSensor.isSeen();
    }

}
