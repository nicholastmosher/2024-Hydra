package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class IntakePosition extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    Arm arm;

    public IntakePosition(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(Rotation2d.fromRotations(arm.config.intakeAngle));
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return arm.endCondition(arm.config.intakeAngle);
    }

}
