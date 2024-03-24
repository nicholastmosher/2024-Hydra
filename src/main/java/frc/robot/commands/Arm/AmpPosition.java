package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class AmpPosition extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    Arm arm;

    public AmpPosition(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setAngle(arm.config.ampAngle);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;//arm.endCondition(arm.config.ampAngle);
    }

}
