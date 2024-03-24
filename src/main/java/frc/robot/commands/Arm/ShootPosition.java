package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ShootPosition extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    Arm arm;

    public ShootPosition(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setAngle(arm.config.shootAngle);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return arm.endCondition(arm.config.shootAngle);
    }

}
