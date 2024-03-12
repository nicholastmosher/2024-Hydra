package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopIntake extends Command{
    Intake intake;

    public StopIntake(Intake subsystem) {
        intake = subsystem;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.stopIntakeMotor();

    }

    @Override
    public void end(boolean interupted) {
        intake.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        if (intake.isIntakeStopped()) {
            return true;
        }
        return false;
    }
}
