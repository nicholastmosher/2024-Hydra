package frc.robot.commands.CommandGroups.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopIntake extends Command{
    Shooter shooter;
    Intake intake;

    public StopIntake(Shooter subsystem, Intake isubsystem) {
        shooter = subsystem;
        intake = isubsystem;
        addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        //shooter.shootNote();
        shooter.stopFeed();
        intake.stopIntakeMotor();

    }

    @Override
    public void end(boolean interupted) {
        shooter.stopFeed();
        intake.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        if (shooter.isIndexerStopped() && intake.isIntakeStopped()) {
            return true;
        }
        return false;
    }
}
