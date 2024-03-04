package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FeedNote extends Command{
    Shooter shooter;
    Intake intake;

    public FeedNote(Shooter subsystem, Intake isubsystem) {
        shooter = subsystem;
        intake = isubsystem;
        addRequirements(shooter, intake);
    }

    @Override
    public void execute() {
        //shooter.shootNote();
        if(shooter.isRevved()) {
            shooter.feedNote();
        } else {
            shooter.stopFeed();
        }

    }

    @Override
    public void end(boolean interupted) {
        shooter.stopFeed();
    }

    @Override
    public boolean isFinished() {
        return !intake.endCondition();
    }
}
