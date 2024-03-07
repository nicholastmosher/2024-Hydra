package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SendBack extends Command{
    Shooter shooter;


    public SendBack(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);

    }



    @Override
    public void execute() {

        shooter.sendBack();
    }

    @Override
    public void end(boolean interupted) {
        shooter.stopFeed();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
