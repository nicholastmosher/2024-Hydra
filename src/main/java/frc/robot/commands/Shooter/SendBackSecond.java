package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SendBackSecond extends Command{
    Shooter shooter;


    public SendBackSecond(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);

    }



    @Override
    public void execute() {

        shooter.sendBack();
        shooter.sendBackShooter();;
    }

    @Override
    public void end(boolean interupted) {
        shooter.stopFeed();
        shooter.stopShoot();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
