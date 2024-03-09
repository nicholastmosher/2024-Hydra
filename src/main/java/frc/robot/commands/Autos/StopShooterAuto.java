package frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooterAuto extends Command{
    Shooter shooter;

    public StopShooterAuto(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interupted) {
        shooter.stopShoot();
        shooter.stopFeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
