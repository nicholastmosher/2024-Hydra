package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command{
    Shooter shooter;

    public StopShooter(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.stopShoot();
    }

    @Override
    public void end(boolean interupted) {
        shooter.stopShoot();
    }

    @Override
    public boolean isFinished() {
        return shooter.isShooterStopped();
    }
}
