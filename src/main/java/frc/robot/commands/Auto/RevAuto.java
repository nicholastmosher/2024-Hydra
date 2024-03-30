package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RevAuto extends Command{
    Shooter shooter;

    public RevAuto(Shooter subsystem) {
        shooter = subsystem;
        addRequirements();
    }

    @Override
    public void execute() {
        shooter.startShooter();
    }

//    @Override
//    public void execute() {
//    }

    @Override
    public void end(boolean interupted) {
    }
//    @Override
//    public boolean isFinished() {
//        return shooter.isRevved();
//    }
}
