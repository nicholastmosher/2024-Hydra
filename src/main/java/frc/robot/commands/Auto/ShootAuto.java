package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class ShootAuto extends Command{
    Shooter shooter;
    Indexer indexer;


    public ShootAuto(Shooter subsystem, Indexer indexer) {
        shooter = subsystem;
        this.indexer = indexer;
        addRequirements();
    }

    @Override
    public void execute() {
        shooter.startShooter();
        indexer.feedNote();
    }

//    @Override
//    public void execute() {
//    }

    @Override
    public void end(boolean interupted) {
        indexer.stopIndexer();
        shooter.stopShoot();
    }
//    @Override
//    public boolean isFinished() {
//        return shooter.isRevved();
//    }
}
