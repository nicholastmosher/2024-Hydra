package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SendBack extends Command{
    Shooter shooter;
    private int counter = 0;
    private int target = 0;

    public SendBack(Shooter subsystem, double seconds) {
        shooter = subsystem;
        addRequirements(shooter);
        target = (int)(seconds * 50);
    }



    @Override
    public void execute() {
        if(counter < target)
            counter++;

        shooter.sendBack();
    }

    @Override
    public void end(boolean interupted) {
        shooter.stop();
    }
    @Override
    public boolean isFinished() {
        return counter >= target;
    }
}
