package frc.robot.commands.Shooter;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SendBackShooter extends Command {
    Shooter shooter;

    public SendBackShooter(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.sendBackShooter();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
