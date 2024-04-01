package frc.robot.commands.Shooter;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PassNote extends Command {
    Shooter shooter;

    public PassNote(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.passNoteShooter();
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
