package frc.robot.commands.Initialize;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class ClimberInit extends Command {
    Climber climber;

    public ClimberInit(Climber c) {
        climber = c;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.init();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}
