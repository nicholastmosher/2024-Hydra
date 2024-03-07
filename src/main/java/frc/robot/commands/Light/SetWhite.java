package frc.robot.commands.Light;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.BlinkinLEDController;

public class SetWhite extends Command {
    private final BlinkinLEDController blinkinController;


    public SetWhite(BlinkinLEDController subsystem) {
        blinkinController = subsystem;

        //addRequirements(blinkinController);
    }

    @Override
    public void execute() {
        blinkinController.setWhite();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return blinkinController.isWhite();
    }
}
