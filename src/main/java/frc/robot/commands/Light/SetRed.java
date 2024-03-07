package frc.robot.commands.Light;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.BlinkinLEDController;
import frc.robot.classes.BlinkinLEDController.BlinkinPattern;

public class SetRed extends Command {
    private final BlinkinLEDController blinkinController;


    public SetRed(BlinkinLEDController subsystem) {
        blinkinController = subsystem;

        //addRequirements(blinkinController);
    }

    @Override
    public void execute() {
        blinkinController.setRed();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return blinkinController.isRed();
    }
}
