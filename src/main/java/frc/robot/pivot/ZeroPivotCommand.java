package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.PivotConstants;

/** A command that pushes the pivot down to the hardstop to zero it */
public class ZeroPivotCommand extends Command {

    /** Creates a new ZeroPivotCommand. */
    public ZeroPivotCommand() {
        setName("ZeroPivotCommand");

        addRequirements(PivotSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        PivotSubsystem.getInstance().setPivotSpeed(-PivotConstants.ZERO_SPEED, false);
        System.out.println("Zeroing Pivot!");
    }


    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        PivotSubsystem.getInstance().setPivotSpeed(0);
        PivotSubsystem.getInstance().setPositionHardstop();
        // LEDSubsystem.getInstance().setColor(interrupted ? StatusColors.ERROR : StatusColors.OK);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}