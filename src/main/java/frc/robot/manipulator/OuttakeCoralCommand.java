package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;

/** A command that outtakes the coral */
public class OuttakeCoralCommand extends Command {
    public OuttakeCoralCommand() {
        setName("OuttakeCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setCoralMotor(ManipulatorConstants.CORAL_OUTTAKE_SPEED);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {}


    @Override
    public boolean isFinished() {
        return false;
    }
}
