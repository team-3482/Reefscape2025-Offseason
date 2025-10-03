package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;

/** A command that intakes the coral */
public class IntakeCoralCommand extends Command {
    public IntakeCoralCommand() {
        setName("IntakeCoralCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setCoralMotor(ManipulatorConstants.CORAL_INTAKE_SPEED);
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
