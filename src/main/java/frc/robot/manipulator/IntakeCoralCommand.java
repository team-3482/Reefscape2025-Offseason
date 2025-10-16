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
        ManipulatorSubsystem.getInstance().setCoralMotor(ManipulatorConstants.MANIPULATOR_CORAL_INTAKE_SPEED);
        ManipulatorSubsystem.getInstance().setFunnelMotor(ManipulatorConstants.FUNNEL_INTAKE_SPEED);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setCoralMotor(0);
        ManipulatorSubsystem.getInstance().setFunnelMotor(0);
    }


    @Override
    public boolean isFinished() {
        return ManipulatorSubsystem.getInstance().hasCoral();
    }
}