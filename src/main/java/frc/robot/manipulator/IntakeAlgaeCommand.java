package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;

/** A command that intakes the algae, then sets a stalling voltage to hold it until outtake */
public class IntakeAlgaeCommand extends Command {
    public IntakeAlgaeCommand() {
        setName("IntakeAlgaeCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(ManipulatorConstants.MANIPULATOR_ALGAE_INTAKE_SPEED);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(ManipulatorConstants.MANIPULATOR_ALGAE_STALL_SPEED);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}