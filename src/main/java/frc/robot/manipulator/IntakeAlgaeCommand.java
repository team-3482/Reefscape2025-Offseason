package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;
import frc.robot.constants.VirtualConstants.SubsystemStates;

/** A command that intakes the algae, then sets a stalling voltage to hold it until outtake */
public class IntakeAlgaeCommand extends Command {
    public IntakeAlgaeCommand() {
        setName("IntakeAlgaeCommand");
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(-ManipulatorConstants.ALGAE_INTAKE_SPEED);

        ManipulatorSubsystem.getInstance().setState("Algae", SubsystemStates.INTAKING);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(-ManipulatorConstants.ALGAE_HOLD_SPEED);

        ManipulatorSubsystem.getInstance().setState("Algae", SubsystemStates.HOLDING);
    }


    @Override
    public boolean isFinished() {
        return
            ManipulatorSubsystem.getInstance().getAlgaeRotorVelocity() == 0 &&
            ManipulatorSubsystem.getInstance().getAlgaeStatorCurrent() >= ManipulatorConstants.ALGAE_CURRENT_LIMIT;
    }
}