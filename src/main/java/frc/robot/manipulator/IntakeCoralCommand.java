package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;
import frc.robot.constants.VirtualConstants.SubsystemStates;

/** A command that intakes the coral */
public class IntakeCoralCommand extends Command {
    public IntakeCoralCommand() {
        setName("IntakeCoralCommand");
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setCoralMotor(ManipulatorConstants.CORAL_INTAKE_SPEED);
        ManipulatorSubsystem.getInstance().setFunnelMotor(ManipulatorConstants.FUNNEL_INTAKE_SPEED);

        ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.INTAKING);
        ManipulatorSubsystem.getInstance().setState("Funnel", SubsystemStates.INTAKING);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setCoralMotor(0);
        ManipulatorSubsystem.getInstance().setFunnelMotor(0);

        ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.IDLE);
        ManipulatorSubsystem.getInstance().setState("Funnel", SubsystemStates.IDLE);
    }


    @Override
    public boolean isFinished() {
        return
            ManipulatorSubsystem.getInstance().getCoralRotorVelocity() == 0 &&
            ManipulatorSubsystem.getInstance().getCoralStatorCurrent() >= ManipulatorConstants.CORAL_CURRENT_LIMIT;
    }
}