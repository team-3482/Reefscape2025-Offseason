package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;

/** A command that outtakes the algae */
public class OuttakeAlgaeCommand extends Command {
    public OuttakeAlgaeCommand() {
        setName("OuttakeAlgaeCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(ManipulatorConstants.ALGAE_OUTTAKE_SPEED);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(0);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
