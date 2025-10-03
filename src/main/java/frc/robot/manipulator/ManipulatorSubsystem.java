// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.manipulator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;

/** An example subsystem that does nothing. */
public class ManipulatorSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ManipulatorSubsystemHolder {
        private static final ManipulatorSubsystem INSTANCE = new ManipulatorSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ManipulatorSubsystem getInstance() {
        return ManipulatorSubsystemHolder.INSTANCE;
    }

    private TalonFX coralMotor = new TalonFX(ManipulatorConstants.CORAL_MOTOR_ID);
    private TalonFX algaeMotor = new TalonFX(ManipulatorConstants.ALGAE_MOTOR_ID);

    private ManipulatorSubsystem() {
        super("ManipulatorSubsystem");
    }

    @Override
    public void periodic() {}

    /**
     * Set the speed of the coral motor.
     * @param speed - Speed between -1.0 and 1.0
     */
    public void setCoralMotor(double speed) {
        coralMotor.set(speed);
    }

    /**
     * Set the speed of the algae motor.
     * @param speed - Speed between -1.0 and 1.0
     */
    public void setAlgaeMotor(double speed) {
        algaeMotor.set(speed);
    }
}