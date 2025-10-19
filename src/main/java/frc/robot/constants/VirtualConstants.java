// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Contains virtual constants that do not correlate to anything physical in real life. */
public final class VirtualConstants {
    /** Constants for the controller and any controller-related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Whether to accept directional pad input for movement. */
        public static final boolean DPAD_DRIVE_INPUT = true;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    /** Tab names in Elastic. */
    public static final class DashboardTabNames {
        public static final String TELEOP = "Teleoperated";
        public static final String AUTON = "Autonomous";
        public static final String DEV = "Dev";
    }

    /** Constants for elevator heights */
    public static final class ScoringConstants {
        // TODO update these values with the new elevator
        /* Heights in elevator meters for scoring at these heights. */
        public static final double L2_ALGAE = 0.08;
        public static final double L3_ALGAE = 0.48;

        public static final double L1_CORAL = 0.235;
        public static final double L2_CORAL = 0.34;
        public static final double L3_CORAL = 0.74;
        public static final double L4_CORAL = 0.74; // placeholder

        public static final double BOTTOM_HEIGHT = 0;
        public static final double MAX_HEIGHT = 1.15;

        public static final double SLOW_DRIVE_HEIGHT = 0.4;
        public static final double INTAKING_HEIGHT = 0;
        public static final double IDLE_HEIGHT = L2_ALGAE;
    }
}