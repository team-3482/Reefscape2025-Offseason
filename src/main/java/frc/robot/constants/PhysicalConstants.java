package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

/** Contains physical constants that are related to real-life things.
 * For example, device IDs, speeds, ratios/multipliers, etc.
 * */
public final class PhysicalConstants {
    public static final class RobotConstants {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    /** Constants for the manipulator subsystem */
    public static final class ManipulatorConstants {
        // TODO: Fill out speeds
        /** The CAN ID for the Manipulator Coral TalonFX */
        public static final int CORAL_MOTOR_ID = 23;
        /** The CAN ID for the Manipulator Algae TalonFX  */
        public static final int ALGAE_MOTOR_ID = 24;
        /** The CAN ID for the Funnel (Coral) TalonFX */
        public static final int FUNNEL_MOTOR_ID = 25;

        /** The speed at which the Manipulator Coral motor will intake at */
        public static final double CORAL_INTAKE_SPEED = 0.1;
        /** The speed at which the Manipulator Coral motor will outtake at */
        public static final double CORAL_OUTTAKE_SPEED = 0.1;
        /** The speed at which the Manipulator Algae motor will intake at */
        public static final double ALGAE_INTAKE_SPEED = 0.1;
        /** The speed at which the Manipulator Algae motor will outtake at */
        public static final double ALGAE_OUTTAKE_SPEED = 0.1;
        /** The speed at which the Manipulator Algae motor will hold the Algae in place with */
        public static final double ALGAE_HOLD_SPEED = 0.05;
        /** The speed at which the Funnel (Coral) motor will intake at */
        public static final double FUNNEL_INTAKE_SPEED = 0.1;
    }
}