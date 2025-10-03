package frc.robot.constants;

/** Contains physical constants that are related to real-life things.
 * For example, device IDs, speeds, ratios/multipliers, etc.
 * */
public final class PhysicalConstants {
    // Constants for the manipulator module (the claw thing)
    public static final class ManipulatorConstants {
        // The speed at which the coral motor will go at to intake the coral tube
        public static final double CORAL_INTAKE_SPEED = 0;
        // The speed at which the algae motor will go at to intake the algae ball
        public static final double ALGAE_INTAKE_SPEED = 0;
        // The speed at which the motors will go at to outtake the balls
        public static final double CORAL_OUTTAKE_SPEED = 0;
        // The speed at which the algae motor will go at to outtake the algae ball
        public static final double ALGAE_OUTTAKE_SPEED = 0;
        // The speed at which the algae motor will go at to hold the coral tube in place
        public static final double ALGAE_STALL_SPEED = 0;

        // TODO: Fill these ids out
        // The TalonFX motor id for the coral motor
        public static final int CORAL_MOTOR_ID = -1;
        // The TalonFX motor id for the algae motor
        public static final int ALGAE_MOTOR_ID = -1;
    }
}
