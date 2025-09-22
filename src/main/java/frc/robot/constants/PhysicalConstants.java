package frc.robot.constants;

/** Contains physical constants that are related to real-life things.
 * For example, device IDs, speeds, ratios/multipliers, etc.
 * */
public class PhysicalConstants {
    /** Constants for physical attributes of the robot. */
    public static final class RobotConstants {
        /** Name of the CTRE CAN bus (configured on the CANivore). */
        public static final String CTRE_CAN_BUS = "ctre";
    }

    /** Constants for the LED strips */
    public static final class LEDConstants {
        /** Id for PWM */
        public static final int PWM_HEADER = 0;
        /** This is how many LEDs are on the strip */
        public static final int LED_LENGTH = 80;
        /** Blink cooldown; time that the selected blink color will stay for, then blink */
        public static final double BLINK_COOLDOWN = 0.2;
        /** Animate scrolling gradient for the LEDs */
        public static final boolean SCROLL = true; // Might cause horrible performance issues, turn off if rio lagging
        /** How fast (m/s) to scroll */
        public static final double SCROLL_SPEED = 1.0;
    }
}
