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
