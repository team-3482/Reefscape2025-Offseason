package frc.robot.led;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.util.Color;
import java.util.HashMap;
import java.util.Map;
import static edu.wpi.first.units.Units.Percent;

/** Colors used with the LEDSubsystem; these are named for readability and priority. */
public enum StatusColors {
    // Default percent is 50
    OFF(Color.kBlack, null, -1, -1),
    CORAL(Color.kWhite, Color.kWhiteSmoke, null, 0, 0.2),
    CAN_ALIGN(Color.kBlue, Color.kDodgerBlue, null, 1, 0.2),
    ERROR(Color.kRed, Color.kTomato,null, 2, 1),
    OK(Color.kGreen, Color.kLime, null, 2, 2.5),
    RSL(new Color(200, 50, 0), Percent.of(40), 5, -1),
    ;

    /** The Colors. */
    public final Color color1, color2;
    /** The priority of this color over other colors (higher = higher priority). */
    public final int priority;
    /** How long this color should stay. -1 is infinitely long. */
    public final double stickyTime;
    /** The brightness to run this color at. */
    public final Dimensionless brightness;

    /**
     * Creates a new StatusColors.
     * @param color1 The first color.
     * @param color2 The second color.
     * @param brightness How bright this color should be. If null, it will be 50%.
     * @param priority The priority of this color over other colors (higher = higher priority).
     * -1 will always be overridden and will always override.
     * @param stickyTime How long this color should stay. -1 is infinitely long.
     */
    StatusColors(Color color1, Color color2, Dimensionless brightness, int priority, double stickyTime) {
        this.color1 = color1;
        this.color2 = color2;
        this.brightness = brightness == null ? Percent.of(75) : brightness;
        this.priority = priority;
        this.stickyTime = stickyTime;
    }

    /**
     * Creates a new StatusColors.
     * @param color The color.
     * @param brightness How bright this color should be. If null, it will be 50%.
     * @param priority The priority of this color over other colors (higher = higher priority).
     * -1 will always be overridden and will always override.
     * @param stickyTime How long this color should stay. -1 is infinitely long.
     */
    StatusColors(Color color, Dimensionless brightness, int priority, double stickyTime) {
        this.color1 = color;
        this.color2 = color;
        this.brightness = brightness == null ? Percent.of(75) : brightness;
        this.priority = priority;
        this.stickyTime = stickyTime;
    }
}