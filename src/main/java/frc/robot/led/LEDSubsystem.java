package frc.robot.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.PhysicalConstants.LEDConstants;

import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class LEDSubsystemHolder {
        private static final LEDSubsystem INSTANCE = new LEDSubsystem();
    }

    public static LEDSubsystem getInstance() {
        return LEDSubsystemHolder.INSTANCE;
    }

    private final AddressableLED LEDStrip = new AddressableLED(LEDConstants.PWM_HEADER);
    private final AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
    private final Distance LEDSpacing = Units.Meter.of(1 / 120.0);

    private StatusColors currentColor = StatusColors.OFF;
    private StatusColors blinkColor = StatusColors.OFF;

    private boolean shouldBlink = false;
    private final Timer blinkTimer = new Timer();
    private final Timer stickyTimer = new Timer();

    private LEDPattern pattern = LEDPattern.rainbow(255, 128); // Placeholder until something else gets loaded

    /** Creates a new LEDSubsystem. */
    private LEDSubsystem() {
        super("LEDSubsystem");

        this.LEDStrip.setLength(this.LEDStripBuffer.getLength());
        this.LEDStrip.setData(this.LEDStripBuffer);
        this.LEDStrip.start();

        this.blinkTimer.start();

        SmartDashboard.putString("LED/Color1", StatusColors.OFF.color1.toHexString());
        SmartDashboard.putString("LED/Color2", StatusColors.OFF.color2.toHexString());
    }

    @Override
    public void periodic() {
        if (this.shouldBlink && this.blinkTimer.hasElapsed(LEDConstants.BLINK_COOLDOWN)) {
            if (this.currentColor == StatusColors.OFF) {
                this.setColor(this.blinkColor, true);
            }
            else {
                this.setColor(StatusColors.OFF, true);
            }

            this.blinkTimer.reset();
        }

        double stickyTime = this.blinkColor.stickyTime;
        // Blink color will always be current color, unless blinking, when it is always the blink color
        if (stickyTime >= 0 && this.stickyTimer.hasElapsed(stickyTime)) {
            setColor(StatusColors.OFF);
        }

        if(LEDConstants.SCROLL){
            pattern.applyTo(this.LEDStripBuffer);
            this.LEDStrip.setData(this.LEDStripBuffer);
        }
    }

    /**
     * Forcefully sets the current color of led strip.
     * @param newColor - The color to set. Will only be set if the priority is higher than the current one.
     * @param forBlink - Whether the color should be blinked.
     * @see LEDSubsystem#periodic()
     */
    private void setColor(StatusColors newColor, boolean forBlink) {
        if (newColor.priority != -1 && newColor.priority < this.currentColor.priority) {
            return;
        }

        if (newColor.stickyTime >= 0) {
            this.stickyTimer.restart();
            if (newColor.equals(this.currentColor)) {
                return;
            }
        }

        this.shouldBlink = forBlink;

        if (this.shouldBlink) {
            this.blinkTimer.reset();
        }

        // LEDPattern pattern = LEDPattern.solid(newColor.color).atBrightness(newColor.brightness);
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, newColor.color1, newColor.color2);
        if(LEDConstants.SCROLL){
            pattern = pattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(LEDConstants.SCROLL_SPEED), LEDSpacing);
        } else {
            pattern.applyTo(this.LEDStripBuffer);
            this.LEDStrip.setData(this.LEDStripBuffer);
        }

        // Don't save black as a blink color on blinks
        if (this.shouldBlink && newColor == StatusColors.OFF) {
            this.currentColor = newColor;
        }
        else {
            this.blinkColor = this.currentColor = newColor;
        }

        updateDashboardAndLogs(newColor);
    }

    /**
     * Updates Dashboard and AdvantageScope logs.
     * @param newColor - The new color to use.
     */
    private void updateDashboardAndLogs(StatusColors newColor) {
        String hexString1 = newColor.color1.toHexString();
        String hexString2 = newColor.color2.toHexString();

        Logger.recordOutput("LED/Status", newColor);
        Logger.recordOutput("LED/Color1", hexString1);
        Logger.recordOutput("LED/Color2", hexString2);

        // Do this because, to display orange on the LEDs, we have to use dark red,
        // but it looks like orange, and thus it should be an actual orange on the dashboard.
        if(newColor.equals(StatusColors.RSL)){
            hexString1 = Color.kDarkOrange.toHexString();
            hexString2 = hexString1;
        }

        SmartDashboard.putString("LED/Color1", hexString1);
        SmartDashboard.putString("LED/Color2", hexString2);
    }

    /**
     * Forcefully sets the current color of led strip
     * @param newColor - The color to set.
     */
    public void setColor(StatusColors newColor) {
        setColor(newColor, false);
    }

    /**
     * Blinks the color every LEDConstants.BLINK_COOLDOWN
     * @param newColor - The color to blink.
     */
    public void blinkColor(StatusColors newColor) {
        setColor(newColor, true);
    }

    /**
     * Gets the currently blinking color.
     * @return The color.
     */
    public StatusColors getBlinkColor() {
        return this.blinkColor;
    }
}
