package frc.robot.Subsystems.Led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class Led extends SubsystemBase {

  private enum BlinkState {
    RAPID,
    SLOW,
    SOLID
  }

  private Led LED;
  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private BlinkState blink;
  private static Led instance;

  /**
   * Gets the current instance of {@blink Led}. If none exists, one is created
   *
   * @return
   */
  public static Led getInstance() {
    return instance;
  }

  private Led() {
    led = new AddressableLED(LedConstants.ledPort);
    buffer = new AddressableLEDBuffer(LedConstants.ledLength);
    led.setLength(buffer.getLength());

    if (instance == null) {
      instance = new Led();
    }

    setDefaultCommand(
        Commands.either(
                runOnce(() -> setTeamColors()),
                Commands.select(
                    (Map.ofEntries(
                        Map.entry(BlinkState.RAPID, setABeautifulHue()),
                        Map.entry(BlinkState.SLOW, blink(0.20, 184, 100, 84)),
                        Map.entry(BlinkState.SOLID, blink(0.10, 184, 100, 84)))),
                    () -> blink),
                DriverStation::isDisabled)
            .ignoringDisable(true));
  }

  public Command setABeautifulHue() {
    return Commands.runOnce(() -> setColor(184, 100, 84, 0, LedConstants.ledLength), LED);
  }

  public Command setCitrus() {
    return Commands.runOnce(() -> setColor(72, 77, 86, 0, LedConstants.ledLength), LED);
  }

  public Command blinkTeamColors(double interrupt) {
    return Commands.sequence(
      Commands.runOnce(()-> setHalves(184, 100, 84, 72, 77, 86), LED),
      Commands.waitSeconds(interrupt),
      Commands.runOnce(()-> setHalves(72, 77, 86, 184, 100, 84), LED),
      Commands.waitSeconds(interrupt));
  }

  public Command setTeamColors() {
    return Commands.runOnce(() -> setHalves(184, 100, 84, 72, 77, 86), LED);
  }

  public Command chase(int h, int s, int v) {
      for(int i = 0; i < LedConstants.ledLength; i ++) {
        return Commands.sequence(
          Commands.runOnce(() -> setColor(h, s, v, i, i++)),
          Commands.waitSeconds(0.5);
    }
  }
  public Command blink(double interrupt, int h, int s, int v) {
    return Commands.sequence(
        runOnce(() -> setColor(h, s, v, 0, LedConstants.ledLength)),
        Commands.waitSeconds(interrupt),
        runOnce(() -> setColor(h, s, v, 0, LedConstants.ledLength)),
        Commands.waitSeconds(interrupt));
  }

  public void setHalves(int h1, int s1, int v1, int h2, int s2, int v2){
    setColorNoOutput(h1, s1, v1, 0, (LedConstants.ledLength / 2) + 1);
    setColorNoOutput(h2, s2, v2, ((LedConstants.ledLength / 2) + 1), LedConstants.ledLength);

    setOutput();
  }

  /**
   * Sets the color of the LED for a specified range
   *
   * @param h Hue
   * @param s Saturation
   * @param v Value
   * @param start Start of the range
   * @param end End of the range
   * @see setColor
   */
  public void setColor(int h, int s, int v, int start, int end) {
    setColorNoOutput(h, s, v, start, end);
    setOutput();
  }

  /**
   * Sets the color of the LED *buffer* for a specified range. This does not flush the LED buffer to
   * the physical LED; make sure to call {@link #setOutput()}.
   *
   * @param h Hue
   * @param s Saturation
   * @param v Value
   * @see setColor
   */
  public void setColorNoOutput(int h, int s, int v, int start, int end) {
    start = MathUtil.clamp(start, 0, LedConstants.ledLength);
    end = MathUtil.clamp(end, start, LedConstants.ledLength);

    for (int i = start; i < end; i++) {
      buffer.setHSV(i, h, s, v);
    }
  }

  /** Flushes the LED buffer to the physical LED */
  public void setOutput() {
    led.setData(buffer);
    led.start();
  }
}
