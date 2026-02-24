package frc.robot.Subsystems.Led;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    if (instance == null) {
      instance = new Led();
    }
    return instance;
  }

  private Led() {
    led = new AddressableLED(LedConstants.ledPort);
    buffer = new AddressableLEDBuffer(LedConstants.ledLength);
    led.setLength(buffer.getLength());

    setDefaultCommand(Commands.either(Commands.runOnce(() -> setTeamColors(), LED), 
    Command.select(Map.ofEntries(
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null))), 
    null));
  }

  /** Sets the led strip to the team color */
  private void setTeamColors() {
    setColorNoOutput(180, 80, 80, 0, (LedConstants.ledLength / 2));
    setColorNoOutput(90, 100, 100, (LedConstants.ledLength / 2 + 1), LedConstants.ledLength);

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
