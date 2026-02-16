package frc.robot.Subsystems.Led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Led {

  private enum BlinkState {
    RAPID,
    SLOW,
    SOLID
  }

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private BlinkState blink;

  private static Led instance;

  /**
   * Gets the current instance of {@link Led}. If none exists, it it created
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
  }

  /**
   * Sets the color of the LED for a specified range
   *
   * @param color A color
   * @param start Start of the range
   * @param end End of the range
   * @see setColor
   */
  public void setColor(Color color, int start, int end) {
    setColorNoOutput(color, start, end);
    setOutput();
  }

  /**
   * Sets the color of the LED *buffer* for a specified range. This does not flush the LED buffer to
   * the physical LED; make sure to call {@link #setOutput()}.
   *
   * @param color The color to set
   * @see setColor
   */
  public void setColorNoOutput(Color color, int start, int end) {
    start = MathUtil.clamp(start, 0, LedConstants.ledLength);
    end = MathUtil.clamp(end, start, LedConstants.ledLength);

    for (int i = start; i < end; i++) {
      buffer.setLED(i, color);
    }
  }

  /** Flushes the LED buffer to the physical LED */
  public void setOutput() {
    led.setData(buffer);
    led.start();
  }
}
