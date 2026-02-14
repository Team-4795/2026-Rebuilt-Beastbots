package frc.robot.Subsystems.Led;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Led {

    private enum blinkState {
        rapid, 
        slow,
        solid
    }

    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private blinkState blink;

    private static Led instance;
    private static Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }
        return instance;
    }

    private Led() {
        led = new AddressableLED(LedConstants.ledPort);
        buffer  = new AddressableLEDBuffer(LedConstants.ledLength);
        led.setLength(buffer.getLength());
    }
    
    // h = Hue, s = Saturation, v = Value, start = Starting Point, end = Ending Point
    private void setColor(int h, int s, int v, int start, int end) {
        setColorNoOutput(h, s, v, start, end);
        setOutput();
    }

    /*  Sets the HSV value of the color, where the color starts,
        and where the color ends */
    private void setColorNoOutput(int h, int s, int v, int start, int end) {
        start = MathUtil.clamp(start, 0, LedConstants.ledLength);
        end = MathUtil.clamp(end, start, LedConstants.ledLength);

        for(int i = start; i < end; i ++) {
        buffer.setHSV(i, h, s, v);
        }
    }

    // Sets the settings of the LEDS to the buffer settings, then starts it
    private void setOutput() {
        led.setData(buffer);
        led.start();
    }
    }

