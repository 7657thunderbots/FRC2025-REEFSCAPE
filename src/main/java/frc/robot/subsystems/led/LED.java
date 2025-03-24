package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase { // using subsystem to keep robot function clean af
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public LED(int port, int length) {
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    // I used chatgpt for this part below as I have no idea how to convert it
    public void setLEDColorHSV(int index, double h, double s, double v) {
        int rgb = hsvToRgb(h, s, v);
        int r = (rgb >> 16) & 0xFF;
        int g = (rgb >> 8) & 0xFF;
        int b = rgb & 0xFF;
        m_ledBuffer.setRGB(index, r, g, b);
        m_led.setData(m_ledBuffer);
    }

    public void clearLEDs() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    // I used chatgpt for this part below as I know with hsv you get better over
    // brightness
    private int hsvToRgb(double h, double s, double v) {
        int r, g, b;

        int i = (int) Math.floor(h * 6);
        double f = h * 6 - i;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);

        switch (i % 6) {
            case 0:
                r = (int) (v * 255);
                g = (int) (t * 255);
                b = (int) (p * 255);
                break;
            case 1:
                r = (int) (q * 255);
                g = (int) (v * 255);
                b = (int) (p * 255);
                break;
            case 2:
                r = (int) (p * 255);
                g = (int) (v * 255);
                b = (int) (t * 255);
                break;
            case 3:
                r = (int) (p * 255);
                g = (int) (q * 255);
                b = (int) (v * 255);
                break;
            case 4:
                r = (int) (t * 255);
                g = (int) (p * 255);
                b = (int) (v * 255);
                break;
            case 5:
                r = (int) (v * 255);
                g = (int) (p * 255);
                b = (int) (q * 255);
                break;
            default:
                throw new RuntimeException(
                        "Something went wrong when converting from HSV to RGB. Input was " + h + ", " + s + ", " + v);
        }

        return (r << 16) | (g << 8) | b;
    }

    public void setAllLEDsColorHSV(double h, double s, double v) {
        int rgb = hsvToRgb(h, s, v);
        int r = (rgb >> 16) & 0xFF;
        int g = (rgb >> 8) & 0xFF;
        int b = rgb & 0xFF;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }
}