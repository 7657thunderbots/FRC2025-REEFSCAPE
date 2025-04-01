package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase { // using subsystem to keep robot function clean af
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public LED() {
        m_led = new AddressableLED(1);
        m_led.setColorOrder(ColorOrder.kRGB);
        m_ledBuffer = new AddressableLEDBuffer(100);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        setLedsWhite();
    }

    // I used chatgpt for this part below as I have no idea how to convert it
    // public void setLEDColorHSV(int index, double h, double s, double v) {
    // int rgb = hsvToRgb(h, s, v);
    // int r = (rgb >> 16) & 0xFF;
    // int g = (rgb >> 8) & 0xFF;
    // int b = rgb & 0xFF;
    // m_ledBuffer.setRGB(index, r, g, b);
    // m_led.setData(m_ledBuffer);
    // }

    public void clearLEDs() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 100, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setLEDsRed() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kRed);
            m_led.setData(m_ledBuffer);
        }
    }

    public void setLEDsOrange() {
        Color orange = new Color("#b37400");
        // Color orange = new Color(242, 112, 5);
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, orange);
            m_led.setData(m_ledBuffer);
        }
    }

    public void setLedsWhite() {
        // Color orange = new Color(242, 112, 5);
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kWhite);
            m_led.setData(m_ledBuffer);
        }
    }
}