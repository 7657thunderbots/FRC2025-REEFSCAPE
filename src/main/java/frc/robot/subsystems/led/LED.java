// 
package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class LED extends SubsystemBase { // using subsystem to keep robot function clean af
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final Timer m_blinkTimer = new Timer();
    private boolean m_isBlinkingGreen = false;
    private boolean m_isBlinkingOrange = false;
    private boolean m_isGreen = false;
    private boolean auto_drive;

    public LED() {
        auto_drive = false;
        m_led = new AddressableLED(5);
        m_ledBuffer = new AddressableLEDBuffer(100);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        clearLEDs();
        setLEDsRed();

    }

    public void clearLEDs() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 100, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setLEDsRed() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 60, 255, 255);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setLEDsOrange() {
        // Color orange = new Color("#b37400");
        // Color orange = new Color(242, 112, 5);
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 55, 255, 255);

        }
        m_led.setData(m_ledBuffer);
    }

    public void setLedsWhite() {
        if (m_isBlinkingGreen || m_isBlinkingOrange) {
        } else {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, Color.kWhite);
                m_led.setData(m_ledBuffer);
            }
        }
    }

    public void setLEDsBlack() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_led.setData(m_ledBuffer);
        }
    }

    public void setLedsGreen() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 180, 255, 255);

        }
        m_led.setData(m_ledBuffer);
    }

    public void setLEDSLightBlue() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 160, 252, 255);
        }
        m_led.setData(m_ledBuffer);
    }
}