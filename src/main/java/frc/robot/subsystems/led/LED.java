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

    public LED() {
        m_led = new AddressableLED(1);
        m_led.setColorOrder(ColorOrder.kRGB);
        m_ledBuffer = new AddressableLEDBuffer(100);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        setLedsWhite();
        m_blinkTimer.start();
    }

    @Override
    public void periodic() {
        if (m_isBlinkingGreen) {
            if (m_blinkTimer.get() > 0.1) {
                if (m_isGreen) {
                    setLEDsBlack(); // Turn off LEDs
                    m_isGreen = false;
                } else {
                    setLEDsGreen(); // Turn on green LEDs
                    m_isGreen = true;
                }
                m_blinkTimer.reset();
            }
        } else if (m_isBlinkingOrange) {
            if (m_blinkTimer.get() > 0.1) {
                if (m_isGreen) {
                    setLEDsBlack(); // Turn off LEDs
                    m_isGreen = false;
                } else {
                    setLEDsOrange(); // Turn on orange LEDs
                    m_isGreen = true;
                }
                m_blinkTimer.reset();
            }
        }
    }

    public void startBlinkingGreen() {
        m_isBlinkingGreen = true;
        m_isBlinkingOrange = false;
        m_isGreen = false;
        m_blinkTimer.reset();
    }

    public void startBlinkingOrange() {
        m_isBlinkingOrange = true;
        m_isBlinkingGreen = false;
        m_isGreen = false;
        m_blinkTimer.reset();
    }

    public void stopBlinking() {
        m_isBlinkingGreen = false;
        m_isBlinkingOrange = false;
        setLedsWhite(); // Revert to default color
    }

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
        if (m_isBlinkingGreen|| m_isBlinkingOrange) {
        }
        else{
        Color orange = new Color("#b37400");
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, orange);
            m_led.setData(m_ledBuffer);
        }}
    }

    public void setLEDsGreen() {
        if (m_isBlinkingGreen|| m_isBlinkingOrange) {
        }
        else{
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kGreen);
            m_led.setData(m_ledBuffer);
        }}
    }

    public void setLedsWhite() {
        if (m_isBlinkingGreen|| m_isBlinkingOrange) {
        }
        else{
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kWhite);
            m_led.setData(m_ledBuffer);
        }}
    }

    public void setLEDsBlack() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_led.setData(m_ledBuffer);
        }
    }
}