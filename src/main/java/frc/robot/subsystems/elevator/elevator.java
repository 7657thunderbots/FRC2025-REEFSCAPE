
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private final TalonFX KrakenLeader = new TalonFX(14);
    private final TalonFX KrakenFollower = new TalonFX(13);
    private PIDController pidController;
    public double elbowSetPoint = 0;
    private double errorSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;
    private double kP = 0.04;
    private double kI = 0.0;
    private double kD = 0.0;
    private double b= .04;
    private double hiLimit = 0.1; // Threshold for integral term
    private final int CURRENT_LIMIT = 10; // Current limit in amps
    private final double GRAVITY_FEEDFORWARD = 0.2; // Feedforward term to counteract gravity
    public elevator() {

        // SmartDashboard.setDefaultNumber("Target Position", 0);
        // SmartDashboard.setDefaultNumber("Target Velocity", 0);
        // SmartDashboard.setDefaultBoolean("Control Mode", false);
        // SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit =10;
    KrakenLeader.getConfigurator().apply(config);
    KrakenFollower.getConfigurator().apply(config);
KrakenLeader.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
KrakenFollower.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

    
    }

    

    public Command elevatorL1() {
        return runOnce(() -> {
            this.elbowSetPoint = -3;
             // change later
        });
    }

    @Override
    public void periodic() {
        double error = elbowSetPoint - KrakenLeader.getPosition().getValueAsDouble();
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < hiLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;
        double output = kP * error + kI * errorSum + kD * errorRate;


        KrakenLeader.set((output - b));
        KrakenFollower.set((output - b ));

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;

        SmartDashboard.putNumber("elevator", KrakenLeader.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("motor amp", motor.getSupplyCurrent());
    }
}
