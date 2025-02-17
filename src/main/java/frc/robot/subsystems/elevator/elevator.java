package frc.robot.subsystems.elevator;

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

public class elevator extends SubsystemBase {

    private double MAX_VALUE;
    private double elevatorSetPoint;
    private final TalonFX KrakenLeader = new TalonFX(14);
    private final TalonFX KrakenFollower = new TalonFX(13);
    private PIDController krakenPidController;
    public double krakenSetPoint;
    private final DutyCycleOut krakenLeftOut = new DutyCycleOut(0);
    private final DutyCycleOut krakenRightOut = new DutyCycleOut(0);

    // Feedforward gain to counteract gravity
    private static final double FEEDFORWARD_GAIN = 0.1; // Adjust this value based on your system

    public void initializeKraken() {
        krakenPidController = new PIDController(0.0, 0.0, 0.00); // Kp, Ki, Kd
        krakenSetPoint = 0; // Desired position
        var krakenLeftConfiguration = new TalonFXConfiguration();
        var krakenRightConfiguration = new TalonFXConfiguration();

        krakenLeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        krakenRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        krakenRightConfiguration.CurrentLimits.SupplyCurrentLimit = Double.MAX_VALUE;
        krakenRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        KrakenLeader.getConfigurator().apply(krakenLeftConfiguration);
        KrakenFollower.getConfigurator().apply(krakenRightConfiguration);

        KrakenFollower.setControl(new Follower(KrakenLeader.getDeviceID(), true));
        KrakenLeader.setSafetyEnabled(true);
    }

    public void controlKraken() {
        double krakenError = krakenSetPoint - KrakenLeader.getPosition().getValueAsDouble();
        double krakenOutput = krakenPidController.calculate(KrakenLeader.getPosition().getValueAsDouble(), krakenSetPoint);

        // Add feedforward term to counteract gravity
        double feedforward = FEEDFORWARD_GAIN * Math.signum(krakenSetPoint - KrakenLeader.getPosition().getValueAsDouble());
        krakenOutput += feedforward;

        KrakenLeader.set(krakenOutput); // Apply the output to the leader motor

        SmartDashboard.putNumber("Kraken Leader Encoder Position", KrakenLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Kraken Setpoint", krakenSetPoint);

        if (KrakenLeader.getPosition().getValueAsDouble() > 0.0) {
            this.MAX_VALUE = 10;
        } else {
            this.MAX_VALUE = 0;
        }
    }

    public Command elevatorL1() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorL2() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorL3() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorL4() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorHome() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorHumanPlayer() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorNet() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    public Command elevatorFloor() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }
}
