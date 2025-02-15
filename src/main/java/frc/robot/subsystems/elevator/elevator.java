/**
 * This package contains the classes and interfaces for the elevator subsystem of the FRC 2025 REEFSCAPE robot.
 * The elevator subsystem is responsible for controlling the vertical movement of the robot's elevator mechanism.
 */
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

/**
 * The elevator class represents the elevator subsystem of the robot.
 * It extends SubsystemBase and provides methods to control the elevator mechanism.
 */
public class elevator extends SubsystemBase {

    /**
     * The maximum value for the elevator position.
     */
    private double MAX_VALUE;

    /**
     * The setpoint for the elevator position.
     */
    private double elevatorSetPoint;

    /**
     * The leader motor controller for the elevator.
     */
    private final TalonFX KrakenLeader = new TalonFX(14);

    /**
     * The follower motor controller for the elevator.
     */
    private final TalonFX KrakenFollower = new TalonFX(13);

    /**
     * The PID controller for the elevator.
     */
    private PIDController krakenPidController;

    /**
     * The setpoint for the Kraken PID controller.
     */
    public double krakenSetPoint;

    /**
     * The duty cycle output for the left Kraken motor.
     */
    private final DutyCycleOut krakenLeftOut = new DutyCycleOut(0);

    /**
     * The duty cycle output for the right Kraken motor.
     */
    private final DutyCycleOut krakenRightOut = new DutyCycleOut(0);

    /**
     * Initializes the Kraken motors and their configurations.
     */
    public void initializeKraken() {
        krakenPidController = new PIDController(0.0, 0.0, 0.00); // Kp, Ki, Kd
        krakenSetPoint = 0; // Desired position
        var krakenLeftConfiguration = new TalonFXConfiguration();
        var krakenRightConfiguration = new TalonFXConfiguration();

        /* User can optionally change the configs or leave it alone to perform a factory default */
        krakenLeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        krakenRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Configure current limits
        krakenRightConfiguration.CurrentLimits.SupplyCurrentLimit = Double.MAX_VALUE;
        krakenRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        KrakenLeader.getConfigurator().apply(krakenLeftConfiguration);
        KrakenFollower.getConfigurator().apply(krakenRightConfiguration);

        /* Set up followers to follow leaders */
        KrakenFollower.setControl(new Follower(KrakenLeader.getDeviceID(), true));
        KrakenLeader.setSafetyEnabled(true);
    }

    /**
     * Controls the Kraken motors based on the PID controller output.
     */
    public void controlKraken() {
        double krakenError = krakenSetPoint - KrakenLeader.getPosition().getValueAsDouble();
        double krakenOutput = krakenPidController.calculate(KrakenLeader.getPosition().getValueAsDouble(), krakenSetPoint);
        KrakenLeader.set(krakenOutput); // Apply the output to the leader motor

        // Update SmartDashboard
        SmartDashboard.putNumber("Kraken Leader Encoder Position", KrakenLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Kraken Setpoint", krakenSetPoint);

        if (KrakenLeader.getPosition().getValueAsDouble() > 0.0) {
            this.MAX_VALUE = 10;
        } else {
            this.MAX_VALUE = 0;
        }
    }

    /**
     * Command to move the elevator to Level 1 position.
     * @return A command that sets the elevator setpoint for Level 1
     */
    public Command elevatorL1() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to Level 2 position.
     * @return A command that sets the elevator setpoint for Level 2
     */
    public Command elevatorL2() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to Level 3 position.
     * @return A command that sets the elevator setpoint for Level 3
     */
    public Command elevatorL3() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to Level 4 position.
     * @return A command that sets the elevator setpoint for Level 4
     */
    public Command elevatorL4() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to the home position.
     * @return A command that sets the elevator setpoint for the home position
     */
    public Command elevatorHome() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to the human player position.
     * @return A command that sets the elevator setpoint for the human player position
     */
    public Command elevatorHumanPlayer() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to the net position.
     * @return A command that sets the elevator setpoint for the net position
     */
    public Command elevatorNet() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }

    /**
     * Command to move the elevator to the floor position.
     * @return A command that sets the elevator setpoint for the floor position
     */
    public Command elevatorFloor() {
        return runOnce(() -> {
            this.elevatorSetPoint = 0; // change later
        });
    }
}


