
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elbow.elbow;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class elevator extends SubsystemBase {
    private final TalonFX KrakenLeader = new TalonFX(14);
    private final TalonFX KrakenFollower = new TalonFX(13);
    // private PIDController pidController;
    public double elbowSetPoint = 0;
    private double errorSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;
    private double kP = .085;
    private double kI = 0.0;
    private double kD = .01;
    private double b = .04;
    private double hiLimit = 0.1; // Threshold for integral term
    // private final int CURRENT_LIMIT = 10; // Current limit in amps
    // private final double GRAVITY_FEEDFORWARD = 0.2; // Feedforward term to
    // counteract gravity
    public double positione;
    public boolean auto = true;
    public final elbow m_elbow = new elbow();
    boolean home=true;

    public elevator() {

        // SmartDashboard.setDefaultNumber("Target Position", 0);
        // SmartDashboard.setDefaultNumber("Target Velocity", 0);
        // SmartDashboard.setDefaultBoolean("Control Mode", false);
        // SmartDashboard.setDefaultBoolean("Reset Encoder", false);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 10;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.6; // Adjust the value as needed
        KrakenFollower.getConfigurator().apply(config);
        KrakenLeader.getConfigurator().apply(config);
        KrakenLeader.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        KrakenFollower.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        KrakenLeader.setPosition(0);
        KrakenFollower.setPosition(0);

    }

    public Command elevatorL1() {
        return runOnce(() -> {
            this.home=false;
            this.elbowSetPoint = -12.5;
            m_elbow.l1();
        });
    }

    public Command elevatorL2() {
        return runOnce(() -> {
            this.home=false;
            this.elbowSetPoint = -13.5;
        });
    }

    public Command elevatorL3() {
        return runOnce(() -> {
            this.home=false;
            this.elbowSetPoint = -15.5;
        });
    }

    public Command elevatorL4() {
        return runOnce(() -> {
            this.home=false;
            this.elbowSetPoint = -29.000;
        });
    }

    public Command elevatorSource() {

        return runOnce(() -> {
            this.home=false;
            this.elbowSetPoint = -7;
            // m_elbow.Human();
            m_elbow.human_auto();
        });
    }

    public Command Home() {
        return runOnce(() -> {
            this.home=true;
            this.elbowSetPoint = -0;
        
        });
    }

    public Command elevatorHighAlgae() {
        return runOnce(() -> {
            this.home=false;
            this.elbowSetPoint = -30.5;
        });
    }

    @Override
    public void periodic() {
        if (elbowSetPoint < -32.) {
            elbowSetPoint = 0;
        }
        if (elbowSetPoint > 0) {
            elbowSetPoint = 0;
        }
        if (home){
            kP=.085;
            kD = .01;
        }
        else{
            kP=.11;
            kD = .01;
        }
        positione = KrakenLeader.getPosition().getValueAsDouble();
        double error = elbowSetPoint - KrakenLeader.getPosition().getValueAsDouble();
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < hiLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;
        double output = kP * error + kI * errorSum + kD * errorRate;
        if (output < -.8) {
            output = -.8;
        }
        if (output > .2) {
            output = .2;
        }

        KrakenLeader.set((output - b));
        KrakenFollower.set((output - b));

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;

        SmartDashboard.putNumber("elevator", KrakenLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator2", KrakenFollower.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("output", output);
        // SmartDashboard.putNumber("motor amp", motor.getSupplyCurrent());
    }
}
