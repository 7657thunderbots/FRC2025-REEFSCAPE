package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;


public class elevator extends SubsystemBase {

    private final TalonFX Leader = new TalonFX(1);
    private final TalonFX Follower = new TalonFX(2);
    private PIDController pidController;
    private double setpoint;
  
  
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

  
    private int printCount = 0;
  
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */


    public elevator() {
        pidController = new PIDController(0., 0.0, 0.00); // Kp, Ki, Kd
        setpoint = 000; // Desired position
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();
    
        /* User can optionally change the configs or leave it alone to perform a factory default */
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        Leader.getConfigurator().apply(leftConfiguration);
        Follower.getConfigurator().apply(leftConfiguration);
    
        /* Set up followers to follow leaders */
        Follower.setControl(new Follower(Leader.getDeviceID(), true));
      
        Leader.setSafetyEnabled(true);

    }
    @Override
    public void periodic()
    {
         BaseStatusSignal.setUpdateFrequencyForAll(100,
            Leader.getPosition());
            Leader.getPosition().getValue();
            

         double error = setpoint - currentPosition;
         double output = pidController.calculate(currentPosition, setpoint);
         Leader.set( output); // Apply the output to the leader motor

        // Update SmartDashboard
        SmartDashboard.putNumber("Leader Encoder Position", Leader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Setpoint", setpoint);
        // SmartDashboard.putNumber("PID Output", output);
    }
    public StatusSignal<Angle> getLeftPos() {
        return Leader.getPosition();
        
    }


}