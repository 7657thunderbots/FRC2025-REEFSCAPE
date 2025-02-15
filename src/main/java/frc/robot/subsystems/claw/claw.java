package frc.robot.subsystems.claw;

import java.security.PrivilegedActionException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class claw extends SubsystemBase {

    public double getCurrentVelocity() {
        return encoder.getVelocity();
    }
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
   
    public claw() {
    motor = new SparkMax(1, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    
    
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    public void intake_in() {
    if (getCurrentVelocity() == 0) {
        motor.set(0.5); // Run the motor to shoot out the game piece
    } else {
        motor.set(0); // Stop the motor
    }
    }
    public void intake_out() {
        double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
        if (getCurrentVelocity() >= targetVelocity) {
            motor.set(0); // Stop the motor when the target velocity is reached
        } else {
            motor.set(-0.5); // Run the motor to intake the game piece
        }
    }
    private boolean hasPiece = false;
    private boolean isRunning = false;

  
    
    //  public Command piviotspeakerfar(){
    public Command controlPiece() {
        return new Command() {
            private final double TARGET_RELEASE_VELOCITY = -2000; // Adjust this value as needed

            @Override
            public void initialize() {
                isRunning = true;
                if (hasPiece) {
                    intake_out();
                } else {
                    intake_in();
                }
            }

            @Override
            public void end(boolean interrupted) {
                motor.set(0);
                isRunning = false;
                hasPiece = !hasPiece;
            }
            
            @Override
            public boolean isFinished() {
                if (hasPiece) {
                    // When releasing, stop at target velocity
                    return !isRunning || getCurrentVelocity() <= TARGET_RELEASE_VELOCITY;
                } else {
                    // When intaking, stop when velocity reaches zero
                    return !isRunning || getCurrentVelocity() == 0;
                }
            }
        };
    }
    //     return runOnce(() -> {
    //        this.piviotsetpoint = 600;
    //     });
    // }

   

    // public Command manualmove(){

    // }

   

    @Override
    public void periodic()
    {}
       

}
