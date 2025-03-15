package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    // private SparkClosedLoopController closedLoopController;
    public AbsoluteEncoder encoder;
    // private PIDController pidController;
    public double wristSetPoint = .264;
    private double errorSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;
    private double kP = 4;
    private double kI = 0.0;
    private double kD = 0.00;
    private double hiLimit = 0.1; // Threshold for integral term
    private final int CURRENT_LIMIT = 10; // Current limit in amps

    // In constructor, add:

    public Wrist() {
        motor = new SparkMax(17, MotorType.kBrushless);
        // closedLoopController = motor.getClosedLoopController();

        encoder = motor.getAbsoluteEncoder();

        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(CURRENT_LIMIT);

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        // motorConfig.encoder
        // .positionConversionFactor(1)
        // .velocityConversionFactor(1);

        // /*
        // * Configure the closed loop controller. We want to make sure we set the
        // * feedback sensor as the primary encoder.
        // */
        // motorConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // // Set PID values for position control. We don't need to pass a closed loop
        // // slot, as it will default to slot 0.
        // .p(0.1)
        // .i(0)
        // .d(0)
        // .outputRange(-1, 1)
        // // Set PID values for velocity control in slot 1
        // .p(0.0001, ClosedLoopSlot.kSlot1)
        // .i(0, ClosedLoopSlot.kSlot1)
        // .d(0, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

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

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    // Desired position

    public Command horizontal() {
        return runOnce(() -> {
            this.wristSetPoint = .264;
        });
    }

    public Command vertical() {
        return runOnce(() -> {
            this.wristSetPoint = .507;
        });
    }

    private boolean isVertical = false;

    public Command toggle() {
        return runOnce(() -> {
            if (isVertical) {
                isVertical = false;
                wristSetPoint = .264; // horizontal
            } else {
                isVertical = true;
                wristSetPoint = .507; // vertical
            }
        });
    }
    // public void stop() {
    // piviot.set(0);
    // }

    // public Command stopPiviot(){
    // return runOnce(() -> {
    // this.stop();
    // });
    // }

    @Override
    public void periodic() {

        double error = wristSetPoint - encoder.getPosition();
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < hiLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;
        double output = kP * error + kI * errorSum + kD * errorRate;

        motor.set(output);

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;

        SmartDashboard.putNumber("Wrist Position", encoder.getPosition());
        SmartDashboard.putNumber("motor amp", motor.getOutputCurrent());
    }
}