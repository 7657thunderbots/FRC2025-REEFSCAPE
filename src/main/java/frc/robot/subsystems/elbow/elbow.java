package frc.robot.subsystems.elbow;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class elbow extends SubsystemBase {
    private SparkMax motor;
    // private SparkMaxConfig motorConfig;
    // private SparkClosedLoopController closedLoopController;
    public AbsoluteEncoder encoder;
    // private PIDController pidController;
    public double elbowSetPoint = .584;
    private double errorSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;
    private double kP = 1.7;
    private double kI = 0.01;
    private double kD = 0.1;
    // private double b;
    private double hiLimit = 0.01; // Threshold for integral term
    public boolean safeL1;
    private boolean L1;
    // private final int CURRENT_LIMIT = 10; // Current limit in amps

    // In constructor, add:

    public elbow() {
        motor = new SparkMax(16, MotorType.kBrushless);

        encoder = motor.getAbsoluteEncoder();
        // encoder = motor.getEncoder();

        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
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
    }

    // Desired position

    public void human_auto() {
         this.elbowSetPoint = .57;
       //this.elbowSetPoint=.6;
    }

    public Command up() {
        return runOnce(() -> {
            this.L1 = false;
            this.elbowSetPoint = .584;
        });
    }

    public Command down() {
        return runOnce(() -> {
            this.L1 = false;
            this.elbowSetPoint = .76;
        });
    }

    // public Command L2L3(){
    // return runOnce(() ->{
    // l
    // this.elbowSetPoint=.6365;
    // }); }
    public Command Human() {
        return runOnce(() -> {
            this.L1 = false;
            this.elbowSetPoint = .57;
            // System.out.println("Human elbow");
        });
    }

    public Command l1() {
        return runOnce(() -> {

            this.L1 = true;
        });
    }

    private enum ElbowState {
        STOPPED,
        INTAKING
    }

    private ElbowState currentState = ElbowState.INTAKING;

    public Command toggleState() {
        return Commands.runOnce(() -> {
            switch (currentState) {
                case STOPPED:
                    this.L1 = false;
                    this.elbowSetPoint = .584;
                    currentState = ElbowState.INTAKING;
                    break;
                case INTAKING:
                    this.L1 = false;
                    this.elbowSetPoint = .76;
                    currentState = ElbowState.STOPPED;
                    break;
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
        if (safeL1 && L1) {
            this.elbowSetPoint = .9;
        }
        SmartDashboard.putString("Elbow Current state", currentState.toString());

        double error = elbowSetPoint - encoder.getPosition();
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < hiLimit) {
            errorSum += error * dt;
        }
        // if (Math.abs(error) < 0.05 && elbowSetPoint == .76) {
        // kP = 5;

        // // Increased P value
        // } else {
        // kP = 1.9; // Original P value
        // }

        double errorRate = (error - lastError) / dt;
        double output = kP * error + kI * errorSum + kD * errorRate;

        motor.set(-(output - .025));

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;

        SmartDashboard.putNumber("elbow Position", encoder.getPosition());
        SmartDashboard.putNumber("motor amp", motor.getOutputCurrent());
    }
}