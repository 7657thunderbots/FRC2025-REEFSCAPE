// package frc.robot.subsystems.climber;

// import java.security.PrivilegedActionException;

// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class climber extends SubsystemBase {
//     private SparkMax motor;
//     private SparkMaxConfig motorConfig;
//     private SparkClosedLoopController closedLoopController;
//     private AbsoluteEncoder encoder;
//     Servo Servo = new Servo(0);
//     double speed;

//     public climber() {
//     //motor = new SparkMax(25, MotorType.kBrushless);
//     // closedLoopController = motor.getClosedLoopController();
//     // encoder = motor.getAbsoluteEncoder();
//    // Servo.setAngle(23);
    
    
//     /*
//      * Create a new SPARK MAX configuration object. This will store the
//      * configuration parameters for the SPARK MAX that we will set below.
//      */
//     // motorConfig = new SparkMaxConfig();

//     /*
//      * Configure the encoder. For this specific example, we are using the
//      * integrated encoder of the NEO, and we don't need to configure it. If
//      * needed, we can adjust values like the position or velocity conversion
//      * factors.
//      */
//     // motorConfig.encoder
//     //     .positionConversionFactor(1)
//     //     .velocityConversionFactor(1);

//     /*
//      * Configure the closed loop controller. We want to make sure we set the
//      * feedback sensor as the primary encoder.
//      */
//     // motorConfig.closedLoop
//     //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//     //     // Set PID values for position control. We don't need to pass a closed loop
//     //     // slot, as it will default to slot 0.
//     //     .p(0.1)
//     //     .i(0)
//     //     .d(0)
//     //     .outputRange(-1, 1)
//     //     // Set PID values for velocity control in slot 1
//     //     .p(0.0001, ClosedLoopSlot.kSlot1)
//     //     .i(0, ClosedLoopSlot.kSlot1)
//     //     .d(0, ClosedLoopSlot.kSlot1)
//     //     .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
//     //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        
      

//     /*
//      * Apply the configuration to the SPARK MAX.
//      *
//      * kResetSafeParameters is used to get the SPARK MAX to a known state. This
//      * is useful in case the SPARK MAX is replaced.
//      *
//      * kPersistParameters is used to ensure the configuration is not lost when
//      * the SPARK MAX loses power. This is useful for power cycles that may occur
//      * mid-operation.
//      */
//    // motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//         motor = new SparkMax(25, MotorType.kBrushless);
//        // closedLoopController = motor.getClosedLoopController();
//        // encoder = motor.getAbsoluteEncoder();
//         //Servo.setAngle(23);

//         /*
//          * Create a new SPARK MAX configuration object. This will store the
//          * configuration parameters for the SPARK MAX that we will set below.
//          */
//         // motorConfig = new SparkMaxConfig();

//         /*
//          * Configure the encoder. For this specific example, we are using the
//          * integrated encoder of the NEO, and we don't need to configure it. If
//          * needed, we can adjust values like the position or velocity conversion
//          * factors.
//          */
//         // motorConfig.encoder
//         //         .positionConversionFactor(1)
//         //         .velocityConversionFactor(1);

//         /*
//          * Configure the closed loop controller. We want to make sure we set the
//          * feedback sensor as the primary encoder.
//          */
//         // motorConfig.closedLoop
//         //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         //         // Set PID values for position control. We don't need to pass a closed loop
//         //         // slot, as it will default to slot 0.
//         //         .p(0.1)
//         //         .i(0)
//         //         .d(0)
//         //         .outputRange(-1, 1)
//         //         // Set PID values for velocity control in slot 1
//         //         .p(0.0001, ClosedLoopSlot.kSlot1)
//         //         .i(0, ClosedLoopSlot.kSlot1)
//         //         .d(0, ClosedLoopSlot.kSlot1)
//         //         .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
//         //         .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

//         /*
//          * Apply the configuration to the SPARK MAX.
//          *
//          * kResetSafeParameters is used to get the SPARK MAX to a known state. This
//          * is useful in case the SPARK MAX is replaced.
//          *
//          * kPersistParameters is used to ensure the configuration is not lost when
//          * the SPARK MAX loses power. This is useful for power cycles that may occur
//          * mid-operation.
//          */
//         // motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//         // Initialize dashboard values
//         // SmartDashboard.setDefaultNumber("Target Position", 0);
//         // SmartDashboard.setDefaultNumber("Target Velocity", 0);
//         // SmartDashboard.setDefaultBoolean("Control Mode", false);
//         // SmartDashboard.setDefaultBoolean("Reset Encoder", false);

//     }

//     // public void stop() {
//     // piviot.set(0);
//     // }

//     // public Command stopPiviot(){
//     // return runOnce(() -> {
//     // this.stop();
//     // });
//     // }

//     public Command up() {
//         return run(() -> {

//            // if (encoder.getPosition()<.8){
//                // speed=1;
//                 // Servo.setAngle(0);
//             // if (encoder.getPosition()<.8){
//            speed = 1;
//             // Servo.setAngle(0);
//             // }
//             // else{
//             // speed =0;
//             // }
//         });
//     }

//     public Command down() {
//         return run(() -> {

//             // if (encoder.getPosition()>.5 && Servo.getAngle()>16){
//                 speed=-1;
//                 // Servo.setAngle(23);
//             // }
//             // else{
//                 // speed=0;
//                 // Servo.setAngle(23);
//             //
//             // if (encoder.getPosition() > .5 && Servo.getAngle() > 16) {
//             //     speed = -.1;
//             //     Servo.setAngle(23);
//             // } else {
//             //     speed = 0;
//             //     Servo.setAngle(23);
//             // }
//         });
//     }

//     // public Command manualmove(){

//     // }

//     @Override
//     public void periodic() {
//         motor.set(speed);
//         speed = 0;
//         SmartDashboard.getNumber("servo", Servo.getAngle());

//     }

// }
