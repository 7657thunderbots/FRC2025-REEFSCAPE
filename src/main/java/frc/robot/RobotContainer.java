// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import swervelib.SwerveInputStream;
//import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.claw.claw;
import frc.robot.subsystems.elbow.elbow;
import frc.robot.subsystems.climber.climber;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.OperatorConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Subsystems
  public final elevator m_elevator = new elevator();
  public final Wrist m_wrist = new Wrist();
  public final claw m_claw = new claw();
  public final elbow m_elbow = new elbow();
  public final climber m_climber = new climber(); //public final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();
  //public final Vision m_vision = new vision();
   // Controllers
  public final CommandXboxController m_operatorController = new CommandXboxController(1);
  public final CommandXboxController m_driverController = new CommandXboxController(0);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(.1)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
  m_operatorController.x().onTrue(m_elevator.elevatorL4());
  m_operatorController.back().onTrue(m_wrist.toggle());
  m_operatorController.button(9).onTrue(m_claw.toggleState());
  m_operatorController.button(10).onTrue(m_elbow.down());
  m_operatorController.y().onTrue(m_elevator.elevatorL2());
  m_operatorController.button(1).onTrue(m_elevator.elevatorL3());
  m_operatorController.rightBumper().onTrue(m_elevator.elevatorL1());
  m_operatorController.leftBumper().onTrue(m_elevator.elevatorSource());
  m_operatorController.b().onTrue(m_elevator.elevatorHighAlgae());
  m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  m_operatorController.y().onTrue(m_elbow.L2L3());
  m_operatorController.button(1).onTrue(m_elbow.L2L3());
  m_operatorController.leftBumper().onTrue(m_elbow.Human());
  m_operatorController.button(8).onTrue(m_elevator.Home());

  // Return elevator to home when no elevator buttons are pressed
  // new Trigger(() -> !m_operatorController.x().getAsBoolean() &&
  //                   !m_operatorController.y().getAsBoolean() &&
  //                   !m_operatorController.b().getAsBoolean() &&
  //                   !m_operatorController.button(3).getAsBoolean() &&
  //                   !m_operatorController.rightBumper().getAsBoolean() &&
  //                   !m_operatorController.leftBumper().getAsBoolean())
  //     .onTrue(m_elevator.Home());


    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
     // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
    // Path Planner commands
    // driverXbox.povUp().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0))));
    // driverXbox.povRight().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 2), Rotation2d.fromDegrees(90))));
    // driverXbox.povDown().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(2, 4), Rotation2d.fromDegrees(180))));
   driverXbox.povLeft().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.630, 5.088), Rotation2d.fromDegrees(-17.943))));
    if (SwerveSubsystem.vision.getAprilTagIdEvenIfNotVisible()==1){
    }
    }
   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  //SmartDashboard.putNumber("closest tag",m_vision.getAprilTagIdEvenIfNotVisible());
}









































































































































// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import frc.robot.subsystems.Vision.VisionSubsystem;
// import frc.robot.subsystems.Wrist.Wrist;
// import frc.robot.subsystems.elevator.elevator;
// import frc.robot.subsystems.claw.claw;
// import frc.robot.subsystems.elbow.elbow;
// import frc.robot.subsystems.climber.climber;

// import java.util.Optional;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import frc.robot.subsystems.Swerve.SwerveSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.Constants.OperatorConstants;

// /**
//  * This class is where the bulk of the robot should be declared. Since
//  * Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in
//  * the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of
//  * the robot (including
//  * subsystems, commands, and trigger mappings) should be declared here.
//  */
// /**
//  * The RobotContainer class is where the bulk of the robot should be declared. 
//  * Since Command-based is a "declarative" paradigm, very little robot logic 
//  * should actually be handled in the Robot periodic methods (other than the 
//  * scheduler calls). Instead, the structure of the robot (including subsystems, 
//  * commands, and button mappings) should be declared here.
//  */
// public class RobotContainer {

//   // Subsystems
//   public final elevator m_elevator = new elevator();
//   public final Wrist m_wrist = new Wrist();
//   public final claw m_claw = new claw();
//   public final elbow m_elbow = new elbow();
//   public final climber m_climber = new climber();
//   public final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();
//   public final VisionSubsystem m_vision = new VisionSubsystem();

//   // Controllers
//   public final CommandXboxController m_operatorController = new CommandXboxController(1);
//   public final CommandXboxController m_driverController = new CommandXboxController(0);

//   // Variables
//   double a = 1;
//   public double b = .6;

//   // Autonomous chooser
//   private final SendableChooser<Command> autoChooser;

//   /**
//    * The container for the robot. Contains subsystems, OI devices, and commands.
//    */
//   public RobotContainer() {
//     // Initialize the autonomous chooser and add it to the Shuffleboard
//     autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
//     Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);

//     if(RobotBase.isSimulation()) {
//       DriverStation.silenceJoystickConnectionWarning(true);
//     }
//     // Configure the trigger bindings
//     configureBindings();
//   }
//   /**
//     * This method binds the claw control to a button press.
//     * When the A button is pressed, the claw stops. When B is pressed, the claw starts.
//     */
//     private void setClawControl() {

//     }
//   /**
//    * Use this method to define your trigger->command mappings. Triggers can be
//    * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} 
//    * constructor with an arbitrary predicate, or via the named factories in 
//    * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses 
//    * for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller 
//    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick 
//    * Flight joysticks}.
//    */
//   private void configureBindings() {
//     // Manual controls
//    // Constants.operatorController.a().whileTrue(m_elevator.elevatorHome());
//      m_operatorController.a().onTrue(m_wrist.vertical());
//       m_operatorController.b().onTrue(m_wrist.horizontal());
//       m_operatorController.x().onTrue(m_elbow.up());
//       m_operatorController.y().onTrue(m_elbow.down());


//     if(RobotBase.isSimulation()) {
//       m_driverController.start().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3,3, new Rotation2d()))));
//       m_driverController.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
//     }
//   }

//   /**
//    * Configures the PathPlanner for the drivebase subsystem.
//    */
//   public void configurePathPlanner() {
//     m_drivebase.setupPathPlanner();
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     return autoChooser.getSelected();
//   }

//   /**
//    * Sets the drive mode for the robot, applying deadbands and inverting controls 
//    * based on the alliance color.
//    */
//   public void setDriveMode() {
//     Optional<Alliance> ally = DriverStation.getAlliance();
//     if (ally.isPresent() && ally.get() != Alliance.Blue) {
//       a = 1;
//     } else {
//       a = -1;
//     }

//     Command driveinfinityturn = m_drivebase.driveCommand(
//         () -> MathUtil.applyDeadband(a * b * Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
//         () -> MathUtil.applyDeadband(a * b * Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
//         () -> MathUtil.applyDeadband(-.8 * Constants.driverController.getRightX(), .3));

//     Command driveinfinityturn_sim = m_drivebase.driveCommand(
//         () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
//         () -> -MathUtil.applyDeadband(Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
//         () -> MathUtil.applyDeadband(Constants.driverController.getRightX(), .3));

//     m_drivebase.setDefaultCommand(
//         RobotBase.isSimulation() ? driveinfinityturn_sim : driveinfinityturn);
//   }

//   /**
//    * Sets the motor brake mode for the drivebase subsystem.
//    *
//    * @param brake whether to enable motor brake mode
//    */
//   public void setMotorBrake(boolean brake) {
//     m_drivebase.setMotorBrake(brake);
//   }

//   /**
//    * Sets the rumble detection based on the vision subsystem's latest result.
//    */
//   public void setRumbleDetection() {
//     if (m_vision.getLatestResult().hasTargets()) {
//       Constants.driverController.getHID().setRumble(RumbleType.kRightRumble, 1.0);
//       Constants.driverController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
//       System.out.println(m_vision.getLatestResult().targets);
//     }
//   }


//   public void updateVisionSimulationPeriodic() {
//     m_vision.simulationPeriodic(m_drivebase.getPose());

//     var debugField = m_vision.getSimDebugField();
//     debugField.getObject("EstimatedRobot").setPose(m_drivebase.getPose());
//   }
//   /**
//    * Periodic method for drive simulation.
//    */
//   public void driveSimulationPeriodic() {
//     m_drivebase.simulationPeriodic();
//   }
// }
