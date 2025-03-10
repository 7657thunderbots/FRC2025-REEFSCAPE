// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import frc.robot.subsystems.elevator.elevator;
import swervelib.SwerveInputStream;
//import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.claw.claw;
import frc.robot.subsystems.elbow.elbow;
//import frc.robot.subsystems.climber.climber;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
 // public final climber m_climber = new climber(); //public final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();
  //public final Vision m_vision = new vision();
   // Controllers
  public final CommandXboxController m_operatorController = new CommandXboxController(1);
  //public final CommandXboxController m_driverController = new CommandXboxController(0);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  //final         CommandXboxController drivebase.driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> drivebase.driverXbox.getLeftY() * -1,
  () -> drivebase.driverXbox.getLeftX() * -1)
  .withControllerRotationAxis(() -> drivebase.driverXbox.getRightX() * -1)
  .deadband(.2)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(drivebase.driverXbox::getRightX,
  drivebase.driverXbox::getRightY)
  .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
  .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> -drivebase.driverXbox.getLeftY(),
  () -> -drivebase.driverXbox.getLeftX())
  .withControllerRotationAxis(() -> drivebase.driverXbox.getRawAxis(
  2))
  .deadband(.05)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
  .withControllerHeadingAxis(() ->
  Math.sin(
  drivebase.driverXbox.getRawAxis(
  2) *
  Math.PI) *
  (Math.PI *
  2),
  () ->
  Math.cos(
  drivebase.driverXbox.getRawAxis(
  2) *
  Math.PI) *
  (Math.PI *2))
  .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private final SendableChooser<Command> autoChooser;

  //here
  public RobotContainer()
  {

    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    configureBindings();
    //SmartDashboard.putNumber("elevator in container", m_elevator.positione);
    SmartDashboard.putBoolean("elbow safe", m_elbow.safeL1);

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
    
  drivebase.LeftTrigger = drivebase.driverXbox.leftTrigger();
  drivebase.RightTrigger = drivebase.driverXbox.rightTrigger();
  m_operatorController.x().onTrue(m_elevator.elevatorL4());
  m_operatorController.x().onTrue(m_elbow.up());
  m_operatorController.back().onTrue(m_wrist.toggle());
  m_operatorController.button(9).whileTrue(m_claw.outtake());
  m_operatorController.button(11).whileTrue(m_claw.intake());
  m_operatorController.button(10).onTrue(m_elbow.toggleState());
 m_operatorController.y().onTrue(m_elevator.elevatorL2());
 m_operatorController.button(1).onTrue(m_elevator.elevatorL3());
  m_operatorController.rightBumper().onTrue(m_elevator.elevatorL1());
  m_operatorController.rightBumper().onTrue(m_elbow.l1());
  m_operatorController.leftBumper().onTrue(m_elevator.Home());
 m_operatorController.b().onTrue(m_elevator.elevatorHighAlgae());
  m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  m_operatorController.y().onTrue(m_elbow.up());
  m_operatorController.button(1).onTrue(m_elbow.up());
  m_operatorController.leftBumper().onTrue(m_elbow.up());
  m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  m_operatorController.button(8).onTrue(m_elevator.elevatorSource());
  m_operatorController.button(8).onTrue(m_elbow.Human());
  // *******home is in robot.java**********


  //m_operatorController.button(2).onFalse(drivebase.driveToPose(new Pose2d(new Translation2d(3.177, 4.167), Rotation2d.fromDegrees(0))));

  // Return elevator to home when no elevator buttons are pressed
  // new Trigger(() -> !m_operatorController.x().getAsBoolean() &&
  //                   !m_operatorController.y().getAsBoolean() &&
  //                   !m_operatorController.b().getAsBoolean() &&
  //                   !m_operatorController.button(3).getAsBoolean() &&
  //                   !m_operatorController.rightBumper().getAsBoolean() &&
  //                   !m_operatorController.leftBumper().getAsBoolean())
  //     .onTrue(m_elevator.Home());


    // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        // driveDirectAngle);
    // Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        // driveDirectAngleKeyboard);

    // if (RobotBase.isSimulation())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else
    // {
    if (drivebase.driverXbox.getLeftTriggerAxis()<.1 && drivebase.driverXbox.getRightTriggerAxis()<.1)
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      //drivebase.driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //drivebase.driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    //   drivebase.driveToPose(new Pose2d(new Translation2d(3.177, 4.167), Rotation2d.fromDegrees(0)));
     }
    if (DriverStation.isTest())
    {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      drivebase.driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      drivebase.driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      drivebase.driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      drivebase.driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      drivebase.driverXbox.leftBumper().onTrue(Commands.none());
      drivebase.driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
     // drivebase.driverXbox.button(2).onTrue((Commands.runOnce(drivebase::zeroGyro)));
     // drivebase.driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // drivebase.driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      drivebase.driverXbox.start().whileTrue(Commands.none());
      drivebase.driverXbox.back().whileTrue(Commands.none());
      drivebase.driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      drivebase.driverXbox.rightBumper().onTrue(Commands.none());
    }
    // Path Planner commands
    // drivebase.driverXbox.povUp().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0))));
    // drivebase.driverXbox.povRight().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 2), Rotation2d.fromDegrees(90))));
    // drivebase.driverXbox.povDown().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(2, 4), Rotation2d.fromDegrees(180))));

  }
  // public void configurePathPlanner(){
  //   drivebase.setupPathPlanner();
  //  // NamedCommands.registerCommand("roller", System.out.println("I Work"));
  //   }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return   autoChooser.getSelected();
  }


  //here
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}