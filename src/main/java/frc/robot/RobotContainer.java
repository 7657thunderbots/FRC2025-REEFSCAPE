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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
//import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.claw.claw;
import frc.robot.subsystems.elbow.elbow;
import frc.robot.subsystems.climber.climber;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


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

  private final SendableChooser<Command> autoChooser;

  //here
  public RobotContainer()
  {
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    configureBindings();
  }



  //Here

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
  m_operatorController.button(10).onTrue(m_elbow.toggleState());
  m_operatorController.y().onTrue(m_elevator.elevatorL2());
  m_operatorController.button(1).onTrue(m_elevator.elevatorL3());
  m_operatorController.rightBumper().onTrue(m_elevator.elevatorL1());
  m_operatorController.rightBumper().onTrue(m_elbow.l1());
  m_operatorController.leftBumper().onTrue(m_elevator.elevatorSource());
  m_operatorController.b().onTrue(m_elevator.elevatorHighAlgae());
  m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  m_operatorController.y().onTrue(m_elbow.L2L3());
  m_operatorController.button(1).onTrue(m_elbow.L2L3());
  m_operatorController.leftBumper().onTrue(m_elbow.Human());
  m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  m_operatorController.button(8).onTrue(m_elevator.Home());
  m_operatorController.button(8).onTrue(m_elbow.up());

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
   
    if (drivebase.findClosestAprilTag() == 19){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.630, 5.088), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.927, 5.309), Rotation2d.fromDegrees(-17.943))));

    }
    if (drivebase.findClosestAprilTag() == 20){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(5.019, 5.272), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(5.294, 5.088), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 21){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(5.813, 4.181), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(5.841, 3.827), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 22){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(5.294, 2.990), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(5.019, 2.806), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 17){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.956, 2.806), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.673, 2.976), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 18){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.177, 4.167), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(3.177, 3.855), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 12){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(1.816, 0.595), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(1.023, 1.162), Rotation2d.fromDegrees(-17.943))));}
    if  (drivebase.findClosestAprilTag() == 13){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(1.632, 7.313), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(0.782, 6.704), Rotation2d.fromDegrees(-17.943))));}

    
    if (drivebase.findClosestAprilTag() == 7){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(14.380, 3.852), Rotation2d.fromDegrees(-26.565))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(14.395, 4.168), Rotation2d.fromDegrees(-26.565))));}
    if (drivebase.findClosestAprilTag() == 8){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(13.869, 5.099), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(13.583, 5.265), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 9){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(12.516, 5.280), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(12.201, 5.099), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 10){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(11.735, 4.183), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(11.750, 3.852), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 11){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(12.231, 2.966), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(12.546, 2.815), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 6){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(13.568, 2.755), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(13.869, 2.951), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 1){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(16.844, 1.358), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(15.942, 0.682), Rotation2d.fromDegrees(-17.943))));}
    if (drivebase.findClosestAprilTag() == 2){
      driverXbox.leftTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(16.799, 6.704), Rotation2d.fromDegrees(-17.943))));
      driverXbox.rightTrigger().onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(15.897, 7.353), Rotation2d.fromDegrees(-17.943))));}
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
  //SmartDashboard.putNumber("closest tag",m_vision.getAprilTagIdEvenIfNotVisible());
}