// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Wrist.wrist;
import frc.robot.subsystems.elevator.elevator;

import java.io.Console;

import java.util.Optional;

import frc.robot.subsystems.elevator.elevator;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Vision;
// import frc.robot.subsystems.LEDSubsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final elevator m_elevator =  new elevator();
  public final wrist m_wrist = new wrist();
  // The robot's subsystems and commands are defined here...

  double a =1;
  public double b=.6;

  public final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();
  public final VisionSubsystem m_vision = new VisionSubsystem();


  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // configurePathPlanner();
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    configureBindings(); // Configure the trigger bindings
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Manual controls
    Constants.operatorController.a().whileTrue(m_elevator.elevatorHome());
    


  }

  public void configurePathPlanner() {
    m_drivebase.setupPathPlanner();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation red positive blue
    // negitive

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() != Alliance.Blue) {
      a = 1;

    } else {
      a = -1;

    }

    Command driveinfinityturn = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband( a*b* Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband( a*b * Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-.8 * Constants.driverController.getRightX(), .3));

    Command driveinfinityturn_sim = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(Constants.driverController.getRightX(), .3));
    m_drivebase.setDefaultCommand(
        RobotBase.isSimulation() ? driveinfinityturn : driveinfinityturn);

  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  public void setRumbleDetection()
  {
    if (m_vision.getLatestResult().hasTargets()) {
      Constants.driverController.getHID().setRumble(RumbleType.kRightRumble, 1.0);
      Constants.driverController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      System.out.println(m_vision.getLatestResult().targets);
    } 
  }

  // public void updateVisionSimulationPeriod() {
  // m_vision.simulationPeriodic(m_drivebase.getPose());

  // var debugField = m_vision.getSimDebugField();
  // debugField.getObject("EstimatedRobot").setPose(m_drivebase.getPose());
  // //
  // debugField.getObject("EstimatedRobotModules").setPoses(m_drivebase.getModulePoses());
  // }
 

  public void driveSimulationPeriodic() {
    m_drivebase.simulationPeriodic();
  }

}
