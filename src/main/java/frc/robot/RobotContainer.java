// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Vision.VisionSubsystem;
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
import frc.robot.subsystems.Swerve.SwerveSubsystem;
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
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
/**
 * The RobotContainer class is where the bulk of the robot should be declared. 
 * Since Command-based is a "declarative" paradigm, very little robot logic 
 * should actually be handled in the Robot periodic methods (other than the 
 * scheduler calls). Instead, the structure of the robot (including subsystems, 
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public final elevator m_elevator = new elevator();
  public final Wrist m_wrist = new Wrist();
  public final claw m_claw = new claw();
  public final elbow m_elbow = new elbow();
  public final climber m_climber = new climber();
  public final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();
  public final VisionSubsystem m_vision = new VisionSubsystem();

  // Controllers
  public final CommandXboxController m_operatorController = new CommandXboxController(1);
  public final CommandXboxController m_driverController = new CommandXboxController(0);

  // Variables
  double a = 1;
  public double b = .6;

  // Autonomous chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize the autonomous chooser and add it to the Shuffleboard
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);

    if(RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    // Configure the trigger bindings
    configureBindings();
  }
  /**
    * This method binds the claw control to a button press.
    * When the A button is pressed, the claw stops. When B is pressed, the claw starts.
    */
    private void setClawControl() {

    }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} 
   * constructor with an arbitrary predicate, or via the named factories in 
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses 
   * for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller 
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick 
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Manual controls
   // Constants.operatorController.a().whileTrue(m_elevator.elevatorHome());
     m_operatorController.a().onTrue(m_wrist.vertical());
      m_operatorController.b().onTrue(m_wrist.horizontal());
      m_operatorController.x().onTrue(m_elbow.up());
      m_operatorController.y().onTrue(m_elbow.down());


    if(RobotBase.isSimulation()) {
      m_driverController.start().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3,3, new Rotation2d()))));
      m_driverController.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
    }
  }

  /**
   * Configures the PathPlanner for the drivebase subsystem.
   */
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

  /**
   * Sets the drive mode for the robot, applying deadbands and inverting controls 
   * based on the alliance color.
   */
  public void setDriveMode() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() != Alliance.Blue) {
      a = 1;
    } else {
      a = -1;
    }

    Command driveinfinityturn = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(a * b * Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(a * b * Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-.8 * Constants.driverController.getRightX(), .3));

    Command driveinfinityturn_sim = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(Constants.driverController.getRightX(), .3));

    m_drivebase.setDefaultCommand(
        RobotBase.isSimulation() ? driveinfinityturn_sim : driveinfinityturn);
  }

  /**
   * Sets the motor brake mode for the drivebase subsystem.
   *
   * @param brake whether to enable motor brake mode
   */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  /**
   * Sets the rumble detection based on the vision subsystem's latest result.
   */
  public void setRumbleDetection() {
    if (m_vision.getLatestResult().hasTargets()) {
      Constants.driverController.getHID().setRumble(RumbleType.kRightRumble, 1.0);
      Constants.driverController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      System.out.println(m_vision.getLatestResult().targets);
    }
  }


  public void updateVisionSimulationPeriodic() {
    m_vision.simulationPeriodic(m_drivebase.getPose());

    var debugField = m_vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(m_drivebase.getPose());
  }
  /**
   * Periodic method for drive simulation.
   */
  public void driveSimulationPeriodic() {
    m_drivebase.simulationPeriodic();
  }
}
