// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.led.LED;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private LED m_ledSubsystem; // declared this here as I don't want to make it command based

  // private AddressableLED m_led;
  // private AddressableLEDBuffer m_ledBuffer;
  // LEDPattern gradient =
  // LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrange,
  // Color.kBlue);

  // // Our LED strip has a density of 120 LEDs per meter
  // private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip,
  // moving at a speed
  // of 1 meter per second.
  // private final LEDPattern m_scrollingRainbow =
  // m_rainbow.scrollAtAbsoluteSpeed(InchesPerSecond.of(1), kLedSpacing);

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // set data
    // m_led.setData(m_ledBuffer);
    // m_led.start();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_ledSubsystem = new LED(0, 60);

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (m_robotContainer.m_elevator.positione < -8) {
      m_robotContainer.m_elevator.m_elbow.safeL1 = true;
    } else {
      m_robotContainer.m_elevator.m_elbow.safeL1 = false;
    }

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // System.gc();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_elevator.auto = true;
    m_robotContainer.m_claw.auto = true;
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.m_claw.auto = false;
    m_robotContainer.m_elevator.auto = false;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  // m_operatorController.x().onTrue(m_elevator.elevatorL4());
  // m_operatorController.back().onTrue(m_wrist.toggle());
  // m_operatorController.button(9).onTrue(m_claw.toggleState());
  // m_operatorController.button(10).onTrue(m_elevator.m_elbow.toggleState());
  // m_operatorController.y().onTrue(m_elevator.elevatorL2());
  // m_operatorController.button(1).onTrue(m_elevator.elevatorL3());
  // m_operatorController.rightBumper().onTrue(m_elevator.elevatorL1());
  // m_operatorController.rightBumper().onTrue(m_elevator.m_elbow.l1());
  // m_operatorController.leftBumper().onTrue(m_elevator.elevatorSource());
  // m_operatorController.b().onTrue(m_elevator.elevatorHighAlgae());
  // m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  // m_operatorController.y().onTrue(m_elevator.m_elbow.up());
  // m_operatorController.button(1).onTrue(m_elevator.m_elbow.up());
  // m_operatorController.leftBumper().onTrue(m_elevator.m_elbow.Human());
  // m_operatorController.leftBumper().onTrue(m_wrist.vertical());
  // m_operatorController.button(8).onTrue(m_elevator.Home());
  // m_operatorController.button(8).onTrue(m_elevator.m_elbow.up());
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // if (!m_robotContainer.m_operatorController.x().getAsBoolean()&&
    // !m_robotContainer.m_operatorController.y().getAsBoolean()&&
    // !m_robotContainer.m_operatorController.button(1).getAsBoolean()&&
    // !m_robotContainer.m_operatorController.rightBumper().getAsBoolean()&&
    // !m_robotContainer.m_operatorController.leftBumper().getAsBoolean()&&
    // !m_robotContainer.m_operatorController.b().getAsBoolean())
    // {
    // m_robotContainer.m_elevator.elbowSetPoint=0;
    // m_robotContainer.m_elevator.m_elbow.up();
    // }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
