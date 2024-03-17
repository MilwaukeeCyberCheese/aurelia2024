// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Transpose;
import frc.robot.utils.parsing.ParseAutoStartIntoPose2d;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public final static SendableChooser<Boolean> m_autoSpin = new SendableChooser<>();

  /**
   * True if red, false if blue
   */
  public static boolean allianceColor;
  public static boolean inAuto;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // port forwarding for photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);

    SmartDashboard.putBoolean("SHOOT!", false);

    // suppress joystick warnings
    DriverStation.silenceJoystickConnectionWarning(true);
    m_autoSpin.setDefaultOption("Pre-Spin", true);
    m_autoSpin.addOption("Don't Pre-Spin", false);
    SmartDashboard.putData("Auto Spin", m_autoSpin);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    allianceColor = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
    RobotContainer.m_liftSubsystem.run(() -> RobotContainer.m_liftSubsystem.zero());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    inAuto = true;

    RobotContainer.m_driveSubsystem.runOnce(() -> RobotContainer.m_driveSubsystem.zeroHeading());
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // reset the pose of the robot to the starting pose of the autonomous command
    System.out.println(m_autonomousCommand.getName() + " da name");

    // get the path of the auto command being run
    Path autoPath = Path.of(
        Filesystem.getDeployDirectory().toString() + "/pathplanner/autos/" + m_autonomousCommand.getName() + ".auto");

    // reset the pose based on the starting position in the auto file
    try {
      Pose2d initialPose = ParseAutoStartIntoPose2d.parseJson(autoPath);
      RobotContainer.m_driveSubsystem
          .resetOdometry((allianceColor) ? Transpose.transposeToRed(initialPose) : initialPose);

      // schedule the autonomous command
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }

    } catch (IOException e) {
      // if the inital pose doesn't exist, reset the pose to the origin
      RobotContainer.m_driveSubsystem
          .resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putString("Current Pose Auto", RobotContainer.m_driveSubsystem.getPose().toString());
  }

  @Override
  public void teleopInit() {
    inAuto = false;

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Current Pose Teleop", RobotContainer.m_driveSubsystem.getPose().toString());
  }

  @Override
  public void testInit() {
    inAuto = false;
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
