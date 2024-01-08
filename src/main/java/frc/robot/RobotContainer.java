// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStop;
import frc.robot.commands.FollowTarget;
import frc.robot.commands.GyroReset;
import frc.robot.commands.OrientToTarget;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FilteredButton;
import frc.robot.utils.FilteredJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // autochooser
        private final SendableChooser<Command> autoChooser;

        // Initialize subsystems
        public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final static CameraSubsystem m_cameraSubsytem = new CameraSubsystem();

        // initialize the controllers
        // the one on the left
        public static FilteredJoystick m_leftJoystick = new FilteredJoystick(Constants.OIConstants.kLeftJoystickPort);

        // the one on the right
        public static FilteredJoystick m_rightJoystick = new FilteredJoystick(Constants.OIConstants.kRightJoystickPort);
        FilteredButton m_buttons = new FilteredButton(OIConstants.kButtonPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // name commands for use in pathPlanner
                NamedCommands.registerCommand("OrientToTarget", new OrientToTarget(m_robotDrive, m_cameraSubsytem));
                // Configure the button bindings
                configureButtonBindings();

                // set default command for drive
                m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive, m_rightJoystick::getX,
                                m_rightJoystick::getY, m_leftJoystick::getX,
                                () -> (!m_rightJoystick.getTriggerActive() && !m_buttons.getTopSwitch()),
                                Constants.DriveConstants.kRateLimitsEnabled, m_rightJoystick::getButtonTwo,
                                m_rightJoystick::getThrottle));

                // Configure the AutoBuilder last
                AutoBuilder.configureHolonomic(
                                m_robotDrive::getPose,
                                m_robotDrive::resetOdometry,
                                m_robotDrive::getRobotRelativeSpeeds,
                                m_robotDrive::drive,
                                Constants.AutoConstants.kPathFollowerConfig,
                                m_robotDrive
                );

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        /**
         * Use this method to define your button -> command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                // top left button and x button on controller sets wheels to x
                new Trigger(m_buttons::getOneA).or(
                                m_rightJoystick::getButtonSeven).whileTrue(m_robotDrive.runOnce( () -> m_robotDrive.setX()));
                // top right button resets gyro or right button five
                new Trigger(m_buttons::getOneC).or(m_rightJoystick::getButtonFive).onTrue(new GyroReset());
                // bottom middle button stops drive
                new Trigger(m_buttons::getThreeB).whileTrue(new DriveStop(m_robotDrive));
                // orient to target on right joystick three, or middle middle button
                new Trigger(m_rightJoystick::getButtonThree).and(m_buttons::getTwoB)
                                .whileTrue(new OrientToTarget(m_robotDrive, m_cameraSubsytem));
                //follow target on right joystick four
                new Trigger(m_rightJoystick::getButtonFour).whileTrue(new FollowTarget(m_robotDrive, m_cameraSubsytem));
                //reset odo on right joystick ten
                new Trigger(m_rightJoystick::getButtonTen).onTrue(m_robotDrive.runOnce(
                                () -> m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}
