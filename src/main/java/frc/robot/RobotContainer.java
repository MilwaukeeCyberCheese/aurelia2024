// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStop;
import frc.robot.commands.GyroReset;
import frc.robot.commands.WheelsX;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.ShooterCommands.SpinDownCommand;
import frc.robot.commands.ShooterCommands.SpinUpCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeCameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterCameraSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.FilteredButton;
import frc.robot.utils.FilteredController;
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
        private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
        private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
        private final static LiftSubsystem m_liftSubsystem = new LiftSubsystem();
        private final static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
        private final static ShooterCameraSubsystem m_shooterCamera = new ShooterCameraSubsystem();
        private final static IntakeCameraSubsystem m_intakeCamera = new IntakeCameraSubsystem();


        // initialize the controllers
        // the one on the left
        public static FilteredJoystick m_leftJoystick = new FilteredJoystick(Constants.OIConstants.kLeftJoystickPort);

        // the one on the right
        public static FilteredJoystick m_rightJoystick = new FilteredJoystick(Constants.OIConstants.kRightJoystickPort);

        //da buttons
        FilteredButton m_buttons = new FilteredButton(OIConstants.kButtonPort);

        //da operator controller
        FilteredController m_operatorController = new FilteredController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // name commands for use in pathPlanner

                // Configure the button bindings
                configureButtonBindings();

                // set default command for drive
                //TODO: inversion may be needed
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
                                () -> DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red)),
                                m_robotDrive);

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
                                m_rightJoystick::getButtonSeven).whileTrue(new WheelsX(m_robotDrive));
                // top right button resets gyro or right button five
                new Trigger(m_buttons::getOneC).or(m_rightJoystick::getButtonFive).onTrue(new GyroReset());
                // bottom middle button stops drive
                new Trigger(m_buttons::getThreeB).whileTrue(new DriveStop(m_robotDrive));
                // reset odo on right joystick ten
                new Trigger(m_rightJoystick::getButtonTen).onTrue(m_robotDrive.runOnce(
                                () -> m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
                //run shooter at full speed
                //TODO:  set MAX RPM
                new Trigger(m_operatorController::getAButton).onTrue(new SpinUpCommand(() -> 1800, m_shooterSubsystem));
                new Trigger(m_operatorController::getBButton).onTrue(new SpinDownCommand(m_shooterSubsystem));
                new Trigger(m_operatorController::getXButton).onTrue(new IntakeCommand(m_intakeSubsystem));
                
                

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}