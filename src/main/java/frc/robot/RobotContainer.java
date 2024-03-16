// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveAndOrientToNote;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowAndIntake;
import frc.robot.commands.LoadAmp;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootWhenSpinning;
import frc.robot.commands.SnapToAndAlign;
import frc.robot.commands.TuckItIn;
import frc.robot.commands.WheelsX;
import frc.robot.commands.IntakeCommands.IntakeThenPulse;
import frc.robot.commands.IntakeCommands.SetIntakePosition;
import frc.robot.commands.IntakeCommands.SetIntakeSpeed;
import frc.robot.commands.IntakeCommands.Pulse;
import frc.robot.commands.LiftCommands.ManualLift;
import frc.robot.commands.ShooterCommands.ManualWristAngle;
import frc.robot.commands.ShooterCommands.SetSpin;
import frc.robot.commands.ShooterCommands.SetSpinAndAngle;
import frc.robot.commands.ShooterCommands.SetWristAngle;
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
        public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();
        public final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
        public final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
        public final static LiftSubsystem m_liftSubsystem = new LiftSubsystem();
        private final static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
        private final static ShooterCameraSubsystem m_shooterCamera = new ShooterCameraSubsystem();
        private final static IntakeCameraSubsystem m_intakeCamera = new IntakeCameraSubsystem();

        // initialize the controllers
        // the one on the left
        private static FilteredJoystick m_leftJoystick = new FilteredJoystick(Constants.OIConstants.kLeftJoystickPort);

        // the one on the right
        private static FilteredJoystick m_rightJoystick = new FilteredJoystick(
                        Constants.OIConstants.kRightJoystickPort);

        // da buttons
        private static FilteredButton m_buttons = new FilteredButton(OIConstants.kButtonPort);

        // da operator controller
        private static FilteredController m_operatorController = new FilteredController(
                        Constants.OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // name commands for use in pathPlanner
                NamedCommands.registerCommand("FollowAndIntake",
                                new FollowAndIntake(m_intakeSubsystem, m_driveSubsystem, m_intakeCamera,
                                                m_liftSubsystem,
                                                m_shooterSubsystem));
                NamedCommands.registerCommand("ShootFromRight",
                                new Shoot(() -> 4000, () -> 5500, () -> 80, m_intakeSubsystem, m_shooterSubsystem,
                                                m_liftSubsystem));
                NamedCommands.registerCommand("Pulse", new Pulse(m_intakeSubsystem));
                NamedCommands.registerCommand("ShootFromLeft",
                                new Shoot(() -> 4000, () -> 5500, () -> 80, m_intakeSubsystem, m_shooterSubsystem,
                                                m_liftSubsystem));
                NamedCommands.registerCommand("ShootFromMiddle",
                                new Shoot(() -> 4000, () -> 5500, () -> 70, m_intakeSubsystem, m_shooterSubsystem,
                                                m_liftSubsystem));
                NamedCommands.registerCommand("IntakeThenPulse", new IntakeThenPulse(m_intakeSubsystem, m_liftSubsystem,
                                m_shooterSubsystem, m_operatorController::getLeftBumper));
                // Configure the button bindings
                configureButtonBindings();

                // set default command for drive
                // TODO: inversion may be needed
                m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
                                m_rightJoystick::getX,
                                m_rightJoystick::getY, m_leftJoystick::getX,
                                () -> !m_buttons.getTopSwitch(),
                                Constants.DriveConstants.kRateLimitsEnabled, m_rightJoystick::getButtonTwo,
                                m_rightJoystick::getThrottle)); // TODO: determine what inversion is needed

                // default command for lift
                m_liftSubsystem.setDefaultCommand(
                                new ManualLift(m_operatorController::getYLeft,
                                                m_operatorController::getLeftStickPressed, m_liftSubsystem));

                // default command for shooter
                m_shooterSubsystem.setDefaultCommand(
                                new ManualWristAngle(m_operatorController::getYRight, m_shooterSubsystem));

                // default command for climber
                m_climberSubsystem
                                .setDefaultCommand(
                                                m_climberSubsystem.run(() -> m_climberSubsystem.setSpeeds(0.0, false)));
                // default command for intake
                // m_intakeSubsystem.setDefaultCommand(new IntakePositionCommand(
                // () -> Constants.IntakeConstants.kIntakeStowedPosition, m_intakeSubsystem));

                // Configure the AutoBuilder last
                AutoBuilder.configureHolonomic(
                                m_driveSubsystem::getPose,
                                m_driveSubsystem::resetOdometry,
                                m_driveSubsystem::getRobotRelativeSpeeds,
                                m_driveSubsystem::drive,
                                Constants.AutoConstants.kPathFollowerConfig,
                                () -> Robot.allianceColor,
                                m_driveSubsystem);

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putString("Starting Pose", m_driveSubsystem.getPose().toString());

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

                // button 7 on right joystick sets wheels to x
                new Trigger(m_rightJoystick::getButtonSeven).whileTrue(new WheelsX(m_driveSubsystem));

                // zero gyro on right joystick button 5
                new Trigger(m_rightJoystick::getButtonFive)
                                .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.zeroHeading()));

                // set shooter to 0
                new Trigger(m_operatorController::getAButton).onTrue(new SetSpin(() -> 0.0, m_shooterSubsystem));

                // intake controls
                {
                        // set intake speeds
                        new Trigger(m_operatorController::getLeftTriggerActive)
                                        .whileTrue(new SetIntakeSpeed(() -> Constants.IntakeConstants.kOuttakeSpeed,
                                                        m_intakeSubsystem));
                        new Trigger(m_operatorController::getRightTriggerActive)
                                        .whileTrue(new SetIntakeSpeed(() -> Constants.IntakeConstants.kIntakeSpeed,
                                                        m_intakeSubsystem));

                        // set intake positions
                        new Trigger(m_operatorController::getXButton)
                                        .onTrue(new SetIntakePosition(
                                                        () -> Constants.IntakeConstants.kIntakeLoadPosition,
                                                        m_intakeSubsystem));
                        new Trigger(m_operatorController::getBButton)
                                        .onTrue(new SetIntakePosition(
                                                        () -> Constants.IntakeConstants.kIntakeStowedPosition,
                                                        m_intakeSubsystem));
                }

                // intake and pulse
                new Trigger(m_operatorController::getRightBumper)
                                .whileTrue(new IntakeThenPulse(m_intakeSubsystem, m_liftSubsystem,
                                                m_shooterSubsystem, m_operatorController::getLeftBumper));

                new Trigger(m_leftJoystick::getTriggerActive)
                                .whileTrue(new DriveAndOrientToNote(m_driveSubsystem, m_intakeCamera,
                                                m_rightJoystick::getX,
                                                m_rightJoystick::getY, m_leftJoystick::getX,
                                                () -> false,
                                                Constants.DriveConstants.kRateLimitsEnabled,
                                                m_rightJoystick::getButtonTwo,
                                                m_rightJoystick::getThrottle));

                // climber bindings (all on left joystick)
                {
                        // hold button three and press four to zero left
                        new Trigger(m_leftJoystick::getButtonFour).and(m_leftJoystick::getButtonThree)
                                        .onTrue(m_climberSubsystem.run(() -> m_climberSubsystem.zeroLeft()));

                        // hold button three and press five to zero right
                        new Trigger(m_leftJoystick::getButtonFive).and(m_leftJoystick::getButtonThree)
                                        .onTrue(m_climberSubsystem.run(() -> m_climberSubsystem.zeroRight()));

                        // button eight controls both down
                        new Trigger(m_leftJoystick::getButtonEight).whileTrue(m_climberSubsystem
                                        .run(() -> m_climberSubsystem.setSpeeds(-Constants.ClimberConstants.kFastSpeed,
                                                        m_leftJoystick.getButtonThree())));
                        // button nine controls both up
                        new Trigger(m_leftJoystick::getButtonNine).whileTrue(m_climberSubsystem
                                        .run(() -> m_climberSubsystem.setSpeeds(Constants.ClimberConstants.kFastSpeed,
                                                        m_leftJoystick.getButtonThree())));

                        // button six moves left up
                        new Trigger(m_leftJoystick::getButtonSix).whileTrue(m_climberSubsystem
                                        .run(() -> m_climberSubsystem.setLeftSpeed(
                                                        Constants.ClimberConstants.kFastSpeed,
                                                        m_leftJoystick.getButtonThree())));
                        // button seven moves left down
                        new Trigger(m_leftJoystick::getButtonSeven).whileTrue(m_climberSubsystem
                                        .run(() -> m_climberSubsystem.setLeftSpeed(
                                                        -Constants.ClimberConstants.kFastSpeed,
                                                        m_leftJoystick.getButtonThree())));

                        // button ten moves right up
                        new Trigger(m_leftJoystick::getButtonTen).whileTrue(m_climberSubsystem
                                        .run(() -> m_climberSubsystem.setRightSpeed(
                                                        -Constants.ClimberConstants.kFastSpeed,
                                                        m_leftJoystick.getButtonThree())));
                        // button eleven moves right down
                        new Trigger(m_leftJoystick::getButtonEleven).whileTrue(m_climberSubsystem
                                        .run(() -> m_climberSubsystem
                                                        .setRightSpeed(Constants.ClimberConstants.kFastSpeed,
                                                                        m_leftJoystick.getButtonThree())));
                }
                // follow and intake note: this is a test
                // new Trigger(m_operatorController::getLeftStickPressed)
                // .whileTrue(new FollowAndIntake(m_intakeSubsystem, m_driveSubsystem,
                // m_intakeCamera,
                // m_liftSubsystem, m_shooterSubsystem));

                // zero absolute encoder lift
                new Trigger(m_operatorController::getBackButton).and(m_operatorController::getStartButton)
                                .onTrue(m_liftSubsystem.runOnce(() -> m_liftSubsystem.zero()));

                // pulse intake to center note
                new Trigger(m_operatorController::getYButton).onTrue(new Pulse(m_intakeSubsystem));

                // orient to speaker
                new Trigger(() -> m_rightJoystick.getPovState() == 180)
                                .whileTrue(new SnapToAndAlign(m_driveSubsystem, m_shooterCamera,
                                                () -> (Robot.allianceColor) ? 4 : 7, () -> 0, m_rightJoystick::getX,
                                                m_rightJoystick::getY));

                // orient to amp blue
                // new Trigger(() -> m_rightJoystick.getPovState() == 270)
                // .whileTrue(new SnapToAndAlign(m_driveSubsystem, m_shooterCamera,
                // () -> 6, () -> 90/*
                // * TODO:may need 270, and may need to invert joystick
                // */, m_rightJoystick::getX));
                // orient to amp red
                // new Trigger(() -> m_rightJoystick.getPovState() == 90)
                // .whileTrue(new SnapToAndAlign(m_driveSubsystem, m_shooterCamera,
                // () -> 5, () -> 270/*
                // * TODO:may need 90, and may need to invert joystick
                // */, m_rightJoystick::getX));

                new Trigger(() -> m_operatorController.getPovState() == 180)
                                .whileTrue(new Shoot(() -> 5500, () -> 5500, () -> 90,
                                                m_intakeSubsystem, m_shooterSubsystem, m_liftSubsystem));

                new Trigger(() -> m_operatorController.getPovState() == 0)
                                .whileTrue(new Shoot(() -> 4000, () -> 5500, () -> 70,
                                                m_intakeSubsystem, m_shooterSubsystem, m_liftSubsystem));
                new Trigger(m_rightJoystick::getTriggerActive)
                                .onTrue(new SetSpinAndAngle(() -> 70, () -> 4000, () -> 5500, m_shooterSubsystem));

                new Trigger(m_rightJoystick::getTriggerActive).onFalse(
                                new ShootWhenSpinning(m_intakeSubsystem, m_shooterSubsystem, m_liftSubsystem));

                new Trigger(() -> m_operatorController.getPovState() == 90)
                                .onTrue(new LoadAmp(m_shooterSubsystem, m_intakeSubsystem));
                new Trigger(m_operatorController::getLeftStickPressed).onTrue(
                                new SetWristAngle(() -> Constants.ShooterConstants.kLiftSafeAngle, m_shooterSubsystem));
                new Trigger(() -> m_operatorController.getPovState() == 270).onTrue(
                                new SetSpin(() -> 5500, m_shooterSubsystem));
                new Trigger(m_operatorController::getRightStickPressed).onTrue(
                                new SetWristAngle(() -> Constants.ShooterConstants.kAmpAngle, m_shooterSubsystem));
                new Trigger(m_operatorController::getBackButton).onTrue(new SetWristAngle(
                                () -> Constants.ShooterConstants.kIntakeSafeAngle, m_shooterSubsystem));

                // tuck everything in for safety
                new Trigger(m_leftJoystick::getButtonTwo).and(m_leftJoystick::getButtonThree)
                                .onTrue(new TuckItIn(m_liftSubsystem, m_shooterSubsystem, m_intakeSubsystem));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}