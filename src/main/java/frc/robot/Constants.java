// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import frc.robot.utils.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.subsystems.MAXSwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final class Sensors {
                /**
                 * remember to invert this with the kGyroReversed
                 */
                public static final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

                // TODO: Limit switch for intake
                public static final DigitalInput intakeLimitSwitch = new DigitalInput(0);

                // public static final ColorSensorV3 shooterColorSensor = new
                // ColorSensorV3(I2C.Port.kOnboard);

        }

        public static final class DriveConstants {
                // Rate limits on or off
                public static final BooleanSupplier kRateLimitsEnabled = () -> true;

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double kMaxSpeedMetersPerSecond = 4.8;
                public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

                public static final double kSlowModifier = 0.7;

                public static final double kDirectionSlewRate = 1.2; // radians per second
                public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(25.5);
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(22);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                // ccw is positive
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kFrontLeftChassisAngularOffset = 3 * Math.PI / 2;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;

                public static final boolean kRotInverted = true;

                // SPARK MAX CAN IDs
                public static final int kFrontLeftDrivingCanId = 7;
                public static final int kBackLeftDrivingCanId = 5;
                public static final int kFrontRightDrivingCanId = 1;
                public static final int kBackRightDrivingCanId = 3;

                public static final int kFrontLeftTurningCanId = 8;
                public static final int kBackLeftTurningCanId = 6;
                public static final int kFrontRightTurningCanId = 2;
                public static final int kBackRightTurningCanId = 4;

                public static final boolean kGyroReversed = true;

                // starting pose of the robot
                public static final Pose2d kStartingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

                // Odometry class for tracking robot pose
                public static final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
                                Constants.DriveConstants.kDriveKinematics,
                                Rotation2d.fromDegrees(
                                                Constants.Sensors.gyro.getAngle()
                                                                * (Constants.DriveConstants.kGyroReversed ? -1 : 1)),
                                new SwerveModulePosition[] {
                                                ModuleConstants.m_backLeft.getPosition(),
                                                ModuleConstants.m_frontLeft.getPosition(),
                                                ModuleConstants.m_backRight.getPosition(),
                                                ModuleConstants.m_frontRight.getPosition()
                                }, kStartingPose);
        }

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                // Currently 12T is installed on the robot
                public static final int kDrivingMotorPinionTeeth = 13;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kVortexFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction) / 60.0; // meters per second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;

                public static final CANSparkMax.IdleMode kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax.IdleMode kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 50; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps

                // Create MAXSwerveModules
                public final static MAXSwerveModule m_frontLeft = new MAXSwerveModule(
                                Constants.DriveConstants.kFrontLeftDrivingCanId,
                                Constants.DriveConstants.kFrontLeftTurningCanId,
                                Constants.DriveConstants.kFrontLeftChassisAngularOffset);

                public final static MAXSwerveModule m_frontRight = new MAXSwerveModule(
                                Constants.DriveConstants.kFrontRightDrivingCanId,
                                Constants.DriveConstants.kFrontRightTurningCanId,
                                Constants.DriveConstants.kFrontRightChassisAngularOffset);

                public final static MAXSwerveModule m_backLeft = new MAXSwerveModule(
                                Constants.DriveConstants.kBackLeftDrivingCanId,
                                Constants.DriveConstants.kBackLeftTurningCanId,
                                Constants.DriveConstants.kBackLeftChassisAngularOffset);

                public final static MAXSwerveModule m_backRight = new MAXSwerveModule(
                                Constants.DriveConstants.kBackRightDrivingCanId,
                                Constants.DriveConstants.kBackRightTurningCanId,
                                Constants.DriveConstants.kBackRightChassisAngularOffset);
        }

        public static final class OIConstants {
                public static final int kLeftJoystickPort = 0;
                public static final int kRightJoystickPort = 1;
                public static final int kButtonPort = 2;
                public static final int kOperatorControllerPort = 3;
                public static final double kDriveDeadband = 0.1;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 1.0;
                public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                // TODO: this will need to be tuned
                public static final com.pathplanner.lib.util.PIDConstants kTranslationPIDConstants = new com.pathplanner.lib.util.PIDConstants(
                                0.05, 0.0, 0.0);

                public static final com.pathplanner.lib.util.PIDConstants kThetaPIDConstants = new com.pathplanner.lib.util.PIDConstants(
                                Math.PI / 3.0, 0.0, 0.0);

                // Constraint for the motion profiled robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

                public static final TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
                                kTranslationPIDConstants, kThetaPIDConstants,
                                DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                                0.53, // Drive base radius in meters. Distance from robot center to
                                      // furthest module
                                new ReplanningConfig() // Default path replanning config. See the API
                // for the options here
                );

                public static final HashMap<String, Pose2d> kStartingPositions = new HashMap<String, Pose2d>() {
                        {
                                put("Copy of RightFiveFour",
                                                new Pose2d(1.11, 1.99, new Rotation2d(Math.toRadians(0.0))));
                                put("Left", new Pose2d(1.29, 7.02, new Rotation2d(Math.toRadians(0.0))));
                                put("Middle", new Pose2d(1.33, 5.53, new Rotation2d(Math.toRadians(0.0))));
                                put("RightFiveFour", new Pose2d(1.11, 1.99, new Rotation2d(Math.toRadians(0.0))));
                                put("RightFourThree", new Pose2d(1.11, 1.99, new Rotation2d(Math.toRadians(0.0))));
                                put("TestAuto", new Pose2d(0.3, 2.0, new Rotation2d(Math.toRadians(0.0))));
                        }
                };
        }

        public static final class MotorConstants {
                public static final double kNeoFreeSpeedRpm = 5676;
                public static final double kVortexFreeSpeedRpm = 6784;
        }

        public class VisionConstants {
                public static final class Note {
                        public static final double kHeight = Units.inchesToMeters(2.0);
                }

                public static final class IntakeCamera {

                        public static final PhotonCamera kCamera = new PhotonCamera("IntakeCamera");

                        public static final double kCameraHeight = Units.inchesToMeters(10.5);

                        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
                                        new Rotation3d(0, 0, 0));

                }

                public static final class ShooterCamera {
                        public static final PhotonCamera kCamera = new PhotonCamera("ShooterCamera");
                        public static final double kCameraHeight = Units.inchesToMeters(11); // TODO: find this, very
                                                                                             // important
                        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                        new Rotation3d(0, 50, 0));

                }

                public static final class Pose {
                        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField
                                        .loadAprilTagLayoutField();
                        public static final PhotonPoseEstimator kPoseEstimator = new PhotonPoseEstimator(
                                        kTagLayout,
                                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ShooterCamera.kCamera,
                                        ShooterCamera.kRobotToCam);

                }

                public static final class TagPositions {
                        private static final Pose3d kTagFour = new Pose3d(Units.inchesToMeters(652.73),
                                        Units.inchesToMeters(218.42), Units.inchesToMeters(57.13),
                                        new Rotation3d(0, 0, 180));
                        private static final Pose3d kTagFive = new Pose3d(Units.inchesToMeters(578.77),
                                        Units.inchesToMeters(323), Units.inchesToMeters(53.38),
                                        new Rotation3d(0, 0, 270));
                        private static final Pose3d kTagSix = new Pose3d(Units.inchesToMeters(72.5),
                                        Units.inchesToMeters(323), Units.inchesToMeters(53.38),
                                        new Rotation3d(0, 0, 270));
                        private static final Pose3d kTagSeven = new Pose3d(Units.inchesToMeters(-1.5),
                                        Units.inchesToMeters(218.42), Units.inchesToMeters(57.13),
                                        new Rotation3d(0, 0, 0));

                        public static final HashMap<Integer, Pose3d> kTagPositions = new HashMap<Integer, Pose3d>() {
                                {
                                        put(4, kTagFour);
                                        put(5, kTagFive);
                                        put(6, kTagSix);
                                        put(7, kTagSeven);
                                }
                        };

                }
        }

        public static final class ShooterConstants {
                public static final int kUpperShooterCanId = 14;
                public static final boolean kUpperShooterInverted = true;
                public static final CANSparkMax kUpperShooterMotor = new CANSparkMax(kUpperShooterCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final int kUpperCurrentLimit = 40;
                public static final RelativeEncoder kUpperShooterEncoder = kUpperShooterMotor.getEncoder();
                public static final SparkPIDController kUpperShooterController = kUpperShooterMotor.getPIDController();

                public static final int kLowerShooterCanId = 16;
                public static final boolean kLowerShooterInverted = false;
                public static final CANSparkMax kLowerShooterMotor = new CANSparkMax(kLowerShooterCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final int kLowerCurrentLimit = 40;
                public static final RelativeEncoder kLowerShooterEncoder = kLowerShooterMotor.getEncoder();
                public static final SparkPIDController kLowerShooterController = kLowerShooterMotor.getPIDController();

                public static final double kShooterConversionFactor = 1.0;

                public static final PIDConstants kUpperShooterPIDConstants = new PIDConstants(0.00, 0.000, 0.0,
                                0.00021); // TODO:
                public static final PIDConstants kLowerShooterPIDConstants = new PIDConstants(0.00, 0.000, 0.0,
                                0.00021); // TODO:
                public static final double kShooterTolerance = 10;
                public static final CANSparkMax.IdleMode kShooterIdleMode = CANSparkMax.IdleMode.kCoast;

                public static final int kWristCanId = 15;
                public static final boolean kWristInverted = false;
                public static final CANSparkMax kWristMotor = new CANSparkMax(kWristCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final AbsoluteEncoder kWristEncoder = kWristMotor
                                .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
                public static final boolean kWristEncoderInverted = false;
                public static final SparkPIDController kWristController = kWristMotor.getPIDController();

                public static final double kWristConversionFactor = 360.0;
                public static final PIDConstants kWristPIDConstants = new PIDConstants(0.015, 0.0, 0.0, 0.0); // TODO:
                                                                                                              // retune
                public static final double kWristMaxOutput = 0.15;
                public static final double kWristTolerance = 3.0; // TODO: find tolerance

                public static final double[] kWristLimits = { 50.0, 170.0 }; // TODO: set limits
                public static final CANSparkMax.IdleMode kWristIdleMode = CANSparkMax.IdleMode.kBrake;

                // angles
                public static final double kIntakeSafeAngle = 140.0;
                public static final double kShootAngle = 110.0;
                public static final double kLiftSafeAngle = 80.0;
                public static final double kAmpAngle = 60.0; // TODO: confirm that

                // TODO: find wait time
                public static final double kShotWaitTime = 400;
                public static final double kAmpWaitTime = 400;

                public static final double kMaxRPM = MotorConstants.kNeoFreeSpeedRpm * kShooterConversionFactor;
                public static final double kAmpRPM = 500;// TODO: find amp RPM
                public static final double kLoadRPM = 20;// TODO: find load RPM

                public static final double kRedNoteDetectionThreshold = 200; // TODO: determine threshold

                public static final double kManualModifier = 1.0 / 3.0;
        }

        public static final class IntakeConstants {
                public static final int kIntakeCanId = 12;
                public static final boolean kIntakeInverted = false;
                public static final CANSparkMax kIntakeMotor = new CANSparkMax(kIntakeCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final int kIntakeCurrentLimit = 80;

                public static final int kIntakeAngleCanId = 11;
                public static final boolean kIntakePivotInverted = false;
                public static final CANSparkMax.IdleMode kIntakePivotIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax.IdleMode kIntakeIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax kIntakePivotMotor = new CANSparkMax(kIntakeAngleCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final AbsoluteEncoder kIntakePositionEncoder = kIntakePivotMotor
                                .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
                public static final SparkPIDController kIntakePositionController = kIntakePivotMotor.getPIDController();
                public static final boolean kIntakePositionEncoderInverted = true;
                public static final double kIntakePivotMaxOuput = 0.3;
                public static final PIDConstants kPIDConstants = new PIDConstants(0.007, 0.0, 0.002, 0.000);
                public static final double kIntakePositionConversionFactor = 360;
                public static final double kTolerance = 2;

                // TODO: determine positions
                public static final double kIntakeLoadPosition = 213;
                public static final double kIntakeStowedPosition = 160;
                public static final double kIntakeAmpPosition = 130;
                public static final double kIntakeOutPosition = 20;

                // TODO: determine limits
                public static final double[] kIntakePositionLimits = { 9, 220 };

                // TODO: determine speeds
                public static final double kIntakeSpeed = 0.7;
                public static final double kLoadSpeed = -1.0;
                public static final double kAmpSpeed = -0.6;

                // TODO: determine range
                public static final double kDeployRange = 2;

                public static final double kPulseSpeed = 0.4;
                public static final double kPulseTime = 80;
        }

        public static final class LiftConstants {
                public static final int kLiftCanId = 13;
                public static final CANSparkMax kLiftMotor = new CANSparkMax(kLiftCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final AbsoluteEncoder kLiftEncoder = kLiftMotor
                                .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
                public static final SparkPIDController kLiftController = kLiftMotor.getPIDController();
                public static final PIDConstants kLiftPIDConstants = new PIDConstants(0.25, 0.0, 0.0);
                public static final CANSparkMax.IdleMode kIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final boolean kInverted = false;

                // conversion factor in inches
                // TODO: probably done, check math
                public static final double kLiftConversionFactor = /* motor shaft to low hex lift */ (27.0 / 24.0) /
                /* circumference of gear */(Math.PI * 1.757) * 16;

                public static final double kLiftConversionFactorOnboard = kLiftConversionFactor / 12;

                // TODO: find tolerance
                public static final double kLiftTolerance = 0.3;

                // TODO: set lift limits
                public static final double[] kLiftLimits = { 0.0, 11 };

                // TODO: find clear of obstructions value
                public static final double kClearOfObstructions = 3;

                // TODO: find manual modifier
                public static final double kManualModifier = 8.0 / 100.0;

                // TODO: find positions
                public static final double kLoadPosition = 0;
                public static final double kAmpPosition = 0;
        }

        public class ClimberConstants {
                public static final int kLeftCanId = 9;
                public static final boolean kLeftInverted = true;
                public static final CANSparkMax.IdleMode kLeftIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax kLeftMotor = new CANSparkMax(kLeftCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final RelativeEncoder kLeftEncoder = kLeftMotor.getEncoder();

                public static final int kRightCanId = 10;
                public static final boolean kRightInverted = false;
                public static final CANSparkMax.IdleMode kRightIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax kRightMotor = new CANSparkMax(kRightCanId,
                                CANSparkMax.MotorType.kBrushless);
                public static final RelativeEncoder kRightEncoder = kRightMotor.getEncoder();

                public static final double kSlowSpeed = 0.4;
                public static final double kFastSpeed = 1.0;

                public static final double kUpperLimit = 300;
        }
}
