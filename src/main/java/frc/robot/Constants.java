// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax.IdleMode;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        // ccw is positive
        public static final double kFrontLeftChassisAngularOffset = Math.PI;
        public static final double kFrontRightChassisAngularOffset = -Math.PI / 2;
        public static final double kBackLeftChassisAngularOffset = Math.PI / 2;
        public static final double kBackRightChassisAngularOffset = 0;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kBackLeftDrivingCanId = 3;
        public static final int kFrontRightDrivingCanId = 5;
        public static final int kBackRightDrivingCanId = 7;

        public static final int kFrontLeftTurningCanId = 2;
        public static final int kBackLeftTurningCanId = 4;
        public static final int kFrontRightTurningCanId = 6;
        public static final int kBackRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;

        // starting pose of the robot
        public static final Pose2d kStartingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        // Odometry class for tracking robot pose
        public static final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(
                        Constants.Sensors.gyro.getAngle() * (Constants.DriveConstants.kGyroReversed ? -1 : 1)),
                new SwerveModulePosition[] {
                        ModuleConstants.m_frontLeft.getPosition(),
                        ModuleConstants.m_frontRight.getPosition(),
                        ModuleConstants.m_backLeft.getPosition(),
                        ModuleConstants.m_backRight.getPosition()
                }, kStartingPose);
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        // Currently 12T is installed on the robot
        public static final int kDrivingMotorPinionTeeth = 12;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
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

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

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
        public static final double kDriveDeadband = 0.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(3.0, 1.0, 0.0);

        public static final PIDConstants kThetaPIDConstants = new PIDConstants(Math.PI, 0.0, 0.0);


        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
                kTranslationPIDConstants,  kThetaPIDConstants, 
                DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                0.53, // Drive base radius in meters. Distance from robot center to
                // furthest module
                new ReplanningConfig() // Default path replanning config. See the API
        // for the options here
        );
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class VisionConstants {

        public static final PhotonCamera kLefty = new PhotonCamera("lefty");
        public static final PhotonCamera kRighty = new PhotonCamera("righty");

        public static final double kCameraHeight = Units.inchesToMeters(14);

        public static final double kConeHeight = Units.inchesToMeters(13);
        public static final IntSupplier kConeIndex = () -> 0;

        public static final double kCubeHeight = Units.inchesToMeters(5.5);
        public static final IntSupplier kCubeIndex = () -> 1;

        public static final Transform3d kRobotToLeftCam = new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));
        public static final Transform3d kRobotToRightCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));

        // this makes reading from a file work, it's kinda messy
        public static final AprilTagFieldLayout kAprilTagFieldLayout;

        static {
            try {
                kAprilTagFieldLayout = AprilTagFieldLayout
                        .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (IOException e) {
                throw new RuntimeException("Welp that's strange");
            }
        }

        public static final PhotonPoseEstimator kPhotonPoseEstimator = new PhotonPoseEstimator(kAprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kLefty, kRobotToLeftCam);
        // TODO

    }

    public static final class PoseConstants {

    }
}
