// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private PIDController m_thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
      Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    Constants.Sensors.gyro.setAngleAdjustment(0.0);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    Constants.DriveConstants.m_odometry.update(
        Rotation2d.fromDegrees(Constants.Sensors.gyro.getAngle() * (Constants.DriveConstants.kGyroReversed ? -1 : 1)),
        new SwerveModulePosition[] {
            Constants.ModuleConstants.m_backLeft.getPosition(),
            Constants.ModuleConstants.m_frontLeft.getPosition(),
            Constants.ModuleConstants.m_backRight.getPosition(),
            Constants.ModuleConstants.m_frontRight.getPosition()
        });

    log();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return new
    // Pose2d(Constants.DriveConstants.m_odometry.getEstimatedPosition().getY(),
    // Constants.DriveConstants.m_odometry.getEstimatedPosition().getX(),
    // Constants.DriveConstants.m_odometry.getEstimatedPosition().getRotation().rotateBy(
    // ((Constants.DriveConstants.kGyroReversed) ? new Rotation2d(Math.PI) : new
    // Rotation2d())));
    return new Pose2d(Constants.DriveConstants.m_odometry.getEstimatedPosition().getX(),
        Constants.DriveConstants.m_odometry.getEstimatedPosition().getY(),
        Constants.DriveConstants.m_odometry.getEstimatedPosition().getRotation());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    Constants.DriveConstants.m_odometry.resetPosition(
        Rotation2d.fromDegrees(Constants.Sensors.gyro.getAngle() * (Constants.DriveConstants.kGyroReversed ? -1 : 1)),
        new SwerveModulePosition[] {
            Constants.ModuleConstants.m_backLeft.getPosition(),
            Constants.ModuleConstants.m_frontLeft.getPosition(),
            Constants.ModuleConstants.m_backRight.getPosition(),
            Constants.ModuleConstants.m_frontRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    double xSpeedDelivered;
    double ySpeedDelivered;
    double rotDelivered;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    xSpeedDelivered = xSpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeedDelivered = ySpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    rotDelivered = m_currentRotation * Constants.DriveConstants.kMaxAngularSpeed
        * ((Constants.DriveConstants.kRotInverted) ? -1.0 : 1.0);

    drive(fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            Rotation2d
                .fromDegrees(Constants.Sensors.gyro.getAngle() * (Constants.DriveConstants.kGyroReversed ? -1 : 1)))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

  }

  private double currentAngle;
  private boolean turningCorrect = false;

  /**
   * Method to drive the robot relative to itself without limiters, etc.
   * 
   * @param ChassisSpeeds: chassisSpeeds to run the robot
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    
    //correct for pathplanner necessitated rotation
    ChassisSpeeds adjusted = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond,
        Rotation2d.fromDegrees((Robot.inAuto) ? 0.0 : 270.0));

    // //correction for rotational slew
    // if (!Robot.inAuto) {
    //   if (!turningCorrect) {
    //     currentAngle = Constants.Sensors.gyro.getAngle();
    //   }

    //   if (adjusted.omegaRadiansPerSecond < 0.05) {
    //     turningCorrect = true;
    //     adjusted.omegaRadiansPerSecond = m_thetaController.calculate(Constants.Sensors.gyro.getAngle(), currentAngle);
    //   } else {
    //     turningCorrect = false;
    //   }
    // }

    // Convert the commanded speeds into the correct units for the drivetrain
    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics
        .toSwerveModuleStates(adjusted);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

    Constants.ModuleConstants.m_backLeft.setDesiredState(swerveModuleStates[0]);
    Constants.ModuleConstants.m_frontLeft.setDesiredState(swerveModuleStates[1]);
    Constants.ModuleConstants.m_backRight.setDesiredState(swerveModuleStates[2]);
    Constants.ModuleConstants.m_frontRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot relative to itself with limiters
   * 
   * @param ChassisSpeeds: chassisSpeeds to run the robot
   */
  public void driveLimited(ChassisSpeeds chassisSpeeds) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    double xSpeedDelivered;
    double ySpeedDelivered;
    double rotDelivered;

    // TODO: may need to invert some speeds

    double ySpeed = chassisSpeeds.vyMetersPerSecond;
    double xSpeed = chassisSpeeds.vxMetersPerSecond;
    double rot = chassisSpeeds.omegaRadiansPerSecond;

    // Convert XY to polar for rate limiting
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
    double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(Constants.DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
    }

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
    if (angleDif < 0.45 * Math.PI) {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * Math.PI) {
      if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      } else {
        m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.calculate(rot);

    xSpeedDelivered = xSpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeedDelivered = ySpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    rotDelivered = m_currentRotation * Constants.DriveConstants.kMaxAngularSpeed
        * ((Constants.DriveConstants.kRotInverted) ? -1.0 : 1.0);

    drive(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    Constants.ModuleConstants.m_backLeft
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    Constants.ModuleConstants.m_frontLeft
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    Constants.ModuleConstants.m_frontRight
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    Constants.ModuleConstants.m_backRight
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
  }

  /*
   * Get state of swerve modules
   * 
   * @return SwerveModuleState[] frontLeft, frontRight, backLeft, backRight
   */
  public static SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = { Constants.ModuleConstants.m_backLeft.getState(),
        Constants.ModuleConstants.m_frontLeft.getState(), Constants.ModuleConstants.m_backRight.getState(),
        Constants.ModuleConstants.m_frontRight.getState() };
    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Constants.ModuleConstants.m_frontLeft.resetEncoders();
    Constants.ModuleConstants.m_backLeft.resetEncoders();
    Constants.ModuleConstants.m_frontRight.resetEncoders();
    Constants.ModuleConstants.m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    Constants.Sensors.gyro.reset();
  }

  public void log() {
    PPLibTelemetry.setCurrentPose(getPose());

    SmartDashboard.putNumber("Gyro", Constants.Sensors.gyro.getAngle());
  }

  /**
   * 
   * @return ChassisSpeeds object relative to the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

}
