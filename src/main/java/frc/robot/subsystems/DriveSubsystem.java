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
import frc.robot.Constants;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  // vision PIDs
  private PIDController thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
      Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);

  private PIDController xController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
      Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);

  private PIDController yController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
      Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    Constants.DriveConstants.m_odometry.update(
        Rotation2d.fromDegrees(Constants.Sensors.gyro.getAngle() * (Constants.DriveConstants.kGyroReversed ? -1 : 1)),
        new SwerveModulePosition[] {
            Constants.ModuleConstants.m_frontLeft.getPosition(),
            Constants.ModuleConstants.m_frontRight.getPosition(),
            Constants.ModuleConstants.m_backLeft.getPosition(),
            Constants.ModuleConstants.m_backRight.getPosition()
        });

    log();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return new Pose2d(Constants.DriveConstants.m_odometry.getEstimatedPosition().getY(),
        Constants.DriveConstants.m_odometry.getEstimatedPosition().getX() * -1,
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
            Constants.ModuleConstants.m_frontLeft.getPosition(),
            Constants.ModuleConstants.m_frontRight.getPosition(),
            Constants.ModuleConstants.m_backLeft.getPosition(),
            Constants.ModuleConstants.m_backRight.getPosition()
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
    rot *= -1;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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
      rotDelivered = m_currentRotation * Constants.DriveConstants.kMaxAngularSpeed;
    

    // Convert the commanded speeds into the correct units for the drivetrain
    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d
                    .fromDegrees(Constants.Sensors.gyro.getAngle() * (Constants.DriveConstants.kGyroReversed ? -1 : 1)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    Constants.ModuleConstants.m_frontLeft.setDesiredState(swerveModuleStates[0]);
    Constants.ModuleConstants.m_frontRight.setDesiredState(swerveModuleStates[1]);
    Constants.ModuleConstants.m_backLeft.setDesiredState(swerveModuleStates[2]);
    Constants.ModuleConstants.m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot relative to itself without limiters, etc.
   * 
   * @param ChassisSpeeds: chassisSpeeds to run the robot
   */
  public void drive(ChassisSpeeds chassisSpeeds) {

    ChassisSpeeds invertedChassisSpeeds = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond * -1,
        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);

    // Convert the commanded speeds into the correct units for the drivetrain
    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics
        .toSwerveModuleStates(invertedChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    Constants.ModuleConstants.m_frontLeft.setDesiredState(swerveModuleStates[0]);
    Constants.ModuleConstants.m_frontRight.setDesiredState(swerveModuleStates[1]);
    Constants.ModuleConstants.m_backLeft.setDesiredState(swerveModuleStates[2]);
    Constants.ModuleConstants.m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot relative to itself without limiters, etc.
   * 
   * @param ChassisSpeeds: chassisSpeeds to run the robot
   */
  public void driveLimited(ChassisSpeeds chassisSpeeds) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    double xSpeedDelivered;
    double ySpeedDelivered;
    double rotDelivered;

    ChassisSpeeds invertedChassisSpeeds = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond * -1,
        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);

    double ySpeed = invertedChassisSpeeds.vyMetersPerSecond;
    double xSpeed = invertedChassisSpeeds.vxMetersPerSecond;
    double rot = invertedChassisSpeeds.omegaRadiansPerSecond;

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
    rotDelivered = m_currentRotation * Constants.DriveConstants.kMaxAngularSpeed;

    // Convert the commanded speeds into the correct units for the drivetrain
    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    Constants.ModuleConstants.m_frontLeft.setDesiredState(swerveModuleStates[0]);
    Constants.ModuleConstants.m_frontRight.setDesiredState(swerveModuleStates[1]);
    Constants.ModuleConstants.m_backLeft.setDesiredState(swerveModuleStates[2]);
    Constants.ModuleConstants.m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Drive to a pose, assuming the robot is always at (0, 0)
   * 
   * @param pose position to drive to
   */
  public void drive(Pose2d pose){
    double x = xController.calculate(0.0, pose.getX());
    double y = yController.calculate(0.0, pose.getY());
    double theta = thetaController.calculate(0.0, pose.getRotation().getRadians());

    driveLimited(new ChassisSpeeds(x, y, theta));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    Constants.ModuleConstants.m_frontLeft
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    Constants.ModuleConstants.m_frontRight
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    Constants.ModuleConstants.m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    Constants.ModuleConstants.m_backRight
        .setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    Constants.ModuleConstants.m_frontLeft.setDesiredState(desiredStates[0]);
    Constants.ModuleConstants.m_frontRight.setDesiredState(desiredStates[1]);
    Constants.ModuleConstants.m_backLeft.setDesiredState(desiredStates[2]);
    Constants.ModuleConstants.m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Get state of swerve modules
   * 
   * @return SwerveModuleState[]
   */
  public static SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = { Constants.ModuleConstants.m_frontLeft.getState(),
        Constants.ModuleConstants.m_frontRight.getState(), Constants.ModuleConstants.m_backLeft.getState(),
        Constants.ModuleConstants.m_backRight.getState() };
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

   
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(Constants.Sensors.gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return Constants.Sensors.gyro.getRate() * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * 
   * @return ChassisSpeeds object relative to the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds speeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    return speeds;
  }

}
