// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

 
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final CANCoder frontleftS = new CANCoder(FRONT_LEFT_MODULE_SPEED_ENCODER);
  private final CANCoder frontrightS = new CANCoder(FRONT_RIGHT_MODULE_SPEED_ENCODER);
  private final CANCoder backleftS = new CANCoder(BACK_LEFT_MODULE_SPEED_ENCODER);
  private final CANCoder backrightS = new CANCoder(BACK_RIGHT_MODULE_SPEED_ENCODER);
  private final CANCoder frontleftT = new CANCoder(FRONT_LEFT_MODULE_STEER_ENCODER);
  private final CANCoder frontrightT = new CANCoder(FRONT_RIGHT_MODULE_STEER_ENCODER);
  private final CANCoder backleftT = new CANCoder(BACK_LEFT_MODULE_STEER_ENCODER);
  private final CANCoder backrightT = new CANCoder(BACK_RIGHT_MODULE_STEER_ENCODER);


  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    
  }

 
  public void zeroGyroscope() {
    m_pigeon.setYaw(GyroOffset);

  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees( m_pigeon.getYaw());

  }

  

  public SwerveModuleState getState(CANCoder turning, CANCoder speed){
          return new SwerveModuleState(speed.getVelocity(), new Rotation2d(turning.getPosition()));
  }

  public double getHeading() {
        return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose(){
        return odometer.getPoseMeters();
  }
  public void resetOdometery(Pose2d pose){
        odometer.resetPosition(pose, getRotation2d());
  }


  public void STOPZEROINGTHEWHEELS(SwerveModuleState state, SwerveModule module){
        
        if(Math.abs(state.speedMetersPerSecond) <= 0.001){
                module.set(0, module.getSteerAngle());
                return;
        }
        module.set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, state.angle.getRadians());


}
  public void setModuleStates(SwerveModuleState[] states){

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        STOPZEROINGTHEWHEELS(states[0], m_frontLeftModule);
        STOPZEROINGTHEWHEELS(states[1], m_frontRightModule);
        STOPZEROINGTHEWHEELS(states[2], m_backLeftModule);
        STOPZEROINGTHEWHEELS(states[3], m_backRightModule);

        // m_frontLeftModule.set(speedFromState(states[0],m_frontLeftModule), rotationFormState(states[0], m_frontLeftModule));
        // m_frontRightModule.set(speedFromState(states[1], m_frontRightModule), rotationFormState(states[1], m_frontRightModule));
        // m_backLeftModule.set(speedFromState(states[2], m_backLeftModule), rotationFormState(states[2], m_backLeftModule));
        // m_backRightModule.set(speedFromState(states[3], m_backRightModule), rotationFormState(states[3],m_backRightModule));
  }

  public void stopModules(){
          setModuleStates(m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0,0.0,0.0)));
  }
  @Override
  public void periodic() {

    getHeading();
    odometer.update(getRotation2d(), 
    getState(frontleftT, frontleftS),
     getState(frontrightT, frontrightS),
      getState(backleftT, backleftS),
       getState(backrightT, backrightS) );

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        }

}
