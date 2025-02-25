// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final XboxController m_controller = new XboxController(0);
  public final TrapezoidProfile.Constraints kTheConstraints = new TrapezoidProfile.Constraints(3, Math.PI / 4);
  SlewRateLimiter XSpeedLimiter = new SlewRateLimiter(5);
  SlewRateLimiter YSpeedLimiter = new SlewRateLimiter(5);
  SlewRateLimiter TurnLimiter = new SlewRateLimiter(3);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> XSpeedLimiter.calculate(-modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> YSpeedLimiter.calculate(-modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -TurnLimiter.calculate(-modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));
    
    // Configure the button bindings
    configureButtonBindings();
    
  }
  public double getXspeed(){
    return m_controller.getLeftX();
  }
  public double getYspeed(){
   return m_controller.getLeftY();
  }
  public double getTurnspeed(){
    return m_controller.getRightX();
  }
  public DrivetrainSubsystem getDrivetrainSubsystem(){
  return m_drivetrainSubsystem;
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getAButtonPressed).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }
  public void resetGyro(){
    m_drivetrainSubsystem.zeroGyroscope();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    /*
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 1).setKinematics(m_drivetrainSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), 
    List.of(
    new Translation2d(1,0), 
    new Translation2d(1,0)),
    new Pose2d(2, 0, Rotation2d.fromDegrees(-180)), 
    trajectoryConfig);

    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, kTheConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics,
      xController,
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometery(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules())
    ); 
    */
    return new InstantCommand();
  }


  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }
}
