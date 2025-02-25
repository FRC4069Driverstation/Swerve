package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
       

        SmartDashboard.putNumber("XspeedCOMMAND", this.m_translationXSupplier.getAsDouble());
        SmartDashboard.putNumber("YspeedCOMMAND", this.m_translationYSupplier.getAsDouble());
        SmartDashboard.putNumber("TurnspeedCOMMAND", this.m_rotationSupplier.getAsDouble());
        m_drivetrainSubsystem.setModuleStates(m_drivetrainSubsystem.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        ));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setModuleStates(m_drivetrainSubsystem.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}
