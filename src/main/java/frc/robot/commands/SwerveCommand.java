package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

public class SwerveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Rotation2d> pigeonYawSupplier;
    private final Supplier<Rotation2d> joystickAngleSupplier;
    private final Supplier<Translation2d> joystickMovementSupplier;
    private final PIDController pidController;

    public SwerveCommand(
            DriveSubsystem driveSubsystem,
            Supplier<Rotation2d> pigeonYawSupplier,
            Supplier<Rotation2d> joystickAngleSupplier,
            Supplier<Translation2d> joystickMovementSupplier
    ) {
        this.driveSubsystem = driveSubsystem;
        this.pigeonYawSupplier = pigeonYawSupplier;
        this.joystickAngleSupplier = joystickAngleSupplier;
        this.joystickMovementSupplier = joystickMovementSupplier;

        pidController = new PIDController(0.01, 0.001, 0);
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(5, 15);

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
        Rotation2d direction = pigeonYawSupplier.get();
        Rotation2d target = joystickAngleSupplier.get();

        SmartDashboard.putNumber("yaw", direction.getDegrees());
        SmartDashboard.putNumber("target", target.getDegrees());

        double rotationsPerSecond = pidController.calculate(direction.getDegrees(), target.getDegrees()) * Math.PI/2;
        if(pidController.atSetpoint()) {
            rotationsPerSecond = 0;
        }
        SmartDashboard.putNumber("robot rotate rotationsPerSecond", rotationsPerSecond);
        Translation2d movement = joystickMovementSupplier.get();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                movement.getX(),
                movement.getY(),
                rotationsPerSecond,
                direction
        );
        driveSubsystem.runSwerve(speeds);
    }
}
