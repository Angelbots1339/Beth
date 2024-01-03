package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class SwerveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Rotation2d> pigeonYawSupplier;
    private final Supplier<Double> joystickAngleSpinSupplier;
    private final Supplier<Translation2d> joystickMovementSupplier;
    private final PIDController pidController;
    private static final double ANGULAR_SPEED = Math.PI / 2;

    public SwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Rotation2d> pigeonYawSupplier,
            Supplier<Double> joystickAngleSpinSupplier,
            Supplier<Translation2d> joystickMovementSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.pigeonYawSupplier = pigeonYawSupplier;
        this.joystickAngleSpinSupplier = joystickAngleSpinSupplier;
        this.joystickMovementSupplier = joystickMovementSupplier;

        pidController = new PIDController(0.01, 0.001, 0);
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(5, 15);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
        Rotation2d direction = pigeonYawSupplier.get();
//        Rotation2d target = joystickAngleSpinSupplier.get();

        SmartDashboard.putNumber("yaw", direction.getDegrees());
//        SmartDashboard.putNumber("target", target.getDegrees());

//        double rotationsPerSecond = pidController.calculate(direction.getDegrees(), target.getDegrees()) * Math.PI/2;
//        if(pidController.atSetpoint()) {
//            rotationsPerSecond = 0;
//        }
//        SmartDashboard.putNumber("robot rotate rotationsPerSecond", rotationsPerSecond);
        Translation2d movement = joystickMovementSupplier.get();
        double rotationsPerSecond = joystickAngleSpinSupplier.get() * ANGULAR_SPEED;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                movement.getX(),
                movement.getY(),
                rotationsPerSecond,
                direction
        );
        swerveSubsystem.apply(speeds);
    }
}
