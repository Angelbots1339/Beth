package frc.robot.commands.swervesetup;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

/**
 * Step 4: Robot Movement
 * <p>
 * Previously, we did rotation without movement. Now we need to do movement without rotation. Instead of returning a
 * single value in the Supplier like we did last time, we return a `Translation2D` object. We create that object by
 * with an `x` and `y` value. The units are "meters per second", but we're still validating the robot, so we're going to
 * multiply that by a multiplier, just to slow it all down. Right now, your swerve modules should still be setting the
 * motor movement power to be directly whatever the {@link edu.wpi.first.math.kinematics.SwerveModuleState#speedMetersPerSecond}
 * value is based on how we set it up in step 2. The consequence of this is if you move the joystick to the full value
 * of 1, it won't be 0.3 meters per second with the speed multiplier. Instead, it will tell the motors to move at 30%
 * power. So maybe don't just go all out with the joystick while testing. :)
 * <p>
 * You'll want to use a different joystick than the rotation one for movement. The X axis for the joystick is -1 for
 * fully left and 1 for fully right. The Y axis for the joystick is -1 for fully up and 1 for fully down. However, for
 * the robot the positive X axis is forward and the positive Y axis is left. In your supplier, when returning the
 * Translation2D, the `x` parameter will be the negative of your joystick Y axis and the `y` parameter will be the
 * negative of your joystick X axis.
 * <p>
 * To verify that that's true, draw two large plus signs on a piece of paper. Label one of them joystick and the other
 * robot. For the joystick one, write X = -1 on the left side, X = 1 on the right, Y = -1 on the top, and finally Y = 1
 * on the bottom. For the robot one, write X = 1 on the top, X = -1 on the bottom, Y = 1 on the left, and Y = -1 on the
 * right. If you draw a third plus called "swap", where you swap the X and Y axis of the joystick, you will get Y = -1
 * on the left side, Y = 1 on the right side, X = -1 on the top, and X = 1 on the bottom. All we did here was swap X and
 * Y. However the difference between our "swap" plus sign and the "robot" plus sign is just that the X and Y axis are
 * also negated swapping 1 for -1 and vice-versa. Makes sense? No? Grab a piece of paper this time instead of trying to
 * do it in your head. ;p
 * <p>
 * Note that the values put into the dashboard are whatever your joystick supplier returns, so they are in reference to
 * the robot and the field (positive X forward, positive Y left).
 * <p>
 * At the end of this, your robot should move forward if you press up on the joystick, backward if you press down, left
 * if you press left, and right if you press right. The rotation joystick will do nothing.
 */
public class Step4RobotMovement extends Command {
    private static final double SPEED_MULTIPLIER = 0.3;

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Translation2d> joystickMovementSupplier;

    public Step4RobotMovement(
            SwerveSubsystem swerveSubsystem,
            Supplier<Translation2d> joystickMovementSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.joystickMovementSupplier = joystickMovementSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Translation2d movement = joystickMovementSupplier.get();

        SmartDashboard.putNumber("joystick move X", movement.getX());
        SmartDashboard.putNumber("joystick move Y", movement.getY());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                movement.getX() * SPEED_MULTIPLIER,
                movement.getY() * SPEED_MULTIPLIER,
                0
        );
        swerveSubsystem.apply(chassisSpeeds);
    }
}
