package frc.robot.commands.swervesetup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

/**
 * Step 6: Tuning Speed
 * <p>
 * The last major thing to do is tune our speed values. So far, we've been sending the "meters per second" values
 * directly to the motors. This isn't technically accurate, and we'd like to be more accurate mostly for odometry.
 * <p>
 * The first thing you're going to want to do here is make sure you never send a value larger than 1 to any of your
 * motors. You've probably already done that for your rotation motor, but it's time to do it for your movement motors
 * too. You can do this individually in each module and that would work just fine. You can also adjust the array
 * of {@link edu.wpi.first.math.kinematics.SwerveModuleState} inside your SwerveSubsystem by normalizing the values. In
 * other words, take the speedMetersPerSecond of each state and find the max absolute value of all of them. If it's
 * greater than 1, divide ALL the speedMetersPerSecond by that absolute value. This will make the largest max out at
 * 1/-1, and the others will be scaled appropriately.
 * <p>
 * I also suggest using the swerve module state optimize function at this point, assuming your robot moves correctly as
 * you'd expect. Essentially what this does is a neat little trick for swerve modules. If the module needs to go to some
 * specific angle to move in that direction, but it would need to rotate more than 90 degrees to do so, it will actually
 * rotate the state's angle by 90 degrees and invert the speedMetersPerSecond value. See
 * {@link edu.wpi.first.math.kinematics.SwerveModuleState#optimize(SwerveModuleState, Rotation2d)} for more details.
 * <p>
 * At this point, you have a choice. You can modify the speed multiplier and the spin rotations per second to where they
 * feel like they'll work well in teleop mode. Essentially, this class is a DriveCommand. You'll have a working Swerve
 * system that you can use in competition. You very likely will not be able to run powerful Autos, though. For that,
 * you need better odometry. Odometry is just the question "where am I".
 * <p>
 * For our robot, we know the gearing of each swerve module, and we know the circumference of the wheel. If we knew the
 * rotations per second of the motor, the math would be easy. If you think about it without the gearing, 1 rotation of
 * the motor moves one circumference of the wheel in distance. The only thing the gearing changes is how many rotations
 * of the input motor does it take to get one rotation of the wheel. For example, with a MK4 swerve module with L4
 * gearing, that's a 5.14 to 1 ratio. That means 5.14 rotations of the input motor is the same as 1 rotation of the
 * wheel.
 * <p>
 * Knowing that means we can work backwards. If we have the meters per second we're targeting from the swerve module
 * state, we can divide that by the circumference of the wheel to get rotations per second of the wheel. Then we
 * multiply that by our gear ratio (5.14 in the example above) and we know the rotations per second we want our drive
 * motor to move at. At this point you can look at what's available for your specific motor to do the correct control
 * mechanism.
 * <p>
 * The reason this gets a little complicated is that your motor is ultimately controlled by voltage and current. As your
 * battery gets weaker, the same value sent to the motor will move your motor slower than it would with a stronger
 * battery. This, coupled with the fact that the motor moving under load (ie the weight of the robot) is different from
 * if it was spinning in air with no weight on it. You should be essentially looking into a "feed forward controller",
 * similar to the pid controller we used for the rotation. Technically, you could have used a feed forward controller
 * there as well, but it honestly doesn't matter as much in that case all things considered, and I find it's a little
 * easier to tune.
 * <p>
 * Ultimately the goal of these steps was to get a working swerve module. The last thing to do is make your DriveCommand
 * for teleop based drive control.
 */
public class Step6TuningSpeed extends Command {
    private static final double SPEED_MULTIPLIER = 3;
    private static final double SPIN_ROTATIONS_PER_SECOND = Math.PI / 2;

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Rotation2d> pigeonYawSupplier;
    private final Supplier<Double> joystickAngleSupplier;
    private final Supplier<Translation2d> joystickMovementSupplier;

    public Step6TuningSpeed(
            SwerveSubsystem swerveSubsystem,
            Supplier<Rotation2d> pigeonYawSupplier,
            Supplier<Double> joystickAngleSupplier,
            Supplier<Translation2d> joystickMovementSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.pigeonYawSupplier = pigeonYawSupplier;
        this.joystickAngleSupplier = joystickAngleSupplier;
        this.joystickMovementSupplier = joystickMovementSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Rotation2d direction = pigeonYawSupplier.get();

        Translation2d movement = joystickMovementSupplier.get();

        double leftRightSpin = joystickAngleSupplier.get();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                movement.getX() * SPEED_MULTIPLIER,
                movement.getY() * SPEED_MULTIPLIER,
                leftRightSpin * SPIN_ROTATIONS_PER_SECOND,
                direction
        );
        swerveSubsystem.apply(chassisSpeeds);
    }
}
