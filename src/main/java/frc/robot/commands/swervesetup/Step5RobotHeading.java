package frc.robot.commands.swervesetup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

/**
 * Step 5: Robot Heading
 * <p>
 * For all the previous steps, you could keep your robot's wheels off the ground or even have your robot on its side.
 * The orientation of the robot itself didn't matter so long as you could mentally think through which side of the robot
 * was the front and which side was the left. Ideally, you would have verified each step with the robot on the ground,
 * but it wasn't strictly required.
 * <p>
 * This step is a bit different. To do FOC (Field Oriented Control), you must know what angle your robot is facing
 * relative to some known vector. In the world of FRC, this is just "positive X". When your robot is placed on the field,
 * imagine it is placed where it is facing away from you. It would be facing in the positive X direction. We consider
 * that to be a heading of 0 degrees. If you turn the robot to the left, that's a heading of 90 degrees, facing you is
 * 180, etc.
 * <p>
 * The goal in this step is to use an IMU, such as a Pigeon, to get the heading value. This part is a little difficult,
 * because you have to make sure the value is in range, but you also have to take into account how the device reading
 * the heading value is mounted to determine whether to use pitch, roll, or yaw. As long as moving your robot
 * counterclockwise results in a positive increase as described above, and it starts out at 0 when facing away from you
 * in the positive X direction of the field, you're golden.
 * <p>
 * You should not expect the swerve modules to move at all in this test, but you will need to rotate the robot manually.
 * We still get the swerve subsystem in the constructor, but this is mostly for consistency and because it ensures no
 * other subsystem may accidentally use it while you're trying to move the robot.
 * <p>
 * Lastly, something outside the scope of this step. You'll probably "zero" the heading value when you start up the
 * robot, however sometimes this is not what you really want. You ALWAYS want zero to be "positive X axis", but if you
 * have an auto that starts with the robot facing away from the positive X axis, you don't want that to become zero.
 * There's a few ways to handle this, the easiest of which is probably just specifying in your auto that you're starting
 * at 180 degrees instead of 0. Another neat trick to use, in case your heading value is at some point incorrect is to
 * map a button, such as start, from the controller to a function that simply "zeros" the heading. This way you can, in
 * the middle of a match, have the robot face "forward", press that button, and it will be zeroed correctly again.
 * <p>
 * As I said though, that last section is a bit out of scope of this step in getting swerve to work, but useful to know.
 */
public class Step5RobotHeading extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;  // Unused but kept here for consistency
    private final Supplier<Rotation2d> headingSupplier;

    public Step5RobotHeading(
            SwerveSubsystem swerveSubsystem,
            Supplier<Rotation2d> headingSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.headingSupplier = headingSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("heading", headingSupplier.get().getDegrees());
    }
}
