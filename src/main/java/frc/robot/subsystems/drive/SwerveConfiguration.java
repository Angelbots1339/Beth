package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;

public enum SwerveConfiguration {
    FRONT_LEFT(
            "Front Left",
            new Translation2d(0.25, 0.25),
            13,
            18,
            8,
            173.6
    ),
    FRONT_RIGHT(
            "Front Right",
            new Translation2d(0.25, -0.25),
            12,
            15,
            9,
            96.2
    ),
    BACK_LEFT(
            "Back Left",
            new Translation2d(-0.25, 0.25),
            20,
            17,
            6,
            -37
    ),
    BACK_RIGHT(
            "Back Right",
            new Translation2d(-0.25, -0.25),
            30,
            14,
            7,
            -157.3
    );

    // If gears face inward on wheels at offset, then move motion should be inverted for left from right

    public final String name;
    public final Translation2d position;
    public final int moveCANSparkMaxId;
    public final int rotateCANSparkMaxId;
    public final int canCoderId;
    public final double magneticOffset;

    SwerveConfiguration(
            String name,
            Translation2d position,
            int moveCANSparkMaxId,
            int rotateCANSparkMaxId,
            int CANCoderId,
            double magneticOffset
    ) {
        this.name = name;
        this.position = position;
        this.moveCANSparkMaxId = moveCANSparkMaxId;
        this.rotateCANSparkMaxId = rotateCANSparkMaxId;
        this.canCoderId = CANCoderId;
        this.magneticOffset = magneticOffset;
    }
}