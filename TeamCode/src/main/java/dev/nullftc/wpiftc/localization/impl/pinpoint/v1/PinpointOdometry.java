package dev.nullftc.wpiftc.localization.impl.pinpoint.v1;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

public class PinpointOdometry extends Odometry<Pose2D> {
    /**
     * Constructs an Odometry object.
     *
     * @param kinematics        The kinematics of the drivebase.
     * @param currentPose       The current position of the OTOS Odometry Module
     * @param intialPose        The initial pose of the odometry
     */
    public PinpointOdometry(PinpointKinematics kinematics, Pose2D currentPose, Pose2d intialPose) {
        super(
                kinematics,
                new Rotation2d(currentPose.getHeading(AngleUnit.RADIANS)),
                currentPose,
                intialPose
        );
    }
}
