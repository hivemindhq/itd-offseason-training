package dev.nullftc.wpiftc.localization.impl.pinpoint.v1;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;

public class PinpointKinematics implements Kinematics<Pose2D, Pose2D> {
    @Override
    public ChassisSpeeds toChassisSpeeds(Pose2D wheelSpeeds) {
        return null;
    }

    @Override
    public Pose2D toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return null;
    }

    @Override
    public Twist2d toTwist2d(Pose2D start, Pose2D end) {
        Pose2d startPose = new Pose2d(start.getX(DistanceUnit.METER), start.getY(DistanceUnit.METER), new Rotation2d(start.getHeading(AngleUnit.RADIANS)));
        Pose2d endPose = new Pose2d(end.getX(DistanceUnit.METER), end.getY(DistanceUnit.METER), new Rotation2d(end.getHeading(AngleUnit.RADIANS)));
        return startPose.log(endPose);
    }

    @Override
    public Pose2D copy(Pose2D positions) {
        return new Pose2D(DistanceUnit.METER, positions.getX(DistanceUnit.METER), positions.getY(DistanceUnit.METER), AngleUnit.RADIANS, positions.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void copyInto(Pose2D positions, Pose2D output) {
    }

    @Override
    public Pose2D interpolate(Pose2D startValue, Pose2D endValue, double t) {
        Pose2d wpiPose = new Pose2d(startValue.getX(DistanceUnit.METER), startValue.getY(DistanceUnit.METER), new Rotation2d(startValue.getHeading(AngleUnit.RADIANS))).interpolate(
                new Pose2d(
                        endValue.getX(DistanceUnit.METER),
                        endValue.getY(DistanceUnit.METER),
                        new Rotation2d(
                                endValue.getHeading(AngleUnit.RADIANS)
                        )
                ), t
        );
        return new Pose2D(
                DistanceUnit.METER,
                wpiPose.getX(),
                wpiPose.getY(),
                AngleUnit.RADIANS,
                wpiPose.getRotation().getRadians()
        );
    }
}
