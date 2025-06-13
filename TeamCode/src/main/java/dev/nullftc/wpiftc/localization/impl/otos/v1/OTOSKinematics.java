package dev.nullftc.wpiftc.localization.impl.otos.v1;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;

public class OTOSKinematics implements Kinematics<SparkFunOTOS.Pose2D, SparkFunOTOS.Pose2D> {
    @Override
    public ChassisSpeeds toChassisSpeeds(SparkFunOTOS.Pose2D wheelSpeeds) {
        return null;
    }

    @Override
    public SparkFunOTOS.Pose2D toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return null;
    }

    @Override
    public Twist2d toTwist2d(SparkFunOTOS.Pose2D start, SparkFunOTOS.Pose2D end) {
        Pose2d startPose = new Pose2d(start.x, start.y, new Rotation2d(start.h));
        Pose2d endPose = new Pose2d(end.x, end.y, new Rotation2d(end.h));
        return startPose.log(endPose);
    }

    @Override
    public SparkFunOTOS.Pose2D copy(SparkFunOTOS.Pose2D positions) {
        SparkFunOTOS.Pose2D out = new SparkFunOTOS.Pose2D();
        copyInto(positions ,out);
        return out;
    }

    @Override
    public void copyInto(SparkFunOTOS.Pose2D positions, SparkFunOTOS.Pose2D output) {
        output.set(new SparkFunOTOS.Pose2D(
                positions.x,
                positions.y,
                positions.h
        ));
    }

    @Override
    public SparkFunOTOS.Pose2D interpolate(SparkFunOTOS.Pose2D startValue, SparkFunOTOS.Pose2D endValue, double t) {
        Pose2d wpiPose = new Pose2d(startValue.x, startValue.y, new Rotation2d(startValue.h)).interpolate(new Pose2d(endValue.x, endValue.y, new Rotation2d(endValue.h)), t);
        return new SparkFunOTOS.Pose2D(wpiPose.getX(), wpiPose.getY(), wpiPose.getRotation().getRadians());
    }
}
