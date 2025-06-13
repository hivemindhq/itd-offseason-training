package dev.nullftc.wpiftc.localization.impl.otos.v1;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

public class OTOSOdometry extends Odometry<SparkFunOTOS.Pose2D> {
    /**
     * Constructs an Odometry object.
     *
     * @param kinematics        The kinematics of the drivebase.
     * @param currentOTOS       The current position of the OTOS Odometry Module
     * @param intialPose        The initial pose of the odometry
     */
    public OTOSOdometry(OTOSKinematics kinematics, SparkFunOTOS.Pose2D currentOTOS, Pose2d intialPose) {
        super(
                kinematics,
                new Rotation2d(currentOTOS.h),
                currentOTOS,
                intialPose
        );
    }
}
