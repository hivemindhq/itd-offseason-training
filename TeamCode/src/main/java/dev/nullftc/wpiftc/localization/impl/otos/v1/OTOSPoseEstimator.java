package dev.nullftc.wpiftc.localization.impl.otos.v1;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import dev.nullftc.wpiftc.localization.FTCPoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A {@link FTCPoseEstimator} for the Sparkfun OTOS sensor.
 */
public class OTOSPoseEstimator extends FTCPoseEstimator<SparkFunOTOS.Pose2D> {
    private ChassisSpeeds speeds = new ChassisSpeeds();

    public OTOSPoseEstimator(SparkFunOTOS.Pose2D otosReading, Pose2d initialPose) {
        this(new OTOSKinematics(), otosReading, initialPose, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.45, 0.45, 1.0e9));
    }

    private OTOSPoseEstimator(OTOSKinematics kinematics, SparkFunOTOS.Pose2D reading, Pose2d initialPose, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, new OTOSOdometry(kinematics, reading, initialPose), stateStdDevs, visionMeasurementStdDevs);
    }

    public void updateVelocity(SparkFunOTOS.Pose2D velocity, SparkFunOTOS.Pose2D position) {
        /**
         * !! IMPORTANT !!
         * NOTE: Requires OTOS initialization angular position to be 0.
         */
        double vX = velocity.x * Math.cos(-position.h) - velocity.y * Math.sin(-velocity.h);
        double vY = velocity.x * Math.sin(-position.h) + velocity.y * Math.cos(-velocity.h);
        speeds = new ChassisSpeeds(vX, vY, velocity.h);
    }

    public void update(SparkFunOTOS.Pose2D wheelPositions) {
        super.update(new Rotation2d(wheelPositions.h), wheelPositions);
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getEstimatedPosition().getRotation());
    }

    public ChassisSpeeds getRobotVelocity() {
        return speeds;
    }
}
