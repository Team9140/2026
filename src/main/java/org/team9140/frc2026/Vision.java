/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.team9140.frc2026;

import java.util.EnumSet;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.team9140.frc2026.helpers.LimelightHelpers;
import org.team9140.frc2026.helpers.LimelightHelpers.PoseEstimate;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision {
    private final EstimateConsumer estConsumer;
    private final String cameraName;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private Supplier<Pose2d> driveSimPose;

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your
     *                    desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(String camera_name, EstimateConsumer estConsumer, Supplier<Pose2d> driveSimPoseSupplier) {
        this.driveSimPose = driveSimPoseSupplier;

        this.cameraName = camera_name;
        this.estConsumer = estConsumer;
        

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim(camera_name);
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout());
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            cameraSim = new PhotonCameraSim(new PhotonCamera(camera_name), cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, Constants.Vision.ROBOT_TO_CAM);

            cameraSim.enableDrawWireframe(true);
        }
    }

    private class Listener implements TableEventListener {
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                periodic();
            }
        }
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(this.cameraName, mode);
    }

    private void periodic() {
        double timestamp = Utils.getCurrentTimeSeconds();
        LimelightHelpers.LimelightResults llResult = LimelightHelpers.getLatestResults(this.cameraName);

        timestamp = llResult.latency_capture - llResult.latency_pipeline;

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(this.cameraName);

        if (Robot.isSimulation()) {
            getSimDebugField()
                .getObject("VisionEstimation")
                .setPose(mt1.pose);
        }

        if (mt1 != null && mt1.tagCount >= 1) {
            this.estConsumer.accept(EstimateType.MT1, timestamp, mt1);
        }
    }

    // ----- Simulation

    private void updateSimState() {
        visionSim.update(driveSimPose.get());
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }

    public enum EstimateType {
        MT1,
        MT2
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(EstimateType kind, double timestamp, PoseEstimate estimate);
    }

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;

    private void startSimThread() {
        m_simNotifier = new Notifier(() -> {
            updateSimState();
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private int m_listenerID = -1;

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable(this.cameraName).addListener("json",
                    EnumSet.of(Kind.kValueAll), new Listener());

            if (Robot.isSimulation()) {
                startSimThread();
            }
        }
    }
}