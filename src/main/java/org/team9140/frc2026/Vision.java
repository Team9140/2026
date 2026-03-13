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

import org.team9140.frc2026.helpers.LimelightHelpers;
import org.team9140.frc2026.helpers.LimelightHelpers.PoseEstimate;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private final EstimateConsumer estConsumer;
    private final String cameraName;

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your
     *                    desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(String camera_name, EstimateConsumer estConsumer, Transform3d robotToCamera) {
        this.cameraName = camera_name;
        this.estConsumer = estConsumer;

        if (robotToCamera != null)
            LimelightHelpers.setCameraPose_RobotSpace(this.cameraName, robotToCamera.getX(), robotToCamera.getY(),
                    robotToCamera.getZ(), Units.radiansToDegrees(robotToCamera.getRotation().getX()),
                    Units.radiansToDegrees(robotToCamera.getRotation().getY()),
                    Units.radiansToDegrees(robotToCamera.getRotation().getZ()));
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

        timestamp = Timer.getFPGATimestamp() - llResult.latency_capture - llResult.latency_pipeline;

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(this.cameraName);

        if (mt1 != null && mt1.tagCount >= 1) {
            this.estConsumer.accept(EstimateType.MT1, timestamp, mt1);
        }
    }

    public enum EstimateType {
        MT1,
        MT2
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(EstimateType kind, double timestamp, PoseEstimate estimate);
    }

    private int m_listenerID = -1;

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable(this.cameraName).addListener("json",
                    EnumSet.of(Kind.kValueAll), new Listener());
            System.out.println("new listener ID" + m_listenerID);
        }
    }
}