// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Subsystems.vision.VisionConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;
  public static final boolean shouldDrive = true;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static class FieldConstants {
    public static double fieldLength = VisionConstants.aprilTagFieldLayout2.getFieldLength();
    public static double fieldlWidth = VisionConstants.aprilTagFieldLayout2.getFieldWidth();

    public static Translation2d redHub = new Translation2d(11.910, 4.060);
    public static Translation2d redLeftTrench =
        VisionConstants.aprilTagFieldLayout2.getTagPose(7).get().getTranslation().toTranslation2d();
    public static Translation2d redRightTrench =
        VisionConstants.aprilTagFieldLayout2
            .getTagPose(12)
            .get()
            .getTranslation()
            .toTranslation2d();

    public static Translation2d blueHub = new Translation2d(fieldLength - 11.910, 4.060);
    public static Translation2d blueLeftTrench =
        VisionConstants.aprilTagFieldLayout2
            .getTagPose(23)
            .get()
            .getTranslation()
            .toTranslation2d();
    public static Translation2d blueRightTrench =
        VisionConstants.aprilTagFieldLayout2
            .getTagPose(28)
            .get()
            .getTranslation()
            .toTranslation2d();
  }
}
