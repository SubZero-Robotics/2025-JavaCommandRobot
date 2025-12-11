// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Gyro
    public static final int kPidgeyCanId = 13;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final Time kPeriodicInterval = Seconds.of(0.02);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 18;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 9;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 20;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AlgaeArmConstants {
    public static final int kArmMotorId = 17;
    public static final int kIntakeMotorId = 6;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 0.0;
    public static final double kFF = 0.0;

    public static final double kHasAlgaeCurrent = 35;
    
    public static final AngularVelocity kMaxRpm = RotationsPerSecond.of(5676 / 60);
    public static final Angle kHomeRotation = Degrees.of(0);
    public static final Angle kMinRotation = Degrees.of(0);
    public static final Angle kMaxRotation = Degrees.of(85);
    public static final Angle kRelativeDistancePerRev = Degrees.of(360 / 75);
    public static final Angle kAbsoluteDistancePerRev = Degrees.of(360);
    public static final AngularVelocity kDefaultVelocity = DegreesPerSecond.of(10);
    public static final double kVelocityScalar = 1.0;
    public static final Angle kTolerance = Degrees.of(2);
    public static final Distance kArmLength = Inches.of(17);
    public static final Angle kAlgaeIntakePosition = Degrees.of(38);

    // 0 Degrees from the robot is different from the mech
    public static final Angle kMechOffset = Radians.of(-Math.PI / 2);
  }

 public static final class ElevatorConstants {
    public static final int kLeadElevatorMotorCanId = 19;
    public static final int kFollowerElevatorMotorCanId = 7;

    public static final int kBottomLimitSwitchPort = 1;
    public static final int kTopLimitSwitchPort = 2;

    // public static final PidSettings kElevatorPidSettings = {
    //   160.0, 0.0, 0.0,
    //   0.0, 0.0, false};

    // public static final subzero::PidSettings kHomePidSettings = {
    //   10.0, 0.0, 0.0,
    //   0.0, 0.0, false};

    public static final double kElevatorStartPosition = 4.875; //inches

    public static final double kMaxRpm = 5676; //rpm

    // Placeholder values
    public static final Distance kMinDistance = Inches.of(0.0); 
    public static final Distance kMaxDistance = Inches.of(20.1);
    public static final Distance kRelativeDistancePerRev = Inches.of((1.685 * Math.PI) / 36.0); // Pitch diameter of gear with 36:1 ratio gearbox
    public static final LinearVelocity kDefaultVelocity = MetersPerSecond.of(0.66); //meters per second
    public static final double kVelocityScalar = 1.0;
    public static final Distance kTolerance = Inches.of(0.1);
    public static final double gearing = 1;
    public static final Mass carriageMassKg = Kilograms.of(3.5);
    public static final Distance drumRadiusMeters = Meters.of(0.04);
    public static final boolean simulateGravity = true;
    // // Placeholder
    // public static final subzero::SingleAxisMechanism kElevatorMechanism {
    // // min length
    // 2_in,
    // // angle
    // 90_deg,
    // // line width
    // 10.0,
    // // color
    // subzero::Colorpublic static finalants::kRed};

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 0.0;
    public static final double kFF = 0.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      1 * 10, 20);
};

}
