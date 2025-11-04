// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Robot;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private Distance m_simDriverEncoderPosition = Meters.of(0.0);
  private Angle m_simCurrentAngle = Radian.of(0.0);
  private LinearVelocity m_simDriverEncoderVelocity = MetersPerSecond.of(0.0);

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    if (RobotBase.isReal())
      return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    else
      return getSimState();
  }

  public SwerveModuleState getSimState() {
    return new SwerveModuleState(m_simDriverEncoderVelocity,
        new Rotation2d(m_simCurrentAngle.in(Radian) - m_chassisAngularOffset));
  }

  public SwerveModulePosition getSimPosition() {
    return new SwerveModulePosition(m_simDriverEncoderPosition,
        new Rotation2d(m_simCurrentAngle.in(Radian) - m_chassisAngularOffset));
  }

  public Rotation2d getRotation() {
    if(!RobotBase.isReal()) {
      return new Rotation2d(m_simCurrentAngle);
    }
    return new Rotation2d(
      m_turningEncoder.getPosition()  
    );
  }

   /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    if (Robot.isReal())
      return new SwerveModulePosition(
          m_drivingEncoder.getPosition(),
          new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    else
      return getSimPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(getRotation());

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(
      correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(
      correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    // if (correctedDesiredState.speedMetersPerSecond > 1e-5) {
    //   System.out.println("Corrected Desired State Speed (Mps)" + correctedDesiredState.speedMetersPerSecond);
    // }

    // if (correctedDesiredState.angle.getRadians() > 1e-5) {
    //   System.out.println("Corrected Desired State Angle (Radians)" + correctedDesiredState.angle.getRadians());
    // }

    m_desiredState = desiredState;

    if(!RobotBase.isReal()){
      simUpdateDrivePosition(correctedDesiredState);
      m_simCurrentAngle = Radians.of(correctedDesiredState.angle.getRadians());
      // System.out.println("Simulation Current Angle: " + m_simCurrentAngle);
    }
  }

  public void simUpdateDrivePosition(SwerveModuleState desiredState){
    m_simDriverEncoderVelocity = MetersPerSecond.of(desiredState.speedMetersPerSecond);
    m_simDriverEncoderPosition = m_simDriverEncoderPosition.plus(m_simDriverEncoderVelocity.times(Constants.DriveConstants.kPeriodicInterval));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
