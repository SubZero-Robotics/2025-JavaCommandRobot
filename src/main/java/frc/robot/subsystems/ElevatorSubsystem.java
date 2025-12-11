package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.*;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax m_leadMotor = new SparkMax(ElevatorConstants.kLeadElevatorMotorCanId, MotorType.kBrushless);
    SparkClosedLoopController m_pidController = m_leadMotor.getClosedLoopController();

    SparkMax m_followMotor = new SparkMax(ElevatorConstants.kFollowerElevatorMotorCanId, MotorType.kBrushless);

    SparkMaxSim m_simElevatorMotor = new SparkMaxSim(m_leadMotor, DCMotor.getNEO(1));
    SparkAbsoluteEncoderSim m_simElevatorEncoder = m_simElevatorMotor.getAbsoluteEncoderSim();

    ElevatorSim m_elevatorSim = new ElevatorSim(DCMotor.getNEO(1), ElevatorConstants.gearing,
            ElevatorConstants.carriageMassKg.magnitude(),
            ElevatorConstants.drumRadiusMeters.magnitude(), ElevatorConstants.kMinDistance.magnitude(), ElevatorConstants.kMaxDistance.magnitude(),
            ElevatorConstants.simulateGravity, ElevatorConstants.kMinDistance.magnitude(), 0.01, 0.00);

    RelativeEncoder m_encoder = m_leadMotor.getEncoder();
    AbsoluteEncoder m_absEnc = m_leadMotor.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("elevatorBase", 1.5, 1);

    MechanismLigament2d m_elevatorArmMech = root.append(new MechanismLigament2d("Elevator", 0.5, 90));

    public ElevatorSubsystem() {

        config.closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD)
                .outputRange(-1, 1);

        m_leadMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard
        // -> Elevator Sim
        SmartDashboard.putData("Elevator Sim", mech);
    }

    public Command moveElevatorToPosition(Distance distanceInInches) {
        return new InstantCommand(() -> {
            m_pidController.setReference(Rotations.of(distanceInInches.magnitude()
                / ElevatorConstants.kRelativeDistancePerRev.magnitude()).magnitude(), ControlType.kPosition);
            }
        );
    }

    public Distance getPosition() {
        if (Robot.isSimulation())
            return Inches
                    .of(m_simElevatorEncoder.getPosition() * ElevatorConstants.kRelativeDistancePerRev.magnitude());

        return Inches.of(m_absEnc.getPosition() * ElevatorConstants.kRelativeDistancePerRev.magnitude());
    }

    private LinearVelocity getVelocity() {
        if (!Robot.isReal())
            return InchesPerSecond.of(
                    m_simElevatorEncoder.getVelocity() * ElevatorConstants.kRelativeDistancePerRev.magnitude() / 60.0);

        return InchesPerSecond
                .of(m_absEnc.getVelocity() * ElevatorConstants.kRelativeDistancePerRev.magnitude() / 60.0);
    }

    @Override
    public void periodic() {
        // 3 Is the height of the mech2d widget
        m_elevatorArmMech.setLength(3.0 * getPosition().magnitude() / ElevatorConstants.kMaxDistance.magnitude());

        if (Robot.isSimulation()) {

        }

        System.out.println(getPosition());
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInput(m_simElevatorMotor.getAppliedOutput() * RoboRioSim.getVInCurrent());

        m_elevatorSim.update(0.020);

        m_simElevatorEncoder.setPosition(Inches.convertFrom(m_elevatorSim.getPositionMeters(), Meters)
                / ElevatorConstants.kRelativeDistancePerRev.magnitude());

        System.out.println(m_elevatorSim.getPositionMeters());

        // m_simElevatorMotor.iterate(
        //         RotationsPerSecond.of(
        //                 InchesPerSecond.convertFrom(
        //                         m_elevatorSim.getVelocityMetersPerSecond(), MetersPerSecond)
        //                         / ElevatorConstants.kRelativeDistancePerRev.magnitude())
        //                 .magnitude() * 60.0,
        //         RoboRioSim.getVInVoltage(), 0.020);

        RoboRioSim
                .setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }
}