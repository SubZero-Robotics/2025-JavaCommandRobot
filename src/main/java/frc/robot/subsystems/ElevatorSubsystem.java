package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.*;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax m_leadMotor = new SparkMax(ElevatorConstants.kLeadElevatorMotorCanId, MotorType.kBrushless);
    SparkClosedLoopController m_pidController = m_leadMotor.getClosedLoopController();

    SparkMax m_followMotor = new SparkMax(ElevatorConstants.kFollowerElevatorMotorCanId, MotorType.kBrushless);

    SparkMaxSim m_simElevatorMotor = new SparkMaxSim(m_leadMotor, DCMotor.getNEO(1));

    // Tried simulation with this and it didn't work
    // SparkAbsoluteEncoderSim m_simElevatorEncoder = m_simElevatorMotor.getAbsoluteEncoderSim();

    // ElevatorSim m_elevatorSim = new ElevatorSim(DCMotor.getNEO(1), ElevatorConstants.gearing,
    //         ElevatorConstants.carriageMassKg.magnitude(),
    //         ElevatorConstants.drumRadiusMeters.magnitude(), ElevatorConstants.kMinDistance.magnitude(),
    //         ElevatorConstants.kMaxDistance.magnitude(),
    //         ElevatorConstants.simulateGravity, ElevatorConstants.kMinDistance.magnitude(), 0.01, 0.00);

    RelativeEncoder m_encoder = m_leadMotor.getEncoder();
    AbsoluteEncoder m_absEnc = m_leadMotor.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("elevatorBase", 1.5, 0);

    MechanismLigament2d m_elevatorArmMech = root.append(new MechanismLigament2d("Elevator", 0.5, 90));

    boolean m_simElevatorIsMoving = false;
    Distance m_simCurrentElevatorDistance = Meters
            .of(Meters.convertFrom(ElevatorConstants.kElevatorStartPosition.magnitude(), Inches));
    Distance m_simTargetElevatorDistance = Meters
            .of(Meters.convertFrom(ElevatorConstants.kElevatorStartPosition.magnitude(), Inches));
    Time m_simInitialElevatorMoveTime = Seconds.of(0.0);
    double m_simulationCoefficient;

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
        Distance distanceInMeters = Meters.of(Meters.convertFrom(distanceInInches.magnitude(), Inch));
        if (Robot.isReal())
            return new InstantCommand(() -> {
                m_pidController.setReference(Rotations.of(distanceInInches.magnitude()
                        / ElevatorConstants.kRelativeDistancePerRev.magnitude()).magnitude(), ControlType.kPosition);
            });

        return new InstantCommand(() -> {
            m_simElevatorIsMoving = true;
            m_simInitialElevatorMoveTime = Seconds.of(Timer.getFPGATimestamp());
            m_simTargetElevatorDistance = distanceInMeters;

            System.out.println("Target elevator distance " + m_simTargetElevatorDistance);

            // Using a parabola time curve that looks ax(x - 1). 'a' needs to be adjusted to
            // change
            // the area under the graph, and that coefficient is 6 times the intended
            // distance to travel
            m_simulationCoefficient = -6.0
                    * m_simTargetElevatorDistance.minus(m_simCurrentElevatorDistance).magnitude();
        });
    }

    public Distance getPosition() {
        if (Robot.isReal()) {
            return Inches
                    .of(m_absEnc.getPosition() * ElevatorConstants.kRelativeDistancePerRev.magnitude());
        }

        return Inches.of(m_simCurrentElevatorDistance.in(Inches));
    }

    @Override
    public void periodic() {
        // 3 Is the height of the mech2d widget
        m_elevatorArmMech.setLength(3.0 * getPosition().magnitude() / ElevatorConstants.kMaxDistance.magnitude());

        // System.out.println(getPosition());
    }

    @Override
    public void simulationPeriodic() {
        if (m_simElevatorIsMoving) {

            Time elaspedSinceStart = Seconds.of(Timer.getFPGATimestamp()).minus(m_simInitialElevatorMoveTime);

            // Because floating point numbers are innacurate, we cannot set the threshold to
            // read position. However, we are integrating the velocity profile over 1
            // second so we can stop moving the elevator once one second is reached and stop
            // adjusting
            if (elaspedSinceStart.gte(Seconds.of(1.0))) {
                m_simElevatorIsMoving = false;
                return;
            }

            LinearVelocity velocity = MetersPerSecond.of(
                    m_simulationCoefficient * elaspedSinceStart.magnitude() * (elaspedSinceStart.magnitude() - 1.0));

            Distance deltaPosition = velocity.times(DriveConstants.kPeriodicInterval);
            m_simCurrentElevatorDistance = m_simCurrentElevatorDistance.plus(deltaPosition);
        }
    }
}