package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;


public class ElevatorSubsystem extends SubsystemBase
{
    SparkMax m_leadMotor = new SparkMax(ElevatorConstants.kLeadElevatorMotorCanId, MotorType.kBrushless);
    SparkClosedLoopController m_pidController = m_leadMotor.getClosedLoopController();

    SparkMax m_followMotor = new SparkMax(ElevatorConstants.kFollowerElevatorMotorCanId, MotorType.kBrushless);

    SparkMaxSim m_simElevatorMotor = new SparkMaxSim(m_leadMotor, DCMotor.getNEO(1));
    SparkAbsoluteEncoderSim m_simElevatorEncoder = m_simElevatorMotor.getAbsoluteEncoderSim();

    double gearing = 1, carriageMassKg = 1, drumRadiusMeters = 1, minHeightMeter= 0, maxHeightMeters = 1, startingHeightMeters = 0;
    boolean simulateGravity = true;

    ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(1), gearing, carriageMassKg, drumRadiusMeters, minHeightMeter, maxHeightMeters, simulateGravity , startingHeightMeters, 0.01, 0.00);

    RelativeEncoder m_encoder = m_leadMotor.getEncoder();
    // AbsoluteEncoder m_absEnc = m_leadMotor.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("elevatorBase", 5, 0);

    MechanismLigament2d m_algaeArmMech = root.append(new MechanismLigament2d("Elevator", 0.5, 90));

    public ElevatorSubsystem() {        
        // TODO: implement code below
        //config.closedLoop.p(AlgaeArmConstants.kP)
//         .i(AlgaeArmConstants.kI)
//         .d(AlgaeArmConstants.kD)
//         .outputRange(-1, 1);
// m_algaeArmMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

// m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);

//     // Publish Mechanism2d to SmartDashboard
//     // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
//     SmartDashboard.putData("Elevator Sim", m_mech2d);
    }
}