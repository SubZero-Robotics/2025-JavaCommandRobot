package frc.robot.subsystems;

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
import frc.robot.Constants.AlgaeArmConstants;

import static edu.wpi.first.units.Units.*;

public class AlgaeArmSubsystem extends SubsystemBase {

    SparkMax m_algaeArmMotor = new SparkMax(AlgaeArmConstants.kArmMotorId, MotorType.kBrushless);
    SparkClosedLoopController m_pidController = m_algaeArmMotor.getClosedLoopController();

    SparkMaxSim m_simAlgaeMotor = new SparkMaxSim(m_algaeArmMotor, DCMotor.getNeo550(1));
    SparkAbsoluteEncoderSim m_simAlgaeEncoder = m_simAlgaeMotor.getAbsoluteEncoderSim();

    SingleJointedArmSim m_armSim = new SingleJointedArmSim(DCMotor.getNeo550(1), 1.0, 0.01,
            AlgaeArmConstants.kArmLength.magnitude(),
            AlgaeArmConstants.kMinRotation.in(Radians), AlgaeArmConstants.kMaxRotation.in(Radians), false, 0);

    RelativeEncoder m_enc = m_algaeArmMotor.getEncoder();
    AbsoluteEncoder m_absEnc = m_algaeArmMotor.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("coralArmBase", 2, 0);

    MechanismLigament2d m_algaeArmMech = root.append(new MechanismLigament2d("Algae Arm", 0.5, 90));

    public AlgaeArmSubsystem() {
        config.closedLoop.p(AlgaeArmConstants.kP)
                         .i(AlgaeArmConstants.kI)
                         .d(AlgaeArmConstants.kD)
                         .outputRange(-1, 1);
        m_algaeArmMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private Command moveToPosition(Angle desiredDegrees) {
        return new InstantCommand(() -> {
            this.m_pidController.setReference(desiredDegrees.magnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }

    public Command home() {
        return moveToPosition(AlgaeArmConstants.kHomeRotation);
    }

    public Command toIntakeAlgaePosition() {
        return moveToPosition(AlgaeArmConstants.kAlgaeIntakePosition);
    }

    public Angle getAngle() {
        if (!Robot.isReal())
            return Degrees.of(m_simAlgaeEncoder.getPosition() * 360.0);

        return Degrees.of(m_absEnc.getPosition() * 360.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Mech2d", mech);

        m_algaeArmMech.setAngle(getAngle().magnitude());
    }

    @Override
    public void simulationPeriodic() {
        m_armSim.setInput(m_simAlgaeMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        m_armSim.update(0.02);

        m_simAlgaeEncoder.setPosition(Units.radiansToRotations(m_armSim.getAngleRads()));

        m_simAlgaeMotor.iterate(Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
}
