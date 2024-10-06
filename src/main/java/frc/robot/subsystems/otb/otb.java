package frc.robot.subsystems.otb;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class otb extends SubsystemBase{
    private final TalonFX pivotmotor = new TalonFX(12, "canivore");
    private MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private TalonFXConfiguration pivotconfigs = new TalonFXConfiguration();
    private final TalonFX intakemotor = new TalonFX(11, "canivore");
    private final StatusSignal<Double> current = pivotmotor.getStatorCurrent();
    private final StatusSignal<Double> temp = pivotmotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = pivotmotor.getRotorVelocity();
    private double setpointvolts = 0.0;
    private double setpointvolts2 = 0.0;
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    public otb(){
        pivotconfigs.CurrentLimits.StatorCurrentLimit = 70; 
        pivotconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotconfigs.MotionMagic.MotionMagicCruiseVelocity = 75;
        pivotconfigs.MotionMagic.MotionMagicAcceleration = 150;
        pivotconfigs.MotionMagic.MotionMagicJerk = 10000;
        pivotconfigs.Slot0.kP = 8.413;
        pivotconfigs.Slot0.kD = 0.003;
        pivotconfigs.Slot0.kS = 0.0587;
        pivotconfigs.Slot0.kV = 0.0044;
        pivotconfigs.Slot0.kG = 0.0662;
        pivotconfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 
        pivotmotor.getConfigurator().apply(pivotconfigs);
    }
    public void setMotionMagic(double degrees){
        double rotations = degrees/360.;
        pivotmotor.setControl(motionmagicrequest.withPosition(rotations));
    }
    public void setPointVolts(double voltage){
        setpointvolts = voltage;
        pivotmotor.setControl(voltageRequest.withOutput(voltage));
    }
    public void runintake(double voltage){
        setpointvolts2 = voltage;
        intakemotor.setControl(voltageRequest.withOutput(voltage));
    }
    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(current,temp,RPS);
        SmartDashboard.putNumber("temperature", temp.getValue());
        SmartDashboard.putNumber("current", current.getValue());
        SmartDashboard.putNumber("RPS", RPS.getValue());
        SmartDashboard.putNumber("setpointvolts", setpointvolts);
    }
}
