package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooterConstants;
import frc.robot.conversions;
public class shooter extends SubsystemBase{
    private final TalonFX shootermotor = new TalonFX(11,"canivore");
    private final TalonFX shootermotor2 = new TalonFX(10, "canivore");
    private final TalonFX pivotmotor = new TalonFX(12, "canivore");
    private final TalonFX pivotmotor2 = new TalonFX(12, "canivore");
    private MotionMagicVoltage motionmagicrequest2 = new MotionMagicVoltage(0).withEnableFOC(true);
    private TalonFXConfiguration pivotconfigs = new TalonFXConfiguration();
    private MotionMagicVelocityVoltage motionmagicrequest = new MotionMagicVelocityVoltage (0).withEnableFOC(true);
    private TalonFXConfiguration shooterconfigs = new TalonFXConfiguration();
    private TalonFXConfiguration shooterconfigs2 = new TalonFXConfiguration();
    private final StatusSignal<Double> current = shootermotor.getStatorCurrent();
    private final StatusSignal<Double> temp = shootermotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = shootermotor.getRotorVelocity();
    private final StatusSignal<Double> current2 = shootermotor.getStatorCurrent();
    private final StatusSignal<Double> temp2 = shootermotor.getDeviceTemp();
    private final StatusSignal<Double> RPS2 = shootermotor.getRotorVelocity();
    private VelocityVoltage velocityvoltage = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage velocityvoltage2 = new VelocityVoltage(0).withEnableFOC(true);
    private VoltageOut shootrequestvoltage = new VoltageOut(0).withEnableFOC(true);
    private double setpointvolts2 = 0.0;
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    public shooter(){
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
        pivotmotor2.setControl(new Follower(pivotmotor.getDeviceID(), true));
        /* */
        shooterconfigs.Slot0.kA = 0.0;
        shooterconfigs.Slot0.kD = 0.0;
        shooterconfigs.Slot0.kV = 0.0;
        shooterconfigs.Slot0.kI = 0.0;
        shooterconfigs.Slot0.kP = 0.0;
        shooterconfigs.Slot0.kS = 0.0;
        shootermotor.getConfigurator().apply(shooterconfigs);
        shooterconfigs.CurrentLimits.StatorCurrentLimit = 70;
        shooterconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterconfigs2.Slot0.kA = 0.0;
        shooterconfigs2.Slot0.kD = 0.0;
        shooterconfigs2.Slot0.kV = 0.0;
        shooterconfigs2.Slot0.kI = 0.0;
        shooterconfigs2.Slot0.kP = 0.0;
        shooterconfigs2.Slot0.kS = 0.0;
        shooterconfigs.CurrentLimits.StatorCurrentLimit = 70;
        shooterconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shootermotor2.getConfigurator().apply(shooterconfigs2);
    }
    public void setVolts(double voltage){
        shootermotor2.setControl(new Follower(shootermotor.getDeviceID(), true));
        shootermotor.setControl(shootrequestvoltage.withOutput(voltage));
    }
    public void setVelocity(double velocity,double ratio){
        shootermotor.setControl(velocityvoltage.withVelocity(conversions.MPStoRPS(velocity, shooterConstants.wheelCircumferenceMeters, 1.0)));
        shootermotor2.setControl(velocityvoltage2.withVelocity(conversions.MPStoRPS(velocity * ratio, shooterConstants.wheelCircumferenceMeters, 1.0)));
    }
    public void setMotionMagic(double degrees){
        double rotations = degrees/360.;
        pivotmotor.setControl(motionmagicrequest2.withPosition(rotations));
    }
    public void setPointVolts(double voltage){
        setpointvolts2 = voltage;
        pivotmotor.setControl(voltageRequest.withOutput(voltage));
    }
    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(current,temp,RPS,current2,temp2,RPS2);
        SmartDashboard.putNumber("temperature", temp.getValue());
        SmartDashboard.putNumber("current", current.getValue());
        SmartDashboard.putNumber("RPS", RPS.getValue());
        SmartDashboard.putNumber("temperature 2", temp2.getValue());
        SmartDashboard.putNumber("current 2", current2.getValue());
        SmartDashboard.putNumber("RPS 2", RPS2.getValue());

    }

}
