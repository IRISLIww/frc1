package frc.robot.subsystems.elevator;
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

public class elevator extends SubsystemBase{
    private final TalonFX elevatormotor = new TalonFX(13, "canivore");
    private final TalonFX elevatormotor2 = new TalonFX(14, "canivore");
    private MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private TalonFXConfiguration elevatorconfigs = new TalonFXConfiguration();
    private final StatusSignal<Double> current = elevatormotor.getStatorCurrent();
    private final StatusSignal<Double> temp = elevatormotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = elevatormotor.getRotorVelocity();
    private final StatusSignal<Double> current2 = elevatormotor2.getStatorCurrent();
    private final StatusSignal<Double> temp2 = elevatormotor2.getDeviceTemp();
    private final StatusSignal<Double> RPS2 = elevatormotor2.getRotorVelocity();
    public elevator(){
        elevatorconfigs.CurrentLimits.StatorCurrentLimit = 70; 
        elevatorconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorconfigs.MotionMagic.MotionMagicCruiseVelocity = 75;
        elevatorconfigs.MotionMagic.MotionMagicAcceleration = 150;
        elevatorconfigs.MotionMagic.MotionMagicJerk = 10000;
        elevatorconfigs.Slot0.kP = 8.413;
        elevatorconfigs.Slot0.kD = 0.003;
        elevatorconfigs.Slot0.kS = 0.0587;
        elevatorconfigs.Slot0.kV = 0.0044;
        elevatorconfigs.Slot0.kG = 0.0662;
        elevatorconfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static; 
        elevatormotor.getConfigurator().apply(elevatorconfigs);
        elevatormotor2.setControl(new Follower(elevatormotor.getDeviceID(), true));

    }
    public void setMotionMagic(double meters){
        double rotations = meters/Units.inchesToMeters(5.51873699838)*17;
        elevatormotor.setControl(motionmagicrequest.withPosition(rotations));
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
