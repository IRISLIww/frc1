package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class indexer extends SubsystemBase{
    private final TalonFX intakemotor = new TalonFX(10, "canivore");
    private final TalonFX ampmotor = new TalonFX(9, "canivore");
    private VoltageOut voltageRequest2 = new VoltageOut(0).withEnableFOC(true);
    private TalonFXConfiguration ampconfigs = new TalonFXConfiguration();
    private TalonFXConfiguration intakeconfigs = new TalonFXConfiguration();
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Double> current = intakemotor.getStatorCurrent();
    private final StatusSignal<Double> temp = intakemotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = intakemotor.getRotorVelocity();
    private final StatusSignal<Double> current2 = ampmotor.getStatorCurrent();
    private final StatusSignal<Double> temp2 = ampmotor.getDeviceTemp();
    private final StatusSignal<Double> RPS2 = ampmotor.getRotorVelocity();
    private double setpointvolts =0.0; 
    private double setpointvolts2 = 0.0;
    public indexer() {
       intakeconfigs.CurrentLimits.StatorCurrentLimit = 70;
       intakeconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
       intakeconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       intakemotor.getConfigurator().apply(intakeconfigs);
       ampconfigs.CurrentLimits.StatorCurrentLimit = 70;
       ampconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
       ampconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       ampmotor.getConfigurator().apply(ampconfigs);
       BaseStatusSignal.setUpdateFrequencyForAll(50,current,temp, RPS);

    }
    public void runintake(double voltage){
        setpointvolts = voltage;
        intakemotor.setControl(voltageRequest.withOutput(voltage));
    }
    public void runamp(double voltage){
        setpointvolts2 = voltage; 
        ampmotor.setControl(voltageRequest2.withOutput(voltage));
    }
    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(current,temp,RPS,current2,temp2,RPS2);
        SmartDashboard.putNumber("temperature", temp.getValue());
        SmartDashboard.putNumber("current", current.getValue());
        SmartDashboard.putNumber("RPS", RPS.getValue());
        SmartDashboard.putNumber("temperature 2 ", temp2.getValue());
        SmartDashboard.putNumber("current 2", current2.getValue());
        SmartDashboard.putNumber("RPS 2", RPS2.getValue());
        SmartDashboard.putNumber("setpointvolts", setpointvolts);
    }
}
