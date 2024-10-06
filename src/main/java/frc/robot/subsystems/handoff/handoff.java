package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class handoff extends SubsystemBase{
    private final TalonFX intakemotor = new TalonFX(10, "canivore");
    private TalonFXConfiguration intakeconfigs = new TalonFXConfiguration();
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Double> current = intakemotor.getStatorCurrent();
    private final StatusSignal<Double> temp = intakemotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = intakemotor.getRotorVelocity();
    private double setpointvolts =0.0; 
    public handoff() {
       intakeconfigs.CurrentLimits.StatorCurrentLimit = 70;
       intakeconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
       intakeconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       intakemotor.getConfigurator().apply(intakeconfigs);

       BaseStatusSignal.setUpdateFrequencyForAll(50,current,temp, RPS);

    }
    public void runintake(double voltage){
        setpointvolts = voltage;
        intakemotor.setControl(voltageRequest.withOutput(voltage));
    }
    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(current,temp,RPS);
        SmartDashboard.putNumber("temperature", temp.getVal7ue());
        SmartDashboard.putNumber("current", current.getValue());
        SmartDashboard.putNumber("RPS", RPS.getValue());
        SmartDashboard.putNumber("setpointvolts", setpointvolts);
    }
}