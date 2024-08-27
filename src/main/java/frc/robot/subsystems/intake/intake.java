package frc.robot.subsystems.intake;

public class intake{
    private final TalonFX intakemotor = new TalonFX(15, "canivore");
    private TalonFXConfiguration motorconfigs = new TalonFXConfiguration();
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Double> current = intakemotor.getStatorCurrent();
    private final StatusSignal<Double> temp = intakemotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = intakemotor.getRotorVelocity();
    private final setpointvolts; 
    public intake{
        
     

    }

}