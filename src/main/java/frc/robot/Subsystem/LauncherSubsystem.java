package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LauncherSubsystem extends SubsystemBase{

    VictorSPX medium1Motor = new VictorSPX(7);
    VictorSPX medium2Motor = new VictorSPX(12);
    CANSparkMax down1Motor = new CANSparkMax(11, MotorType.kBrushed);

    private boolean speaker, on;

    public LauncherSubsystem(){
        initMotors();
        initShuffleboard();
    }

    private void initMotors(){
        down1Motor.setInverted(true);
        medium1Motor.setInverted(true);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Down1Motor", down1Motor.get());
        SmartDashboard.putNumber("Medium1Motor", medium1Motor.getMotorOutputPercent());
        SmartDashboard.putNumber("Medium2Motor", medium2Motor.getMotorOutputPercent());

    }

    /*public void launcherSpeaker(double speed){
        launcherDelayed1(speed);
        launcherDelayed2(speed);
        speaker = true;
        on = true;
    }

    public void launcherReturn(){

        medium1Motor.set(ControlMode.PercentOutput, -0.6);
        medium2Motor.set(ControlMode.PercentOutput, -0.6);
    }

    public void launcherAmp(){

        medium1Motor.set(ControlMode.PercentOutput, 1/3.5);
        medium2Motor.set(ControlMode.PercentOutput, 1/3.5);
        down1Motor.set(1/3.5);

        speaker = false;
        on = true;
    }

    public void launcherShooterOff(){
        medium1Motor.set(ControlMode.PercentOutput,0);
        medium2Motor.set(ControlMode.PercentOutput,0);
        down1Motor.set(0);
        on = false;
    }

    public void launcherDelayed1(double speed){

        medium1Motor.set(ControlMode.PercentOutput, speed);
        medium2Motor.set(ControlMode.PercentOutput, speed);
    }

    public void launcherDelayed2(double speed) {
        down1Motor.set(speed);
    }

    public void launcherCapSync(){
        down1Motor.set(0.7);
        on = true;
    }*/

    public void firstStep(){
        medium1Motor.set(ControlMode.PercentOutput, 0.8);
        medium2Motor.set(ControlMode.PercentOutput, 0.8);
        on = true;
    }

    public void stopAll(){
        medium1Motor.set(ControlMode.PercentOutput, 0);
        medium2Motor.set(ControlMode.PercentOutput, 0);

        on = false;
    }
    private void initShuffleboard(){

    }
}