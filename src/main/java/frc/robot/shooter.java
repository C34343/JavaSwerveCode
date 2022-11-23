package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import java.lang.Math;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class shooter {
    static double COUNTS_PER_METER = 51213;
    private final static TalonFX shooterMotor1 = new TalonFX(10); 
    private final static  TalonFX shooterMotor2 = new TalonFX(9); 

    public final static CANSparkMax topNeckMotor = new CANSparkMax(21, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final static CANSparkMax bottomNeckMotor = new CANSparkMax(20, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private final static PIDController m_shooterPIDController = new PIDController(0.0001, 0, 0);
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
    
    public static void reverseBottomNeckMotor(){
        bottomNeckMotor.set(-.6);
    }
    public static void  runNeckMotors (){
        topNeckMotor.set(.6);
        bottomNeckMotor.set(.6);
    }
    public static void  stopNeckMotors (boolean intakeToggle){
        topNeckMotor.set(0.0);

        if (!intakeToggle) {
            bottomNeckMotor.set(0.0);
        }
    }

    public static void startBottomNeckMotor () {
        bottomNeckMotor.set(0.6);
    }

    public static void shooterInit(){
        shooterMotor1.setInverted(true);
    }
    public static void startShooter(int desiredRPM) {
        // final double shooterOutput = m_shooterPIDController.calculate(shooterMotor1.getSelectedSensorVelocity() / COUNTS_PER_METER, desiredRPM);
        shooterMotor1.set(ControlMode.Velocity, (desiredRPM * 2048) / 600);
        shooterMotor2.set(ControlMode.Velocity, (desiredRPM * 2048) / 600);
    }

    public static void stopShooter () {
        shooterMotor1.set(ControlMode.PercentOutput, 0.0);
        shooterMotor2.set(ControlMode.PercentOutput, 0.0);
    }
    public static boolean checkShooterSpeed (int desiredRPM) {
        double currentVelocity = shooterMotor1.getSelectedSensorVelocity() / 2048 * 600;
        if (Math.abs(desiredRPM - currentVelocity) < 250){ 
            return true; 
        }else {
            return false;
        }
    }

}

