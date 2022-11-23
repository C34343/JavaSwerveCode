package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class auto {
    private static final DigitalInput bottomBreakbeam = new DigitalInput(4);
    private static final DigitalInput topBreakbeam = new DigitalInput(5);
    private static final Timer timer  = new Timer();
    static int step = 1;

    public static void oneBall(){
        
        switch(step){
            case 1:
                shooter.startShooter(2300);
                if(shooter.checkShooterSpeed(2100)){
                    step = 2;
                }
                break;
            case 2:
                shooter.runNeckMotors();
                timer.reset();
                timer.start();
                step = 3;
                break;
            case 3:
                if(timer.get()>2){
                    timer.stop();
                    step = 4;
                }
                break;
            case 4:
                shooter.stopShooter();
                shooter.stopNeckMotors(false);
                timer.reset();
                timer.start();
                step = 5;
                break;
            case 5:
                Robot.m_swerve.drive(-1.5, 0, 0, true);
                step = 6;
                break;
            case 6:
                if(timer.get() > 4){
                    timer.stop();
                    Robot.m_swerve.drive(0, 0, 0, true);
                    step = 7;
                } else {
                    Robot.m_swerve.drive(-1.5, 0, 0, true);
                }
                break;
            case 7:
                break;

        }
            System.out.println("Step: " + step);
            System.out.println("TBB: " + topBreakbeam.get());
            System.out.println("BBB: " + bottomBreakbeam.get());
    }
}
