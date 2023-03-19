package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.Timer;
import java.util.TimerTask;

import com.kauailabs.navx.frc.AHRS;

public class BalanceOnChargingStation extends CommandBase {
    
    //protected double startBalancePoint = 7.5;
    protected double balancePoint = 3.0;
    protected double maxAngle = 15.0;
    protected double currentAngle;
    protected DriveSubsystem driveSubsystem;
    protected double balanceRatio;
    protected boolean isMoving;
    protected boolean wasMoving;
    protected boolean hasNotMoved1s = false;
    //TEMP VALUE
    protected double balanceSpeed = 0.375;

    
    //protected DriveSubsystem m_robotDrive;

    public BalanceOnChargingStation(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        wasMoving = false;
    }

    @Override
    public void execute() {
        currentAngle = driveSubsystem.getPitch();
        wasMoving = isMoving;
        if (currentAngle > balancePoint || currentAngle < -1 * balancePoint) {
            isMoving = true;
            balanceRatio = Math.abs(currentAngle / maxAngle);
            if (balanceRatio > 0) {
                driveSubsystem.setSpeed(() -> /*-1 * */ balanceRatio * balanceSpeed, () -> 0, () -> true);
            } else if (balanceRatio < 0) {
                driveSubsystem.setSpeed(() -> -1 * balanceRatio * balanceSpeed, () -> 0, () -> true);
            }
        } else {
            isMoving = false;
            if (wasMoving = false) {
                end(false);
            } else {
                Timer timer = new Timer();
                timer.schedule(checkIfMoving(), 1000);
            }
        }

    }

    public TimerTask checkIfMoving() {
        if (isMoving == false && Math.abs(currentAngle) > balancePoint) {

        }
        return new TimerTask() {
            @Override
            public void run() {
                if (isMoving == false && Math.abs(currentAngle) > balancePoint) {
                    end(false);
                } else {
                    isMoving = true;
                }
            }
        };
    }

    public void hibernate() {
        end(false);
    }

    @Override
    public void end(boolean interrupted) {
        this.cancel();
    }
      
    
    /* 
    public CommandBase DoBalanceOnChargingStation(DriveSubsystem driveSubsystem) {
        // Dumbass way to do this not using PID, probably won't work
        currentAngle = driveSubsystem.getPitch();
        if (currentAngle > balancePoint || currentAngle < -1 * balancePoint) {
            double pitchAngleRadians = driveSubsystem.getPitch() * (Math.PI / 180.0);
            double xAxisRate = -1 * Math.sin(pitchAngleRadians);
            System.out.println("Pitch in radians is " + pitchAngleRadians);
            return Commands.run(() -> driveSubsystem.arcadeDrive(xAxisRate, 0), driveSubsystem);
        } else {
            return Commands.runOnce(driveSubsystem::stop, driveSubsystem);
        }

    }
    */
        
    

    /*public void execute(DriveSubsystem driveSub) {
        m_robotDrive = driveSub;
        while (currentAngle > balancePoint) {
            Commands.runOnce(() -> DoBalanceOnChargingStation());
            currentAngle = Math.abs(driveSub.getPitch());
        }
    }*/

}