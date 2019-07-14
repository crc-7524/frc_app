/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    //private final DifferentialDrive m_robotDrive
    //    = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    private final SpeedControllerGroup m_left = new SpeedControllerGroup(new PWMVictorSPX(0), new PWMVictorSPX(1));
    private final SpeedControllerGroup m_right = new SpeedControllerGroup(new PWMVictorSPX(2), new PWMVictorSPX(3));
    private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    private final Joystick m_stick = new Joystick(0);
    private final Timer m_timer = new Timer();
    private static int circle = 0;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        CameraServer.getInstance().startAutomaticCapture(0);
    }

    /**
     * This function is run once each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
        m_timer.reset();
        m_timer.start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // Drive for 2 seconds
        if (m_timer.get() < 2.0) {
            m_drive.arcadeDrive(0.5, 0.0); // drive forwards half speed
        } else {
            m_drive.stopMotor(); // stop robot
        }
    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
      double x = m_stick.getX();
      double y = m_stick.getY();
      m_drive.arcadeDrive(m_stick.getY() /4*3, m_stick.getX()/4*3 );
      if (circle == 24) {
        circle = 0;
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        //System.out.println("X: "+x);
        //System.out.println("y: "+y);
      }else circle ++;
      /*double[] speed = getSpeed(m_stick.getX(), m_stick.getY());
      System.out.println("output x: "+speed[0]);
      System.out.println("output y: "+speed[1]);
      m_left.set(-speed[0]/4);
      m_right.set(-speed[1]/4);*/
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    private static double[] getSpeed(double x, double y) {
      double speedL, speedR;
      if (x != 0 && y != 0) {
          double ax = Math.abs(x);
          double ay = Math.abs(y);
          int sign = y > 0 ? 1 : -1;
          double r = sign * (Math.toDegrees(Math.atan2(ay, ax)) - 45) / 45;
          double s = sign * (ax >= ay ? (Math.sqrt(2) * ay) : (Math.sqrt(2) * ax));
          if (x * y > 0) {
              speedL = r;
              speedR = -s;
          } else {
              speedL = s;
              speedR = -r;
          }
      } else if (x == 0 && y == 0) {
          speedL = 0;
          speedR = 0;
      } else if (x == 0) {
          speedL = y;
          speedR = -y;
      } else {
          speedL = -x;
          speedR = -x;
      }
      double[] speed = {speedL, speedR};
      return speed;
  }

}
