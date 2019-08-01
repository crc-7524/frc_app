/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ckcyberpack.lib.FRCPixy2;
import com.ckcyberpack.lib.FRCPixyBlock;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  //SerialPort m_testSerial = new SerialPort(115200, Port.kUSB);

  Timer m_Timer = new Timer();

  FRCPixy2 m_pixy2 = new FRCPixy2(edu.wpi.first.wpilibj.SPI.Port.kOnboardCS0);

  AnalogInput m_switcher = new AnalogInput(0);

  TalonSRX m_LifterMain = new TalonSRX(2);
  TalonSRX m_LifterFollower = new TalonSRX(3);

  private Joystick m_leftJoystick = new Joystick(0);
  private Joystick m_rightJoystick = new Joystick(1);
  private Joystick m_lifterJoystick = new Joystick(2);
/*
  private Joystick m_testJoystick = new Joystick(3);
  private Joystick m_testJoystick2 = new Joystick(4);
*/
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private double kZeroPosition = 0;
  private double kSetPoint = 0;
  private double kJoystickGain = 1.8;
  private double kSlowRate = 0.75;

  private WPI_TalonSRX m_leftMain = new WPI_TalonSRX(0);
  private VictorSPX m_leftFollower1 = new VictorSPX(4);
  private VictorSPX m_leftFollower2 = new VictorSPX(5);

  private WPI_TalonSRX m_rightMain = new WPI_TalonSRX(1);
  private VictorSPX m_rightFollower1 = new VictorSPX(6);
  private VictorSPX m_rightFollower2 = new VictorSPX(7);

  private PWMVictorSPX m_intakeGreen = new PWMVictorSPX(5);
  private PWMVictorSPX m_intakeGrey = new PWMVictorSPX(1);

  DifferentialDrive m_robot = new DifferentialDrive(m_leftMain, m_rightMain);

  Compressor m_compressor = new Compressor(0);
  
  // proportional speed constant
  private double kP = 0.025;//,kPIncreace = 0.01/ 100;
  private double kI = 0.0001;    //,kIIncreace = 0.0001 / 100;
  private double kD = 0.045;//,kDIncreace = 0.001 / 100;

  private MyPID m_lifterPID = new MyPID("lifterPID");

  DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(0, 1);

  private boolean kLastBtnX = false;
  private boolean kLastSwitcher = false;
  private boolean kSolenoid = true;//"True" means pull up or forward.

  private double kIntakePower = 0.35;

  private boolean kLastLifterBtn = false;

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_lifterPID.setPID(kP, kI, kD, 0);
    m_lifterPID.setOutputRange(0.65, -0.2);

    m_LifterMain.configFactoryDefault();
    m_LifterFollower.configFactoryDefault();

    m_LifterFollower.follow(m_LifterMain);

    kZeroPosition = m_LifterMain.getSelectedSensorPosition();
    
    m_Timer.reset();
    m_Timer.start();

    m_leftMain.configFactoryDefault();
    m_leftFollower1.configFactoryDefault();
    m_leftFollower2.configFactoryDefault();

    m_rightMain.configFactoryDefault();
    m_rightFollower1.configFactoryDefault();
    m_rightFollower2.configFactoryDefault();

    m_rightMain.setInverted(true);
    m_rightFollower1.setInverted(true);
    m_rightFollower2.setInverted(true);

    m_leftFollower2.follow(m_leftFollower1);
    m_rightFollower2.follow(m_rightFollower1);

    m_robot.setRightSideInverted(false);

    m_compressor.setClosedLoopControl(true);

    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */ 
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopPeriodic() {
/*
    if(m_Joystick.getRawButton(8)){kP += kPIncreace;}if(m_Joystick.getRawButton(7)){kP -= kPIncreace;}
    if(m_Joystick.getRawButton(10)){kI += kIIncreace;}if(m_Joystick.getRawButton(9)){kI -= kIIncreace;}
    if(m_Joystick.getRawButton(12)){kD += kDIncreace;}if(m_Joystick.getRawButton(11)){kD -= kDIncreace;}
*/

    if(m_rightJoystick.getRawButton(3)){      //Fast key
      kSlowRate = 1.0;
    }else if(m_rightJoystick.getRawButton(4)){//Slow key
      kSlowRate = 0.5;
    }

    //Drive
    m_robot.tankDrive(joystickOutput(-m_leftJoystick.getY(), 0.1) * kSlowRate, -joystickOutput(-m_rightJoystick.getY(), 0.1) * kSlowRate);

    //Follow
    m_leftFollower1.set(ControlMode.PercentOutput, m_leftMain.getMotorOutputPercent());
    m_rightFollower1.set(ControlMode.PercentOutput, m_rightMain.getMotorOutputPercent());
    
    //Press "left bumber" to set zero position of the lifter
    if(m_lifterJoystick.getRawButton(7)){
      kZeroPosition = m_LifterMain.getSelectedSensorPosition();
    }

    //Press right shoot to collect objects.
    if(m_rightJoystick.getRawButton(1)){
      m_intakeGreen.set(kIntakePower);
      m_intakeGrey.set(kIntakePower);
      SmartDashboard.putString("Intake status", "Collecting, " + kIntakePower);
    }    
    
    //Press left key 3 to throw objects.
    else if(m_leftJoystick.getRawButton(3)){
      m_intakeGreen.set(-kIntakePower);
      m_intakeGrey.set(-kIntakePower);
      SmartDashboard.putString("Intake status", "Throwing, " + kIntakePower);
    }
    
    double processVar = 1 * (m_LifterMain.getSelectedSensorPosition() - kZeroPosition) * 0.004026699568199808; // = / 4096 * 5.25 * Math.PI;
    
    if(m_lifterJoystick.getRawButton(1)){
      kSetPoint = 30;
      kLastLifterBtn = true;
    }else if(m_lifterJoystick.getRawButton(2)){
      kSetPoint = 45;
      kLastLifterBtn = true;
    }else if(m_lifterJoystick.getRawButton(3)){
      kSetPoint = 60;
      kLastLifterBtn = true;
    }else if(m_lifterJoystick.getRawButton(4)){
      kSetPoint = 75;
      kLastLifterBtn = true;
    }else if(m_lifterJoystick.getRawButton(5)){
      kSetPoint = 90;
      kLastLifterBtn = true;
    }else if(m_lifterJoystick.getRawButton(6)){
      kSetPoint = 105;
      kLastLifterBtn = true;
    }else if(kLastBtnX){
      kSetPoint = processVar;
      kLastBtnX = false;
    }

    double output = m_lifterPID.calc(kSetPoint, processVar);

    m_LifterMain.set(ControlMode.PercentOutput, output);

    //Test lifter.
    //kSetPoint = ( -m_testJoystick2.getRawAxis(3) + 1 ) * 90;

    SmartDashboard.putNumber("LifterMain Current", m_LifterMain.getOutputCurrent());
    SmartDashboard.putNumber("LifterFollower Current", m_LifterFollower.getOutputCurrent());

    //Press key to change solenoid's status
    boolean btnX = m_leftJoystick.getRawButton(3);
    if(!kLastBtnX && btnX){
      kSolenoid = !kSolenoid;
      if(kSolenoid){
        m_doubleSolenoid.set(Value.kForward);
      }else{
        m_doubleSolenoid.set(Value.kReverse);
      }
    }
    kLastBtnX = btnX;
    SmartDashboard.putBoolean("Solenoid", kSolenoid);

/*
    if(kLastSwitcher && !m_switcher.get()){
      kZeroPosition = m_LifterMain.getSensorCollection().getPulseWidthPosition();
    }
    kLastSwitcher = m_switcher.get();
 */

    SmartDashboard.putNumber("Limit Switcher", m_switcher.getValue());

    FRCPixyBlock pixyBlock = m_pixy2.getBlocks(0, 0);
    pixyBlock.getX();
  }

  @Override
  public void testPeriodic() {
  }

  public double joystickOutput(double input, double minOutput){
    double ret = Math.copySign( Math.pow(Math.abs(input),kJoystickGain), input);
    if(Math.abs(ret) > minOutput){
      return ret;
    }else{
      return 0;
    }
  }
}