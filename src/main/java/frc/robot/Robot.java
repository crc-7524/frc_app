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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  //SerialPort m_testSerial = new SerialPort(115200, Port.kUSB);

  Timer m_Timer = new Timer();

  AnalogInput m_switcher = new AnalogInput(0);

  TalonSRX m_LifterMain = new TalonSRX(2);
  TalonSRX m_LifterFollower = new TalonSRX(3);
/*
  private Joystick m_leftJoystick = new Joystick(1);
  private Joystick m_rightJoystick = new Joystick(0);

*/
  //private Joystick m_lifterJoystick = new Joystick(2);
  //private Joystick m_testJoystick = new Joystick(3);
  private Joystick m_xbox = new Joystick(2);
  private Joystick m_testJoystick2 = new Joystick(4);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private double kZeroPosition = 0;
  private double kSetPoint = 0;
  private double kJoystickGain = 1.75;
  private double kNormalRate = 0.55;
  private double kRate = kNormalRate;
  private double kFastRate = 0.9;

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

  private MyPID m_lifterPID = new MyPID("lifterPID");

  DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(0, 1);

  PowerDistributionPanel m_pdp = new PowerDistributionPanel();

  private boolean kLastSolenoidBtn = false;
  private boolean kSolenoid = true;//"True" means pull up or forward.

  private double kIntakeGreyPower = 0.95;
  private double kIntakeGreenPower = 0.5;

  // proportional speed constant
  private double kP = 0.035;//,kPIncreace = 0.01/ 100;
  private double kI = 0.0;    //,kIIncreace = 0.0001 / 100;
  private double kD = 0.048;//,kDIncreace = 0.001 / 100;
  
  Pixy2 pixy = Pixy2.createInstance(LinkType.SPI);

  MyPID m_pixyPID = new MyPID("pixy");

  private double kRightSpeed;
  private double kLeftSpeed;

  @Override
  public void robotInit() {
    pixy.init();

    m_pixyPID.setPID(0.015, 0, 0, 0);

    System.out.println("[status]robotInit() is running.");
    CameraServer.getInstance().startAutomaticCapture();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_lifterPID.setPID(kP, kI, kD, 0);
    m_lifterPID.setOutputRange(0.95, -0.3);

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
    System.out.println("robotInit, double solenoid set forward.");
    m_doubleSolenoid.set(Value.kReverse);
  }

  @Override
  public void robotPeriodic() {
    //Press right shoot to collect objects.
    if(m_testJoystick2.getRawButton(6)){
      m_intakeGreen.set(kIntakeGreenPower);
      m_intakeGrey.set(kIntakeGreyPower);
      SmartDashboard.putString("Intake status", "Collecting, ");
    }else if(m_testJoystick2.getRawButton(5)){ //Press left key 4 to throw objects.
      m_intakeGreen.set(-kIntakeGreenPower);
      m_intakeGrey.set(-kIntakeGreyPower);
      SmartDashboard.putString("Intake status", "Throwing, ");
    }else{
      m_intakeGreen.stopMotor();
      m_intakeGrey.stopMotor();;
    }

    //Press button 4 to change solenoid's status
    boolean solenoidBtn = m_testJoystick2.getRawButton(4);
    if(!kLastSolenoidBtn && solenoidBtn){
      kSolenoid = !kSolenoid;
      if(kSolenoid){
        m_doubleSolenoid.set(Value.kForward);
      }else{
        m_doubleSolenoid.set(Value.kReverse);
      }
    }
    kLastSolenoidBtn = solenoidBtn;
    SmartDashboard.putBoolean("Solenoid", kSolenoid);

    //Fast and slow btn.
    if(m_xbox.getRawButton(5)){
      kRate = kFastRate;
    }else{
      kRate = kNormalRate;
    }
    
    kRightSpeed = joystickOutput(-m_xbox.getRawAxis(5), 0.05) * kRate;
    kLeftSpeed = joystickOutput(-m_xbox.getRawAxis(1), 0.05) * kRate;
    
    SmartDashboard.putNumber("left speed", kLeftSpeed);
    SmartDashboard.putNumber("right speed", kRightSpeed);

    //Follow
    m_leftFollower1.set(ControlMode.PercentOutput, m_leftMain.getMotorOutputPercent());
    m_rightFollower1.set(ControlMode.PercentOutput, m_rightMain.getMotorOutputPercent());

    SmartDashboard.putNumber("Left 1 current", m_pdp.getCurrent(0));
    SmartDashboard.putNumber("Left 2 current", m_pdp.getCurrent(1));
    SmartDashboard.putNumber("Left 3 current", m_pdp.getCurrent(2));
    SmartDashboard.putNumber("Right 1 current", m_pdp.getCurrent(15));
    SmartDashboard.putNumber("Right 2 current", m_pdp.getCurrent(14));
    SmartDashboard.putNumber("Right 3 current", m_pdp.getCurrent(13));

  }

  @Override
  public void autonomousInit() {
    System.out.println("[status]autonomousInit is running.");
    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_lifterPID.resetError();
    kZeroPosition = m_LifterMain.getSelectedSensorPosition();
  }

  /**
   * This function is called periodically during autonomous.
   */ 
  @Override
  public void autonomousPeriodic() {
    /*
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    */
    double mid = 158;
    int ret = pixy.getCCC().getBlocks(false, 1, 2);
    if(ret > 0){
      System.out.printf("Deceted %d:", ret);
      for(int i = 0; i < ret; ++i){
        System.out.printf("[%d]: x = %d", i+1, pixy.getCCC().getBlocks().get(i).getX());
      }
      System.out.println("");
      if(ret == 2){
        SmartDashboard.putString("Pixy status", "Deceted 2 successfully.");
        mid = (double)( pixy.getCCC().getBlocks().get(0).getX() + pixy.getCCC().getBlocks().get(1).getX() ) / 2; 
      }else{
        SmartDashboard.putString("Pixy status", "Number of bolcks is " + Integer.toString(ret));
      }
    }else if(ret == Pixy2.PIXY_RESULT_OK){
      SmartDashboard.putString("Pixy status", "Result OK, blocks not found.");
    }else if(ret == Pixy2.PIXY_RESULT_BUSY){
      SmartDashboard.putString("Pixy status", "data is not available");
    }else if(ret == Pixy2.PIXY_RESULT_PROG_CHANGING){
      SmartDashboard.putString("Pixy status", "RESULT_PROG_CHANGING");
    }else{
      SmartDashboard.putString("Pixy status", "Unknown error, ret=" + Integer.toString(ret));
    }
    double turnSpeed = m_pixyPID.calc(mid, 158.0);

    if(m_xbox.getRawAxis(3) > 0.5){
      m_robot.arcadeDrive(kRightSpeed, turnSpeed);   //CV PID control turn speed.
    }else if(m_xbox.getRawButton(6)){
      m_robot.arcadeDrive(kRightSpeed, 0);    //Drive straight when right button 4 is hold.
    }else{
      m_robot.tankDrive(kLeftSpeed, kRightSpeed);     //Tank drive.
    }
  }

  @Override
  public void teleopPeriodic() {

    if(m_xbox.getRawButton(6)){
      m_robot.arcadeDrive(kRightSpeed, 0);    //Drive straight when right button 4 is hold.
    }else{
      m_robot.tankDrive(kLeftSpeed, kRightSpeed);     //Tank drive.
    }

/*
    if(m_Joystick.getRawButton(8)){kP += kPIncreace;}if(m_Joystick.getRawButton(7)){kP -= kPIncreace;}
    if(m_Joystick.getRawButton(10)){kI += kIIncreace;}if(m_Joystick.getRawButton(9)){kI -= kIIncreace;}
    if(m_Joystick.getRawButton(12)){kD += kDIncreace;}if(m_Joystick.getRawButton(11)){kD -= kDIncreace;}
*/
    if(m_testJoystick2.getRawButton(11)){
      m_lifterPID.resetError();
    }
 
    /*
    //Press "left bumber" to set zero position of the lifter
    if(m_lifterJoystick.getRawButton(7)){
      kZeroPosition = m_LifterMain.getSelectedSensorPosition();
    }
    */
    double processVar = 1 * (m_LifterMain.getSelectedSensorPosition() - kZeroPosition) * 0.004026699568199808; // = / 4096 * 5.25 * Math.PI;
    
    /*
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
    }else if(kLastLifterBtn){
      kSetPoint = processVar;
      kLastLifterBtn = false;
    }
    */

    //Test the lifter.
    kSetPoint = ( -m_testJoystick2.getRawAxis(3) + 1 ) * 92;
    double output = m_lifterPID.calc(kSetPoint, processVar);
    m_LifterMain.set(ControlMode.PercentOutput, output);

    SmartDashboard.putNumber("LifterMain Current", m_LifterMain.getOutputCurrent());
    SmartDashboard.putNumber("LifterFollower Current", m_LifterFollower.getOutputCurrent());
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