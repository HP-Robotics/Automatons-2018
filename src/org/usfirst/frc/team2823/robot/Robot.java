/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	Joystick joystick;
	
	final int xButton = 1;
	final int aButton = 2;
	final int bButton = 3;
	final int yButton = 4;
	final int leftBumper = 5;
	final int rightBumper = 6;
	final int leftTrigger = 7;
	final int rightTrigger = 8;
	
	Button intakeOpenButton;
	Button intakeCubeButton; 
	Button intakeStowButton;
	Button dropCubeButton;
	Button gearHighButton;
	Button gearLowButton;
	
	Button testButton;

	TalonSRX leftMotor1;
	TalonSRX leftMotor2;
	TalonSRX rightMotor3;
	TalonSRX rightMotor4;
	
	TalonSRX leftElbow;
	TalonSRX rightElbow;
	TalonSRX leftBelt;
	TalonSRX rightBelt;
	
	TalonSRX fourbarMotor;
	TalonSRX elevatorMotor;

	DoubleSolenoid solenoid1;
	Compressor compressor;
	
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	Encoder lIntakeEncoder;
	Encoder rIntakeEncoder;
	Encoder fourbarEncoder;
	Encoder elevatorEncoder;
	
	DriveInchesPIDSource leftInches;
	DriveInchesPIDSource rightInches;
	
	
	SnazzyPIDController leftPIDControl;
	SnazzyPIDController rightPIDControl;
	
	SnazzyMotionPlanner leftControl;
	SnazzyMotionPlanner rightControl;
	
	LeftDrivePIDOutput lDriveOutput;
	RightDrivePIDOutput rDriveOutput;
	
	TrajectoryPlanner traj;
	
	SendableChooser<Autonomous> autonomousChooser;
	
	double maxMotorPower = .8; // maximum motor power output for 775's - owen said 0.8, but trying 
	double motorRampRate = 36;// owen
	//double lastShift;
	
	//final double ENC_TO_IN = 2 * Math.PI * WHEEL_RADIUS * DRIVE_RATIO / ENCODER_RESOLUTION;
	//final double IN_TO_ENC = ENCODER_RESOLUTION / (2 * Math.PI * WHEEL_RADIUS * DRIVE_RATIO);
	
	DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kForward;
	
	double wheelDiameter = 4;
	double highGearDistancePerRev = (wheelDiameter * Math.PI) / 12.2646604938; // 12.2646604938 775 turns per wheel rev
	double lowGearDistancePerRev = (wheelDiameter * Math.PI) / 30.9375; // 30.9375 775 turns per wheel rev
	double highGearDistancePerPulse = (highGearDistancePerRev / 12)*((15+5/8)/4); // 12 pulses per rev
	double lowGearDistancePerPulse = (lowGearDistancePerRev / 12)*((15+5/8)/4); // 12 pulses per rev; ((15-5/8)/4) multiplier  because Owen probably did his math wrong
	
	final static double ENC_TO_INCH = 0.01;
	final static double INCH_TO_ENC = 100;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		autonomousChooser = new SendableChooser<Autonomous>();
		//autonomousChooser.addDefault("Empty: Do Nothing", new EmptyAutonomous(this));
		autonomousChooser.addObject("Cross Baseline", new GoGoFallbotAuto(this));
		autonomousChooser.addObject("Do a Spin", new SpinnyAuto(this));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
		/** UsbCamera c = CameraServer.getInstance().startAutomaticCapture();
	        c.setResolution(320, 180);
	        c.setFPS(29);
	        **/
		joystick = new Joystick(0);

		intakeOpenButton = new Button();
		intakeCubeButton = new Button();
		intakeStowButton = new Button();
		dropCubeButton = new Button();
		gearHighButton = new Button();
		gearLowButton = new Button();
		
		testButton = new Button();
		
		leftMotor1 = new TalonSRX(1);
		leftMotor2 = new TalonSRX(2);
		rightMotor3 = new TalonSRX(3);
		rightMotor4 = new TalonSRX(4);
		
		leftMotor1.configOpenloopRamp(0, 0);
		leftMotor2.configOpenloopRamp(0, 0);
		rightMotor3.configOpenloopRamp(0, 0);
		rightMotor4.configOpenloopRamp(0, 0);
		
		leftElbow = new TalonSRX(5);
		rightElbow = new TalonSRX(6);
		leftBelt = new TalonSRX(7);
		rightBelt = new TalonSRX(8);
		
		fourbarMotor = new TalonSRX(9);
		elevatorMotor = new TalonSRX(10);
			
		lDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		lDriveEncoder.setDistancePerPulse(1);
		rDriveEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		rDriveEncoder.setDistancePerPulse(1);
		
		lIntakeEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		rIntakeEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		
		leftInches = new DriveInchesPIDSource(lDriveEncoder);
		rightInches = new DriveInchesPIDSource(rDriveEncoder);
		
		fourbarEncoder = new Encoder(8,9, false, Encoder.EncodingType.k4X);
		elevatorEncoder = new Encoder( 10, 11, false, Encoder.EncodingType.k4X);

		//leftEncoder.setSamplesToAverage(20);
		//rightEncoder.setSamplesToAverage(20);

		solenoid1 = new DoubleSolenoid(0, 1);
		compressor = new Compressor(0);

		solenoid1.set(lowGear);

		lDriveEncoder.reset();
		rDriveEncoder.reset();
		lIntakeEncoder.reset();
		rIntakeEncoder.reset();
		fourbarEncoder.reset();
		elevatorEncoder.reset();
	
		lDriveOutput = new LeftDrivePIDOutput(this);
		rDriveOutput = new RightDrivePIDOutput(this);
		
		leftControl = new SnazzyMotionPlanner(0.1, 0.001, 0.3, 0, 0.00143, 0.0102,  leftInches, lDriveOutput, 0.005, "Left.csv");
		rightControl= new SnazzyMotionPlanner(0.1, 0.001, 0.3, 0, 0.00143, 0.0102,  rightInches, rDriveOutput, 0.005,"Right.csv");
		
		leftPIDControl = new SnazzyPIDController(0.04, 0.001, 0.8, 0, leftInches, lDriveOutput, 0.005, "Left.csv");
		rightPIDControl= new SnazzyPIDController(0.04, 0.001, 0.8, 0, rightInches, rDriveOutput, 0.005,"Right.csv");
		
		SmartDashboard.putNumber("P", 0.01);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		SmartDashboard.putNumber("Setpoint", 0);
		SmartDashboard.putNumber("L Encoder", 0);
		SmartDashboard.putNumber("R Encoder", 0);
		SmartDashboard.putNumber("L Elbow", 0);
		SmartDashboard.putNumber("R Elbow", 0);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		
		lDriveEncoder.reset();
		rDriveEncoder.reset();
		lIntakeEncoder.reset();
		rIntakeEncoder.reset();
		fourbarEncoder.reset();
		elevatorEncoder.reset();
		
		leftControl.reset();
		rightControl.reset();
		
		solenoid1.set(highGear);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		solenoid1.set(lowGear);
		
		((Autonomous) autonomousChooser.getSelected()).init();
		
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		((Autonomous) autonomousChooser.getSelected()).periodic();
		
		SmartDashboard.putNumber("L Encoder", lDriveEncoder.get());
		SmartDashboard.putNumber("R Encoder", rDriveEncoder.get());
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		gearLowButton.update(joystick.getRawButton(leftTrigger));
		gearHighButton.update(joystick.getRawButton(leftBumper));
		intakeOpenButton.update(joystick.getRawButton(rightBumper));
		intakeCubeButton.update(joystick.getRawButton(rightTrigger));
		intakeStowButton.update(joystick.getRawButton(xButton));
		dropCubeButton.update(joystick.getRawButton(bButton));
		testButton.update(joystick.getRawButton(1));

		if (gearLowButton.changed()) {
			solenoid1.set(lowGear);
			System.out.println("low");
			//leftEncoder.setDistancePerPulse(lowGearDistancePerPulse);
			//rightEncoder.setDistancePerPulse(lowGearDistancePerPulse);

		}
		if (gearHighButton.changed()) {
			solenoid1.set(highGear);
			System.out.println("high");
			//leftEncoder.setDistancePerPulse(highGearDistancePerPulse);
			//rightEncoder.setDistancePerPulse(highGearDistancePerPulse);

			
		}
		
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
		SmartDashboard.putNumber("L Elbow", rDriveEncoder.get());
		SmartDashboard.putNumber("R Elbow", rDriveEncoder.get());
		//System.out.println(lowGearDistancePerPulse);
		
		leftMotor1.set(ControlMode.PercentOutput, Math.pow(joystick.getRawAxis(1), 1) );
		leftMotor2.set(ControlMode.PercentOutput, Math.pow(joystick.getRawAxis(1), 1) );
		rightMotor3.set(ControlMode.PercentOutput, -Math.pow(joystick.getRawAxis(3), 1));
		rightMotor4.set(ControlMode.PercentOutput, -Math.pow(joystick.getRawAxis(3), 1));
		
		/*leftPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		rightPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		
		if(testButton.on()){
			if(testButton.changed()) {
				lDriveEncoder.reset();
				rDriveEncoder.reset();
				
				//leftControl.configureGoal(SmartDashboard.getNumber("Setpoint", 0), 300, 300);
				//rightControl.configureGoal(SmartDashboard.getNumber("Setpoint", 0), 300, 300);
				
				//leftControl.enable();
				//rightControl.enable();
				
				leftPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
				leftPIDControl.enable();
				
				rightPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
				rightPIDControl.enable();

			}
			
		}else if (testButton.changed()&& !testButton.on()){
			leftPIDControl.disable();
			rightPIDControl.disable();
			
		}*/
		
		//This thing maybe
		//motorx.set(joystick.getRawAxis(3) > maxSpeed ? joystick.getRawAxis(3) : 0.8)
		//System.out.println(leftEncoder.getPeriod() + " " + rightEncoder.getPeriod());
//System.out.println(motor3.getOutputCurrent());
	}

	/*
	 * This function is called periodically during test mode.
	 */

	@Override
	public void testInit() {
		leftControl.disable();
		rightControl.disable();
	}

	@Override
	public void testPeriodic() {
		testButton.update(joystick.getRawButton(1));
		
		
		leftControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		rightControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		
		if(testButton.on()){
			if(testButton.changed()) {
				lDriveEncoder.reset();
				rDriveEncoder.reset();
				
				//leftControl.configureGoal(SmartDashboard.getNumber("Setpoint", 0), 300, 300);
				//rightControl.configureGoal(SmartDashboard.getNumber("Setpoint", 0), 300, 300);
				
				//leftControl.enable();
				//rightControl.enable();
				
				leftControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
				leftControl.enable();
				
				//rightControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
				//rightControl.enable();

			}
			
		}else if (testButton.changed()&& !testButton.on()){
			leftControl.disable();
			//rightControl.disable();
			
			
		}
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
	}
	public void disabledInit() {
		//System.out.println("disabled");
	}
}
