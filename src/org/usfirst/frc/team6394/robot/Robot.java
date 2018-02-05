/**
 * A bare-bones test project using Pigeon for "Go-Straight" servo-ing.
 * The goal is for a basic robot with left/right side drive
 * to automatically hold a heading while driver is holding the top-left
 * shoulder button (Logitech Gamepad).
 *
 * If Pigeon is present on CANbus, or ribbon-cabled to a CAN-Talon, the robot will use the IMU to servo.
 * If Pigeon is not present, robot will simply apply the same throttle to both sides.
 *
 * When developing robot applications with IMUs, it's important to design in what happens if
 * the IMU is disconnected or un-powered.
 */
package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
//import edu.wpi.first.wpilibj.Joysticklogi;
//import edu.wpi.first.wpilibj.Joysticklogi.AxisType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends IterativeRobot {

 
	/* robot peripherals */
	private XboxController sticklogi = new XboxController(0);
	private XboxController stickbeitong = new XboxController(1);
	private TalonSRX t_l = new TalonSRX(0);
	private TalonSRX t_r = new TalonSRX(2);
	private VictorSPX v_l = new VictorSPX(1);
	private VictorSPX v_r = new VictorSPX(3);
	private Talon i_l = new Talon(0);
	private Talon i_r = new Talon(1);
	private Talon lift_l = new Talon(2);
	private Talon lift_r = new Talon(3);
	
	
	private AHRS ahrs = new AHRS(Port.kMXP);		/* Joysticklogi object on USB port 1 */

	/** state for tracking whats controlling the drivetrain */
	enum GoStraight
	{
		Off, UseAHRS, SameThrottle
	};

	GoStraight _goStraight = GoStraight.Off;

	/*
	 * Some gains for heading servo, these were tweaked by using the web-based
	 * config (CAN Talon) and pressing gamepad button 6 to load them.
	 */
	double kPgain = 0.05; /* percent throttle per degree of error */
	double kDgain = 0; /* percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 0.30; /* cap corrective turning throttle to 30 percent of forward throttle */
	/** holds the current angle to servo to */
	double _targetAngle = 0;
	/** count loops to print every second or so */
	int _printLoops = 0;

	public Robot() {
		
		t_r.setInverted(true);
		v_r.setInverted(true);
		v_l.follow(t_l);
		v_r.follow(t_r);
		
		

		/* choose which cabling method for Pigeon */
		//_pidgey = new PigeonImu(0); /* Pigeon is on CANBus (powered from ~12V, and has a device ID of zero */
		//_pidgey = new PigeonIMU(_spareTalon); /* Pigeon is ribbon cabled to the specified CANTalon. */

		/* Define joysticklogi being used at USB port #0 on the Drivers Station */
		//_drivesticklogi = new Joysticklogi(0);	
	}
	
	private void turnDegree(double degree) {
		
		int sign;
		ahrs.reset();
		if(degree>0) {
			sign = 1;
			
		}else if(degree<0){
			sign = -1;
		}else {
			return;
		}
		while (sign*ahrs.getAngle()<sign*degree) {
			double spd = sign * ((degree - ahrs.getAngle())/degree*0.3 + 0.2);
			t_l.set(ControlMode.PercentOutput, spd);
			t_r.set(ControlMode.PercentOutput, -spd);
		}
		t_l.set(ControlMode.PercentOutput, 0);
		t_r.set(ControlMode.PercentOutput, 0);
	}
	
    public void teleopInit() {
		ahrs.reset(); /* reset heading, angle measurement wraps at plus/minus 23,040 degrees (64 rotations) */
		_goStraight = GoStraight.Off;  
		t_l.setNeutralMode(NeutralMode.Brake);
		t_r.setNeutralMode(NeutralMode.Brake);
		v_l.setNeutralMode(NeutralMode.Brake);
		v_r.setNeutralMode(NeutralMode.Brake);
		
    }
    private int loops = 0;
	private StringBuilder console = new StringBuilder();
	private boolean straightButtonPast = false;
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	if(sticklogi.getRawButton(1) && !straightButtonPast) {
			ahrs.reset();
			Timer.delay(0.05);
		}
    	if(sticklogi.getRawButtonPressed(2)) {
    		turnDegree(90);
    	}
    	double s_i = -stickbeitong.getRawAxis(5);
    	s_i = (s_i < 0.08 ? (s_i < -0.08 ? s_i : 0) : s_i);
    	
    	i_r.set(s_i);
    	i_l.set(s_i);
    	
    	double s_lift = stickbeitong.getTriggerAxis(Hand.kRight);
    	if (stickbeitong.getRawButton(3)) {
    		s_lift = s_lift *-1;
    	}
    	
    	lift_l.set(s_lift);
    	lift_r.set(s_lift);
    	/* some temps for Pigeon API */
//		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
//		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
//		double [] xyz_dps = new double [3];
		/* grab some input data from Pigeon and gamepad*/
	
		double currentAngle = ahrs.getAngle();
		boolean angleIsGood = ahrs.isConnected();
		double currentAngularRate = ahrs.getRate();
		/* get input from gamepad */
		boolean userWantsGoStraight = sticklogi.getRawButton(1); /* top left shoulder button */
//		double throttle = sticklogi.getThrottle();
//		throttle++; throttle /= 2;
		
		double s_y = -sticklogi.getRawAxis(1)/1.5;
		s_y = (s_y < 0.08 ? (s_y < -0.08 ? s_y : 0) : s_y);
		
		double s_z = sticklogi.getRawAxis(4)/2;
		boolean isInSlowMode = sticklogi.getRawButton(6);
    	
    	if (isInSlowMode) {
    		s_y = s_y*0.5;
    		s_z = s_z*0.5;
    	}
		/* deadbands so centering joysticklogis always results in zero output */
//		forwardThrottle = Db(forwardThrottle);
//		turnThrottle = Db(turnThrottle);
		/* simple state machine to update our goStraight selection */
		switch (_goStraight) {

			/* go straight is off, better check gamepad to see if we should enable the feature */
			case Off:
				if (userWantsGoStraight == false) {
					/* nothing to do */
				} else if (angleIsGood == false) {
					/* user wants to servo but Pigeon isn't connected? */
					_goStraight = GoStraight.SameThrottle; /* just apply same throttle to both sides */
				} else {
					/* user wants to servo, save the current heading so we know where to servo to. */
					_goStraight = GoStraight.UseAHRS;
				}
				break;

			/* we are servo-ing heading with ahrs */
			case UseAHRS:
				if (userWantsGoStraight == false) {
					_goStraight = GoStraight.Off; /* user let go, turn off the feature */
				} else if (angleIsGood == false) {
					_goStraight = GoStraight.SameThrottle; /* we were servoing with pidgy, but we lost connection?  Check wiring and deviceID setup */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;

			/* we are simply applying the same throttle to both sides, apparently Pigeon is not connected */
			case SameThrottle:
				if (userWantsGoStraight == false) {
					_goStraight = GoStraight.Off; /* user let go, turn off the feature */
				} else {
					/* user still wants to drive straight, keep doing it */
					
					
//					double l_trg = (s_y+s_z)* 4096 * 500.0 / 600;
//					double r_trg = (s_y-s_z)* 4096 * 500.0 / 600;
					double l_trg = s_y+s_z;
					double r_trg = s_y-s_z;
//					l_trg = Cap(l_trg, 1.0);
//					r_trg = Cap(r_trg, 1.0);
//					l_trg *= throttle;
//					r_trg *= throttle;
					l_trg=s_y;
					r_trg=l_trg;
//					t_l.set(ControlMode.PercentOutput, l_trg);
//					t_r.set(ControlMode.PercentOutput, r_trg);
				}
				break;
		}

		/* if we can servo with IMU, do the math here */
		if (_goStraight == GoStraight.UseAHRS) {
			
			/* very simple Proportional and Derivative (PD) loop with a cap,
			 * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
			s_z = (_targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
			/* the max correction is the forward throttle times a scalar,
			 * This can be done a number of ways but basically only apply small turning correction when we are moving slow
			 * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
//			double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
//			turnThrottle = Cap(turnThrottle, maxThrot);
		} else if (_goStraight == GoStraight.SameThrottle) {
			/* clear the turn throttle, just apply same throttle to both sides */
			s_z = 0;
		} else {
			/* do nothing */
		}

		/* positive turnThrottle means turn to the left, this can be replaced with ArcadeDrive object, or teams drivetrain object */
//		double left = forwardThrottle - turnThrottle;
//		double right = forwardThrottle + turnThrottle;
		

		/* my right side motors need to drive negative to move robot forward */
//		ahrs.reset();
		double l_trg = s_y+s_z;
		double r_trg = s_y-s_z;
//		l_trg = Cap(l_trg, 1.0);
//		r_trg = Cap(r_trg, 1.0);
//		l_trg *= throttle;
//		r_trg *= throttle;
		t_l.set(ControlMode.PercentOutput,l_trg);
		
		t_r.set(ControlMode.PercentOutput,r_trg);

		/* some printing for easy debugging */
/*		if (++_printLoops > 50){
			_printLoops = 0;
			
			System.out.println("------------------------------------------");
			System.out.println("error: " + (_targetAngle - currentAngle) );
			System.out.println("angle: "+ currentAngle);
			System.out.println("rate: "+ currentAngularRate);
			System.out.println("noMotionBiasCount: "+ genStatus.noMotionBiasCount);
			System.out.println("tempCompensationCount: "+ genStatus.tempCompensationCount);
			System.out.println( angleIsGood ? "Angle is good" : "Angle is NOT GOOD");
			System.out.println("------------------------------------------");
		}*/

		/* press btn 6, top right shoulder, to apply gains from webdash.  This can
		 * be replaced with your favorite means of changing gains. */
//		if (sticklogi.getRawButton(6)) {
//			UpdatGains();
//		}     
		console.append("angle:");
		console.append(ahrs.getAngle());
		console.append("\tanglerate:");
		console.append(ahrs.getRate());
		console.append("\tvelocity:");
		console.append(ahrs.getVelocityY());
		
		
		if (++loops >= 8) {
			loops = 0;
			System.out.println(console.toString());
		}
		console.setLength(0);
		
		straightButtonPast = sticklogi.getRawButton(1);
    }
    /** @return 10% deadband */
/*	double Db(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
	}
	/** @param value to cap.
	 * @param peak positive double representing the maximum (peak) value.
	 * @return a capped value.
	 */
/*	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
	}
	
	void UpdatGains() {
		//No way to get gains
	}
	/**
	 * Given the robot forward throttle and ratio, return the max
	 * corrective turning throttle to adjust for heading.  This is
	 * a simple method of avoiding using different gains for
	 * low speed, high speed, and no-speed (zero turns).
	 */
/*	double MaxCorrection(double forwardThrot, double scalor) {
		/* make it positive */
//		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
//		/* max correction is the current forward throttle scaled down */
//		forwardThrot *= scalor;
		/* ensure caller is allowed at least 10% throttle,
		 * regardless of forward throttle */
//		if(forwardThrot < 0.10)
//			return 0.10;
//		return forwardThrot;
//	}
    
}