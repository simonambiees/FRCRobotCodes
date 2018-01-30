/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team6394.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import java.util.FormatFlagsConversionMismatchException;

import org.omg.PortableInterceptor.TRANSPORT_RETRY;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


public class Robot extends IterativeRobot {
	private boolean[] colorPos = new boolean[3]; 
	
	private Joystick stick = new Joystick(0);
	private TalonSRX t_l = new TalonSRX(0);
	private TalonSRX t_r = new TalonSRX(2);
	private VictorSPX v_l = new VictorSPX(1);
	private VictorSPX v_r = new VictorSPX(3);

	
	@Override
	public void robotInit() {
		v_l.follow(t_l);
		v_r.follow(t_r);
		
		t_l.configNominalOutputForward(0, 10);
		t_l.configNominalOutputReverse(0, 10);
		t_l.configPeakOutputForward(1, 10);
		t_l.configPeakOutputReverse(-1, 10);
		t_l.config_kF(0, 0.3892, 10);
		t_l.config_kP(0, 0.1496, 10);
		t_l.config_kI(0, 0.0025, 10);
		t_l.config_kD(0, 3.496, 10);
		t_l.setSensorPhase(true);
		
		t_r.configNominalOutputForward(0, 10);
		t_r.configNominalOutputReverse(0, 10);
		t_r.configPeakOutputForward(1, 10);
		t_r.configPeakOutputReverse(-1, 10);
		t_r.setInverted(true);
		v_r.setInverted(true);
		t_r.config_kF(0, 0.3892, 10);
		t_r.config_kP(0, 0.1180, 10);
		t_r.config_kI(0, 0.0022, 10);
		t_r.config_kD(0, 10.180, 10);
		t_r.setSensorPhase(true);
	}
	

	private boolean finished;
	@Override
	public void autonomousInit() {
		finished = false;
	}
	
	@Override
	public void autonomousPeriodic() {
		if (finished) return;
		t_l.set(ControlMode.Velocity, 0.5 * 4096 * 500.0 / 600);//run straight for 5m
		t_r.set(ControlMode.Velocity, 0.5 * 4096 * 500.0 / 600);
		Timer.delay(0.5*5.5);
		t_l.set(ControlMode.Velocity, 0.0);
		t_r.set(ControlMode.Velocity, 0.0);
		Timer.delay(0.3);
		t_l.set(ControlMode.Velocity, -0.25 * 4096 * 500.0 / 600);//turn left 90 degrees
		t_r.set(ControlMode.Velocity, 0.25 * 4096 * 500.0 / 600);
		Timer.delay(0.8);
		t_l.set(ControlMode.Velocity, 0.0);
		t_r.set(ControlMode.Velocity, 0.0);
		Timer.delay(0.3);
		t_l.set(ControlMode.Velocity, 0.5 * 4096 * 500.0 / 600);//run straight for 1.25m
		t_r.set(ControlMode.Velocity, 0.5 * 4096 * 500.0 / 600);
		Timer.delay(0.5*1.1);
		t_l.set(ControlMode.Velocity, 0.0);
		t_r.set(ControlMode.Velocity, 0.0);
		Timer.delay(0.3);
		t_l.set(ControlMode.Velocity, -0.25 * 4096 * 500.0 / 600);//turn left 90 degrees
		t_r.set(ControlMode.Velocity, 0.25 * 4096 * 500.0 / 600);
		Timer.delay(0.8);
		t_l.set(ControlMode.Velocity, 0.0);
		t_r.set(ControlMode.Velocity, 0.0);
		/*
		 * code for grabing and lifting cubes is needed here.
		 * then the robot should run forwards at low speed and then drop the cube
		*/
		t_l.set(ControlMode.Velocity, 0.1 * 4096 * 500.0 / 600);//run straight at low speed
		t_r.set(ControlMode.Velocity, 0.1 * 4096 * 500.0 / 600);//to contact the fence
		Timer.delay(3);
		t_l.set(ControlMode.Velocity, 0.0);
		t_r.set(ControlMode.Velocity, 0.0);
		finished = true;
	}
	
	@Override
	public void teleopInit() {
	}
	
	private int loops = 0;
	private StringBuilder console = new StringBuilder();
	
	@Override
	public void teleopPeriodic() {
		double throttle = stick.getThrottle();
		throttle++; throttle /= 2;
		double s_y = -stick.getY()/2;
		s_y = (s_y < 0.08 ? (s_y < -0.08 ? s_y : 0) : s_y);
		double s_z = stick.getZ()/3;
		double l_trg = (s_y+s_z)* 4096 * 500.0 / 600;
		double r_trg = (s_y-s_z)* 4096 * 500.0 / 600;
//		double l_trg = s_y+s_z;
//		double r_trg = s_y-s_z;
		l_trg *= throttle;
		r_trg *= throttle;
		if(stick.getRawButton(1)){
			r_trg=l_trg;
		}
		t_l.set(ControlMode.Velocity, l_trg);
		t_r.set(ControlMode.Velocity, r_trg);
		
//		for the test of Velocity closed-loop
//		console.append("out:");
//		console.append(t_l.getMotorOutputPercent());
//		console.append("\tspd:");
//		console.append(t_l.getSelectedSensorVelocity(0));
//		console.append("\terr:");
//		console.append(t_l.getClosedLoopError(0));
//		console.append("\ttrg:");
//		console.append(l_trg);
//		
//		console.append("\t\tout:");
//		console.append(t_r.getMotorOutputPercent());
//		console.append("\tspd:");
//		console.append(t_r.getSelectedSensorVelocity(0));
//		console.append("\terr:");
//		console.append(t_r.getClosedLoopError(0));
//		console.append("\ttrg:");
//		console.append(r_trg);
//		
//		if (++loops >= 8) {
//			loops = 0;
//			System.out.println(console.toString());
//		}
//		console.setLength(0);
	}
}