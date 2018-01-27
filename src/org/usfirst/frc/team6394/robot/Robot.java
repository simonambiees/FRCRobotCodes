<<<<<<< HEAD
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

		t_r.configNominalOutputForward(0, 10);
		t_r.configNominalOutputReverse(0, 10);
		t_r.configPeakOutputForward(1, 10);
		t_r.configPeakOutputReverse(-1, 10);
		t_r.setInverted(true);
		v_r.setInverted(true);
	}
	
	
	@Override
	public void autonomousInit() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		for (int i=0;i<3;i++){
			colorPos[i] = (gameData.charAt(i) == 'L');
			System.out.println(colorPos[i]);
		}
	}
	
	private boolean flag = false; 
	
	@Override
	public void autonomousPeriodic() {
		if (flag) return;
		
		t_l.set(ControlMode.PercentOutput,0.3);
		t_r.set(ControlMode.PercentOutput,0.3);
		Timer.delay(1);
		t_l.set(ControlMode.PercentOutput,0.0);
		t_r.set(ControlMode.PercentOutput,0.0);
		
		flag = true;
	}
	
	
	@Override
	public void teleopInit() {
	}
	
	@Override
	public void teleopPeriodic() {
		double s_y = -stick.getY()/3;
		double s_z = stick.getZ()/4;
		t_l.set(ControlMode.PercentOutput, s_y+s_z);
		t_r.set(ControlMode.PercentOutput, s_y-s_z);
		StringBuilder console = new StringBuilder();
		console.append("out:");
		console.append(t_l.getMotorOutputPercent());
		console.append("|\t");
		console.append(t_r.getMotorOutputPercent());
		System.out.println(console);
=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

