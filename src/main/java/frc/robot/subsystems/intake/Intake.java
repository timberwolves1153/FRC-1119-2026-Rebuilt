package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class Intake extends SubsystemBase{
    
    public SparkMax rollerMotor;
    public SparkMax pivotMotor;
    

    public Intake() {
        rollerMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        pivotMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        rollerMotor.configure(
            Configs.Intake.rollerConfig, 
            com.revrobotics.ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        pivotMotor.configure(
            Configs.Intake.pivotConfig, 
            com.revrobotics.ResetMode.kResetSafeParameters,  
            PersistMode.kPersistParameters
        );
    }

    public void intakeIn() {
        rollerMotor.setVoltage(12);
    }

    public void inTakeOut() {
        rollerMotor.setVoltage(-12);
    }

    public void intakeStop() {
        rollerMotor.setVoltage(0);
    }

    public void intakeIdle() {
        rollerMotor.setVoltage(1);
    }



}
