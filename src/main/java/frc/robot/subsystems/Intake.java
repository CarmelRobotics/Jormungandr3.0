package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private TalonFX pivotKrakenOne;
    private TalonFX pivotKrakenTwo;
    private TalonFX rollerKraken;

    public IntakeState currentState = IntakeState.STOW;
    final MotionMagicVoltage m_request;


    public Intake(){
        pivotKrakenOne = new TalonFX(IntakeConstants.kIntakePivotOneCanID, IntakeConstants.kIntakeCanivoreName);
        pivotKrakenTwo = new TalonFX(IntakeConstants.kIntakePivotTwoCanID, IntakeConstants.kIntakeCanivoreName);
        rollerKraken = new TalonFX(IntakeConstants.kIntakeRollerCanID, IntakeConstants.kIntakeCanivoreName);
        m_request = new MotionMagicVoltage(0);
        configMotors();
    }

    public void configMotors(){
        var pivOneConfig = new TalonFXConfiguration();
        var pivTwoConfig = new TalonFXConfiguration();
        var rollerConfig = new TalonFXConfiguration();
        var slot0Configs = pivOneConfig.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 7; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        var motionMagicConfigs = pivOneConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 250; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        var invert = pivOneConfig.MotorOutput;
        invert.Inverted = InvertedValue.CounterClockwise_Positive;


        var slot0Configs1 = pivTwoConfig.Slot0;
        slot0Configs1.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs1.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs1.kP = 7; // A position error of 2.5 rotations results in 12 V output
        slot0Configs1.kI = 0; // no output for integrated error
        slot0Configs1.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        var motionMagicConfigs1 = pivTwoConfig.MotionMagic;
        motionMagicConfigs1.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
        motionMagicConfigs1.MotionMagicAcceleration = 250; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs1.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        var invert1 = pivTwoConfig.MotorOutput;
        invert1.Inverted = InvertedValue.Clockwise_Positive;
       

        var currentLimitconfigs = pivOneConfig.CurrentLimits;
        currentLimitconfigs.SupplyCurrentLimit = 65;
        currentLimitconfigs.SupplyCurrentLimitEnable = true;

        var currentLimitconfigs1 = pivTwoConfig.CurrentLimits;
        currentLimitconfigs.SupplyCurrentLimit = 65;
        currentLimitconfigs.SupplyCurrentLimitEnable = true;

        var rollerCurrentLimit = rollerConfig.CurrentLimits;
        rollerCurrentLimit.SupplyCurrentLimit = 15;
        rollerCurrentLimit.SupplyCurrentLimitEnable = true;

        this.pivotKrakenOne.getConfigurator().apply(pivOneConfig);
        this.pivotKrakenTwo.getConfigurator().apply(pivTwoConfig);
        this.rollerKraken.getConfigurator().apply(rollerConfig);


    }
    private void setRollerLimit(double limit){
        var rollerConfig = new TalonFXConfiguration();
        
        var rollerCurrentLimit = rollerConfig.CurrentLimits;
        rollerCurrentLimit.SupplyCurrentLimit = limit;
        rollerCurrentLimit.SupplyCurrentLimitEnable = true;
        this.rollerKraken.getConfigurator().apply(rollerConfig);

    }


    public enum IntakeState{
        INTAKING(PivotState.DOWN,RollerState.INTAKE),
        OUTTAKING(PivotState.DOWN,RollerState.OUTTAKE),
        SCORING(PivotState.SCORE,RollerState.OUTTAKE),
        CLIMBSTART(PivotState.CLIMB,RollerState.INTAKE),
        CLIMBFINAL(PivotState.STOW,RollerState.IDLE),
        STOW(PivotState.STOW,RollerState.IDLE),
        STATION_INTAKE(PivotState.STATION,RollerState.INTAKE);
        public PivotState pivotState;
        public RollerState rollerState;
        private IntakeState(PivotState pivotState, RollerState rollerState){
            this.pivotState = pivotState;
            this.rollerState = rollerState;
        }
    }

    public enum PivotState{
        STOW(15),
        DOWN(30.35),
        SCORE(15),
        CLIMB(2.5),
        STATION(.75);
        public double position;
        private PivotState(double position){
            this.position = position;
        }
    }

    public enum RollerState{
        INTAKE(-12),
        OUTTAKE(12),
        IDLE(-5);
        public double volts;
        private RollerState(double volts){
            this.volts = volts;
        }
    }
    public void zeroEncoders(){
        this.pivotKrakenOne.setPosition(0);
        this.pivotKrakenTwo.setPosition(0);
    }

    public Command setState(IntakeState state){
        return Commands.runOnce(()->{
            this.currentState = state;
        });
    }

    @Override
    public void periodic() {
        this.pivotKrakenOne.setControl(m_request.withPosition(this.currentState.pivotState.position));
        this.pivotKrakenTwo.setControl(m_request.withPosition(this.currentState.pivotState.position));
        this.rollerKraken.setVoltage(this.currentState.rollerState.volts);
        
        SmartDashboard.putNumber("Encoder val 1", this.pivotKrakenOne.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder val 2", this.pivotKrakenTwo.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Kraken connected", this.pivotKrakenTwo.isConnected());
        SmartDashboard.putString("current state", this.currentState.toString());
        boolean stall = (this.rollerKraken.getSupplyCurrent().getValueAsDouble() > 5) && this.rollerKraken.getVelocity().getValueAsDouble() == 0;
        SmartDashboard.putBoolean("stalling rollers", stall);
        if(stall){
            this.setRollerLimit(4);
        } else {
            this.setRollerLimit(15);
        }
    }

    public boolean hasCoral(){
        return (this.rollerKraken.getSupplyCurrent().getValueAsDouble() > 4 && this.rollerKraken.getVelocity().getValueAsDouble() == 0);
    }
    
    
}
