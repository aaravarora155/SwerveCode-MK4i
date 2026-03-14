package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class ModuleIOKraken implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder absoluteEncoder;
    
    private final VoltageOut driveVoltage = new VoltageOut(0);
    private final VoltageOut steerVoltage = new VoltageOut(0);

    public ModuleIOKraken(int driveId, int steerId, int encoderId) {
        driveMotor = new TalonFX(driveId);
        steerMotor = new TalonFX(steerId);
        absoluteEncoder = new CANcoder(encoderId);
        
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;
        driveMotor.getConfigurator().apply(driveConfig);
        
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Feedback.SensorToMechanismRatio = SwerveConstants.STEER_GEAR_RATIO;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerMotor.getConfigurator().apply(steerConfig);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
        inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();

        inputs.steerAbsolutePositionRad = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        inputs.steerRelativePositionRad = Units.rotationsToRadians(steerMotor.getPosition().getValueAsDouble());
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerMotor.getVelocity().getValueAsDouble());
        inputs.steerAppliedVolts = steerMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerCurrentAmps = steerMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setControl(driveVoltage.withOutput(volts));
    }
    
    @Override
    public void setSteerVoltage(double volts) {
        steerMotor.setControl(steerVoltage.withOutput(volts));
    }
}