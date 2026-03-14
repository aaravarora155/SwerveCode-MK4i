package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(int canId) {
        pigeon = new Pigeon2(canId);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = 1.0;
        inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }

    @Override
    public void resetYaw() {
        pigeon.setYaw(0.0);
    }
}
