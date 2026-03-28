package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// Snapshot providers consumed by autonomous phases and safety checks.
public final class AutoContext {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Optional<Alliance>> allianceSupplier;
    private final BooleanSupplier hangerHomedSupplier;
    private final BooleanSupplier plannerHealthySupplier;

    public AutoContext(
            Supplier<Pose2d> poseSupplier,
            Supplier<Optional<Alliance>> allianceSupplier,
            BooleanSupplier hangerHomedSupplier,
            BooleanSupplier plannerHealthySupplier) {
        this.poseSupplier = poseSupplier;
        this.allianceSupplier = allianceSupplier;
        this.hangerHomedSupplier = hangerHomedSupplier;
        this.plannerHealthySupplier = plannerHealthySupplier;
    }

    public Pose2d robotPose() {
        return poseSupplier.get();
    }

    public Optional<Alliance> alliance() {
        return allianceSupplier.get();
    }

    public boolean isHangerHomed() {
        return hangerHomedSupplier.getAsBoolean();
    }

    public boolean isPlannerHealthy() {
        return plannerHealthySupplier.getAsBoolean();
    }
}
