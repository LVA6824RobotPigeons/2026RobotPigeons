package frc.robot.commands.auto;

// Local planner adapter used before BCNP offboard planning is enabled.
public final class LocalAutoPlannerClient implements AutoPlannerClient {
    @Override
    public AutoLinkState linkState() {
        return AutoLinkState.LOCAL_ONLY;
    }

    @Override
    public boolean isHealthy() {
        return true;
    }
}
