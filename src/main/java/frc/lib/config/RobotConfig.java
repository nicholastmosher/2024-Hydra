package frc.lib.config;

import frc.lib.CtreConfigs;

public class RobotConfig {
    public final CtreConfigs ctreConfigs;
    public final DashboardConfig dashboardConfig;

    public RobotConfig(CtreConfigs ctreConfigs, DashboardConfig dashboardConfig) {
        this.ctreConfigs = ctreConfigs;
        this.dashboardConfig = dashboardConfig;
    }
}
