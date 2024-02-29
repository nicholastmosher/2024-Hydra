package frc.lib.config;

public class CurrentLimitConfig {
    public final int currentLimit;
    public final int currentThreshold;
    public final double currentThresholdTime;
    public final boolean enableCurrentLimit;

    public CurrentLimitConfig(int limit, int threshold, double thresholdTime, boolean enableLimit) {
        this.currentLimit = limit;
        this.currentThreshold = threshold;
        this.currentThresholdTime = thresholdTime;
        this.enableCurrentLimit = enableLimit;
    }
}
