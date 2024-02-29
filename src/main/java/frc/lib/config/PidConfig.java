package frc.lib.config;

public class PidConfig {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    public PidConfig(double p, double i, double d) {
        this(p, i, d, 0.0);
    }

    public PidConfig(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }
}
