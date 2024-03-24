package frc.robot.hybrid;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class SwerveVector extends Vector<N3> {
    public SwerveVector() {
        this(0, 0, 0);
    }

    public SwerveVector(double x, double y, double rotation) {
        super(Nat.N3());
        this.set(x, y, rotation);
    }

    public void set(double x, double y, double rotation) {
        this.set(0, 0, x);
        this.set(1, 0, y);
        this.set(2, 0, rotation);
    }

    public double x() {
        return this.get(0, 0);
    }

    public double y() {
        return this.get(1, 0);
    }

    public double rot() {
        return this.get(2, 0);
    }


    public SwerveVector times(double scalar) {
        return new SwerveVector(
                this.x() * scalar,
                this.y() * scalar,
                this.rot() * scalar

        );
    }

    public SwerveVector plus(SwerveVector other) {
        return new SwerveVector(
                this.x() + other.x(),
                this.y() + other.y(),
                this.rot() + other.rot()
        );
    }
}
