package frc.robot.hybrid;

import java.util.ArrayList;

public class BlendedSwerve {
    private ArrayList<SwerveVectorProvider> pValues = new ArrayList<>();
    private ArrayList<SwerveVector> tValues = new ArrayList<>();

    public void addComponent(SwerveVectorProvider pValue, SwerveVector tValue) {
        pValues.add(pValue);
        tValues.add(tValue);
    }

    /**
     * Multiply the components of all inputs by their corresponding weights, then sum them
     * @return A SwerveVector representing the blend of all inputs
     */
    public SwerveVector solve() {
        double x = 0;
        double y = 0;
        double rot = 0;

        for (int i = 0; i < pValues.size(); i++) {
            double pX = pValues.get(i).get().x();
            double tX = tValues.get(i).x();
            x += pX * tX;

            double pY = pValues.get(i).get().y();
            double tY = tValues.get(i).y();
            y += pY * tY;

            double pRot = pValues.get(i).get().rot();
            double tRot = tValues.get(i).rot();
            rot += pRot * tRot;
        }

        return new SwerveVector(x, y, rot);
    }
}

