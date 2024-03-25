package frc.robot.hybrid;

import java.util.ArrayList;

public class BlendedControl {
    private final ArrayList<ControlVectorProvider> pValues = new ArrayList<>();
    private final ArrayList<ControlVectorProvider> tValues = new ArrayList<>();

    /**
     * Add a new input component to the blended control by providing
     * @param pValue function to get latest pValue for this component
     * @param tValue function to get latest tValue for this component
     */
    public void addComponent(ControlVectorProvider pValue, ControlVectorProvider tValue) {
        pValues.add(pValue);
        tValues.add(tValue);
    }

    /**
     * Multiply the components of all inputs by their corresponding weights, then sum them
     * @return A SwerveVector representing the blend of all inputs
     */
    public ControlVector solve() {
        double fX = 0; // Field X
        double fY = 0; // Field Y
        double rX = 0; // Robot X
        double rY = 0; // Robot Y
        double rot = 0; // Robot rotation
        double arm = 0; // Arm power

        for (int i = 0; i < pValues.size(); i++) {
            // Calculate weighted fieldX
            double pFx = pValues.get(i).get().swerveFieldX();
            double tFx = tValues.get(i).get().swerveFieldX();
            fX += pFx * tFx;

            // Calculate weighted fieldY
            double pFy = pValues.get(i).get().swerveFieldY();
            double tFy = tValues.get(i).get().swerveFieldY();
            fY += pFy * tFy;

            // Calculate weighted robotX
            double pRx = pValues.get(i).get().swerveRobotX();
            double tRx = tValues.get(i).get().swerveRobotX();
            rX += pRx * tRx;

            // Calculate weighted robotY
            double pRy = pValues.get(i).get().swerveRobotY();
            double tRy = tValues.get(i).get().swerveRobotY();
            rY += pRy * tRy;

            // Calculate weighted rotation
            double pRot = pValues.get(i).get().swerveRotation();
            double tRot = tValues.get(i).get().swerveRotation();
            rot += pRot * tRot;

            // Calculate weighted arm power
            double pArm = pValues.get(i).get().armPower();
            double tArm = tValues.get(i).get().armPower();
            arm += pArm * tArm;
        }

        return new ControlVector(fX, fY, rX, rY, rot, arm);
    }
}

