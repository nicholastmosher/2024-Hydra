package frc.robot.classes;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class RollingAverage {
    private final int bufferSize;
    private final LinkedList<Double> buffer;
    public RollingAverage(int bufferSize) {
        this.bufferSize = bufferSize;
        this.buffer = new LinkedList<>();
    }

    /**
     * Add a value from input, returning the latest rolling average
     * @param input
     * @return
     */
    public void addInput(double input) {
        // If the buffer is full, remove the last (oldest) item
        if (this.buffer.size() >= this.bufferSize) {
            this.buffer.removeLast();
        }

        this.buffer.addFirst(input);
    }

    public double getOutput() {
        double sum = 0.0;
        for (double value : this.buffer) {
            sum += value;
        }

        double average = sum / this.buffer.size();
        return average;
    }
}
