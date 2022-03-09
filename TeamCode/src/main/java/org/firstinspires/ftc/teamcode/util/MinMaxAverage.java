package org.firstinspires.ftc.teamcode.util;

public class MinMaxAverage {
    private double min = Double.MAX_VALUE, max = -Double.MAX_VALUE, total = 0, numValues = 0;

    public void update(double newValue) {
        total += newValue;
        numValues ++;
        if (newValue < min) {
            min = newValue;
        } else if (newValue > max) {
            max = newValue;
        }
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    public double getAverage() {
        return total / numValues;
    }

    public String toString() {
        return String.format("min %.2f, max %.2f, avg %.2f", getMin(), getMax(), getAverage());
    }
}
