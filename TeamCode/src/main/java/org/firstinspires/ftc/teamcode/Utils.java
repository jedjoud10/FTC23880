package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

public class Utils {
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // https://stackoverflow.com/questions/3451553/value-remapping
    public static double remap(double x, double low1, double high1, double low2, double high2) {
        return low2 + (x - low1) * (high2 - low2) / (high1 - low1);
    }
}
