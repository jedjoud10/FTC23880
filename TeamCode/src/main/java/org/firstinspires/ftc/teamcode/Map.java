package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

// Contains distances and measurements from the centerstage map
// All measurements are in the metric system (meters)
@Config
public class Map {
    public static double PIXEL_OUTER_SIZE = 0.0762;
    public static double PIXEL_INNER_SIZE = 0.03175;
    public static double TILE_SIZE = 0.5842;
    public static double MAX_MAP_SIZE = 6 * TILE_SIZE;
    public static double DIST_BETWEEN_WHITE_PIXELS = 0.3048;
    public static double TOP_OF_RIGGING = 0.5969;
    public static double BOTTOM_OF_RIGGING = 0.3556;
    public static double HORIZ_RIGGING_DIST = 0.5461;
    public static double HORIZ_RIGGING_LARGE_DIST = 1.17475;
}
