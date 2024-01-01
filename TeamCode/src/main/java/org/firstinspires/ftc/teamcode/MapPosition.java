package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

// Map position starting from A1 (bottom left), red player
public class MapPosition {
    public double x;
    public double y;

    public MapPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public MapPosition(MapCoord coordinate) {
        int i = coordinate.value;
        this.x = (i%6) * Map.TILE_SIZE;
        this.y = (((double)i/6)) * Map.TILE_SIZE;
    }

    public MapCoord convertToId() {
        double xNew = MathUtils.clamp(x, 0, Map.MAX_MAP_SIZE)  / Map.TILE_SIZE;
        double yNew = MathUtils.clamp(y, 0, Map.MAX_MAP_SIZE) / Map.TILE_SIZE;
        int xFloored = (int)Math.floor(xNew);
        int yFloored = (int)Math.floor(yNew);
        return MapCoord.withValue(xFloored + yFloored*6);
    }

    @Override
    public String toString() {
        return convertToId().toString();
    }
}
