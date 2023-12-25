package org.firstinspires.ftc.teamcode;

import junit.framework.TestCase;

import org.junit.Assert;
import org.junit.Test;

public class MapPositionTest extends TestCase {
    @Test
    public void test() {
        MapPosition zero = new MapPosition(0.0, 0.0);
        Assert.assertEquals(0.0, zero.x, 0.01);
        Assert.assertEquals(0.0, zero.y, 0.01);
        Assert.assertEquals(MapCoord.A1, zero.convertToId());
        Assert.assertEquals(zero.toString(), new MapPosition(MapCoord.A1).toString());
    }
}