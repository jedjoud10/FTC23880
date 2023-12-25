package org.firstinspires.ftc.teamcode;

@SuppressWarnings("unused")
public enum MapCoord {
    A1(0),
    B1(1),
    C1(2),
    D1(3),
    E1(4),
    F1(5),
    A2(6),
    B2(7),
    C2(8),
    D2(9),
    E2(10),
    F2(11),
    A3(12),
    B3(13),
    C3(14),
    D3(15),
    E3(16),
    F3(17),
    A4(18),
    B4(19),
    C4(20),
    D4(21),
    E4(22),
    F4(23),
    A5(24),
    B5(25),
    C5(26),
    D5(27),
    E5(28),
    F5(29),
    A6(30),
    B6(31),
    C6(32),
    D6(33),
    E6(34),
    F6(35);


    public final int value;
    MapCoord(int value) { this.value = value; }
    public static MapCoord withValue(int value) {
        for (MapCoord m : MapCoord.values()) {
            if (m.value == value) return m;
        }
        return null;
    }
}
