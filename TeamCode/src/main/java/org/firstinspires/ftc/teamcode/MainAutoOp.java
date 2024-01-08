/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
30s autonomous period:
purple pixel on spike mark tape: 10p
purple pixel on spike mark tape with team prop: 20p
backstage pixels: 3p
pixel on backdrop: 5p
yellow pixel on backdrop spike mark indicator: 10bp
yellow pixel on backdrop team prop indicator: 20bp
parked bot in backstage: 5p

Autonomous (cam + sensors) (multiple op modes based on pos.
Looking for avg 30 pixels in auto)

1. Place purple pixel on team prop (20 points) (remember pos of team prop spike mark)
2. Go to backdrop to saved pos and place yellow pixel there (20p)
3. Park (5p)

Gonna make it detect the location of white pixel / team prop and place purple pixel there. Basically make it go forward, check using cam if the pixel is there, if not, rotate (+- 90 deg), check again, and place.
Make it pick up white pixels from back, then move to the backdrop and place pixels (white + yellow).
If I have enough time make another round trip and place 2 more pixels.
Near the end go park backstage.

*/

@Autonomous

public class MainAutoOp extends LinearOpMode {
    private RobotMovement movement;
    private RobotGripper gripper;
    private DistanceSensors distances;

    private int teamPropIndex = 0; // [left = -1, center = 0, right = 1]
    private IMU imu;
    private MultipleTelemetry debug = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() {
        // Initial TFOD, AprilTag processors, webcam, and vision portal
        initHwMap();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        debug.addLine("Test");
        movement.moveLine(0.3);

        /*
        // Make square using the robot
        for (int i = 0; i < 4; i++) {
            movement.moveLine(0.3);
            sleep(1000);
            movement.rotate90InPlace(true);
        }
        */

        /*

        sleep(5000);
        movement.moveAround(1.0, 90);
        sleep(5000);
        movement.moveAround(-1.0, 90);
        sleep(5000);
        movement.rotate90InPlace(false);
        sleep(5000);
        movement.rotate180InPlace(false);
        sleep(5000);
        movement.moveHorizontal(1, 1, 30);
        sleep(5000);
        movement.moveHorizontal(1, 1, 45);
        sleep(5000);
        movement.moveHorizontal(1, 1, 60);
        sleep(5000);
        for (int i = 0; i < 10; i++) {
            movement.moveHorizontal(0.1, 0.1, 45);
        }


        // Place purple pixel on team prop (20p), align to backstage
        MapCoord coord = teamPropPlacePurple();

        // Move to backstage (if necessary)
        moveBackstage(coord);

        // Align with drop column, drop yellow pixel (20p)
        alignDropColumnAndDrop(true);

        // (Maybe) round trip back to pickup 2 white pixels (extra 6p-10p)
        if (AutoStrategy.PICKUP_WHITE_PIXEL) {
            // Go pick em up

            // Align with drop column, drop 2 white pixels (20p)
            alignDropColumnAndDrop(false);
        }

        // Park (5p) (assumes it is at E5)
        park();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetryTfod();
            telemetryAprilTag();
            debug.update();

            visionPortal.resumeStreaming();
            visionPortal.resumeLiveView();
        }

         */
    }

    private void initHwMap() {
        imu = hardwareMap.get(IMU.class, "imu");
        movement = new RobotMovement(hardwareMap, debug);
        distances = new DistanceSensors(hardwareMap, debug);
        gripper = new RobotGripper(movement, hardwareMap, debug);
    }

    // Place purple pixel and moves to main column to be ready to move to backstage
    // Returns the end map coordinate to feed it to the next step, which is actually *going* to the backstage
    private MapCoord teamPropPlacePurple() {
        // Check center
        int teamPropPosBitMask = 0b111; // left = 2, center = 1, right = 0
        movement.moveLine(Map.TILE_SIZE);
        teamPropPosBitMask &= (distances.checkFront() ? 2 : 8);

        // Check right
        movement.rotate90InPlace(true);
        teamPropPosBitMask &= (distances.checkFront() ? 1 : 8);

        // No need to check left as we KNOW it is on the left (or not at all)
        // Convert to index [left, center, right]
        teamPropIndex = Integer.numberOfLeadingZeros(teamPropPosBitMask) - 1;

        // ------------------------------------------------------------------
        // STRATEGY FOR F2-E2
        // If it is in the center, place, maneuver back, go main D. D2-N
        // If it is in the right, place, main column D, and get to backstage. D2-N
        //    If we are at E2 it might be nice to pickup white pixel while we are at it
        // If it is in the left, place, main column D, and get to backstage. D2-N
        // ------------------------------------------------------------------
        // STRATEGY FOR F4-E4
        // If it is in the center, place, face backstage, get to backstage. E4-N
        // If it is in the right, do funky maneuver. E5-N
        // If it is in the left, place, face backstage, and get to backstage. E4-N
        switch (AutoStrategy.STARTING_POS) {
            case F2:
                switch (teamPropIndex) {
                    case -1:
                    case 1:
                        // Face forward (perp to backstage)
                        movement.rotate90InPlace(false);

                        // Currently in column E, move to D
                        movement.moveLine(Map.TILE_SIZE);
                        movement.rotate90InPlace(true);
                        break;
                    case 0:
                        // Do funky maneuver
                        movement.rotate90InPlace(false);
                        movement.moveLine(-0.50 * Map.TILE_SIZE);
                        movement.rotate90InPlace(false);
                        movement.moveAround(0.80 * Map.TILE_SIZE, 270);
                        break;
                }
                return MapCoord.D2;
            case F4:
                switch (teamPropIndex) {
                    case -1:
                        movement.rotate180InPlace(false);
                        return MapCoord.E4;
                    case 0:
                        movement.rotate90InPlace(false);
                        return MapCoord.E4;
                    case 1:
                        // Do funky maneuver
                        movement.rotate90InPlace(false);
                        movement.moveLine(0.50 * Map.TILE_SIZE);
                        movement.moveAround(Map.TILE_SIZE, 180);
                        movement.moveLine(0.50 * Map.TILE_SIZE);
                        movement.rotate90InPlace(false);
                        return MapCoord.E5;
                }
                break;
        }

        return null;
    }

    // Move to backstage using current map coord
    private void moveBackstage(MapCoord current) {
        switch (current) {
            case D2:
                movement.moveLine(Map.TILE_SIZE * 2);
                movement.moveAround(Map.TILE_SIZE, -90);
                movement.rotate90InPlace(false);
                break;
            case E4:
                movement.moveLine(Map.TILE_SIZE);
                break;
            case E5:
                break;
        }
    }

    // Align the robot with the corresponding drop column (team prop spike mark pos) if neede

    private void alignDropColumnAndDrop(boolean yellowPixel) {
        movement.setTargetTickThrottle(0.2);
        movement.moveLine(0.2);

        double horizontalDelta = 0.0;

        if (yellowPixel) {
            // move to corresponding column
            // Margin of 2 x pixel size
        }

        movement.moveHorizontal(horizontalDelta * Map.PIXEL_OUTER_SIZE, 0.02, 45);
        gripper.setArmHeight(0.0);

        while (!distances.checkFront()) {
            movement.moveLine(0.03);
        }

        gripper.setGripperTargetsWait(false, false);
    }

    // Pickup 2 white pixels from the audience side. Assumes the bot is D2-S
    private void pickupWhitePixels() {
        movement.moveLine(Map.TILE_SIZE * 0.9);
        gripper.setGripperTargetsWait(false, false);
        movement.setTargetTickThrottle(0.2);
        movement.moveLine(Map.TILE_SIZE * 0.9);
        movement.setTargetTickThrottle(1.0);
    }

    // Park the bot. Either to the corner or the other wing. Assumes the bot is E5-N state
    private void park() {
        switch (AutoStrategy.PARK_POS) {
            case D6:
                movement.rotate90InPlace(false);
                movement.moveAround(Map.TILE_SIZE, 90);
                break;
            case F6:
                movement.rotate90InPlace(true);
                movement.moveAround(Map.TILE_SIZE, -90);
                break;
        }
    }
}