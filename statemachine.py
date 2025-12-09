from XRPLib.defaults import *
from machine import UART, Pin
import time
import json

# ============================================================================
# CONFIGURATION CLASS
# ============================================================================

class Robot:
    # Pressure plate detection
    PLATE_X_CENTER = 0.5
    PLATE_X_DEADZONE = 0.15

    # Basket pickup configuration
    SEARCH_TURN_EFFORT = 0.3
    LINEUP_TURN_EFFORT = 0.22
    DRIVE_EFFORT = 0.5
    BASKET_X_TARGET = 0.55
    BASKET_X_DEADZONE = 0.035
    BASKET_Y_TARGET = 0.85
    BASKET_Y_DEADZONE = 0.1
    SERVO_PICKUP = 95.0
    SERVO_CARRY = 135.0
    PICKUP_INCREMENTS = 10
    PICKUP_DISTANCE = 13.0
    MAX_EFFORT = 0.5
   
    # Navigation configuration
    TARGET_WALL_DISTANCE = 15.0
    STEP_DISTANCE = 12
    OBSTACLE_THRESHOLD = 30.0

    # Line following PID
    LINE_FOLLOW_BASE_EFFORT = -0.3
    LINE_FOLLOW_KP = 0.35
    LINE_FOLLOW_KI = 0.00
    LINE_FOLLOW_KD = 0.00

    # Line detection
    LINE_DIFF_THRESHOLD = 0.05

    
    
    # Communication
    UART_BAUDRATE = 115200
    
    class State:
        FOLLOW_LINE = 0
        GO_PLATE = 1
        RETURN_TO_LINE = 2
        ENTER_MAZE = 3
        EXPLORE_FOR_BUCKET = 4
        PICKUP_BUCKET = 5
        BACK_TO_WALL = 6
        ALIGN_TO_ARROW = 9
        FOLLOW_LINE_SECOND = 10

    class BoundingBoxId:
        ANGLED_BUCKET = 1
        APPROACHABLE_BUCKET = 2
        ARROW = 3
        PRESSURE_PLATE = 4


# ============================================================================
# SENSOR AND COMMUNICATION MANAGEMENT
# ============================================================================

class SensorManager:
    def __init__(self):
        pass

    def get_reflectance_readings(self):
        """Get left and right reflectance sensor readings."""
        left_val = reflectance.get_right()
        right_val = reflectance.get_left()
        return left_val, right_val

    def get_reflectance_difference(self):
        """Get difference between left and right reflectance readings."""
        left_val, right_val = self.get_reflectance_readings()
        return left_val - right_val

    def get_distance(self):
        """Get distance from rangefinder."""
        return rangefinder.distance()

    def get_avg_distance(self, samples=3, sleep=0):
        """Get averaged distance reading to reduce noise."""
        time.sleep(sleep)
        total = 0
        for _ in range(samples):
            total += self.get_distance()
            time.sleep(0.02)
        return total / samples

    def get_encoder_positions(self):
        """Get left and right encoder positions."""
        left = drivetrain.get_left_encoder_position()
        right = drivetrain.get_right_encoder_position()
        return left, right

    def get_average_distance_traveled(self):
        """Get average distance traveled from both encoders."""
        left, right = self.get_encoder_positions()
        return (left + right) / 2

    def reset_odometry(self):
        """Reset all odometry sensors."""
        left_motor.reset_encoder_position()
        right_motor.reset_encoder_position()
        imu.reset_yaw()

    def get_heading(self):
        """Get current heading from IMU."""
        return imu.get_heading()


class CommunicationManager:
    def __init__(self):
        self.uart = UART(
            0,
            baudrate=Robot.UART_BAUDRATE,
            tx=Pin(0),
            rx=Pin(1),
            timeout=200,
        )

    def get_bbox_info(self, min_score=0.0):
        """
        Reads one JSON line from UART and returns:
            id, score, xmin, xmax, ymin, ymax, width, height, x_center, y_center

        Returns:
            dict with keys OR None if no detection
        """
        line = self.uart.readline()
        print(line)  # debug line
        if line is None:
            return None

        try:
            data = json.loads(line.decode("utf-8"))
        except:
            return None

        best_bbox = None

        for bbox in data.get("bboxes", []):
            cid = bbox.get("id", None)
            score = bbox.get("score", 0.0)

            if cid is None:
                continue

            if score < min_score:
                continue

            # Keep only best score
            if best_bbox is None or score > best_bbox["score"]:
                best_bbox = bbox

        # If nothing found
        if best_bbox is None:
            return None

        # Extract values
        xmin = best_bbox["xmin"]
        xmax = best_bbox["xmax"]
        ymin = best_bbox["ymin"]
        ymax = best_bbox["ymax"]
        score = best_bbox["score"]
        obj_id = best_bbox["id"]

        # Calculations
        width = xmax - xmin
        height = ymax - ymin
        x_center = xmin + width / 2
        y_center = ymin + height / 2
        print(width)
        # Return ONE structured object
        return {
            "id": obj_id,
            "score": score,
            "xmin": xmin,
            "xmax": xmax,
            "ymin": ymin,
            "ymax": ymax,
            "width": width,
            "height": height,
            "x_center": x_center,
            "y_center": y_center,
        }


# ============================================================================
# ROBOT STATE MACHINE CLASS
# ============================================================================

class StateMachine:
    def __init__(self):
        self.departure_heading = 0
        self.distance_from_line = 0
        self.line_heading = 0
        self.coral_info = None
        self.state = Robot.State.FOLLOW_LINE
        #self.state = Robot.State.EXPLORE_FOR_BUCKET
        self.scan_counter = 0
        self.return_done = False
        self.line_lost_counter = 0

        self.comm = CommunicationManager()
        self.sensor = SensorManager()

    # ------------------------------------------------------------------------
    # SENSOR AND DATA MANAGEMENT
    # ------------------------------------------------------------------------

    def update_coral_info(self):
        """Read one frame of Coral data and store it."""
        self.coral_info = self.comm.get_bbox_info()
        if self.coral_info is None:
            print("No object detected")
        else:
            print("Coral:", self.coral_info)

    def reset_odometry(self):
        """Call when leaving the line"""
        self.sensor.reset_odometry()
        self.departure_heading = 0
        self.distance_from_line = 0
        self.line_heading = self.sensor.get_heading()

    # ------------------------------------------------------------------------
    # LINE FOLLOWING AND DETECTION
    # ------------------------------------------------------------------------

    def line_follow(self, previous_time=None, integral=0, previous_error=0):
        time.sleep(1)

        base_effort = Robot.LINE_FOLLOW_BASE_EFFORT
        KP = Robot.LINE_FOLLOW_KP
        KI = Robot.LINE_FOLLOW_KI
        KD = Robot.LINE_FOLLOW_KD

        error = self.sensor.get_reflectance_difference()

        if previous_time is None:
            previous_time = time.ticks_ms()

        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, previous_time) / 1000.0
        if dt <= 0:
            dt = 0.001

        integral += error * dt
        integral = max(-1.0, min(1.0, integral))
        derivative = (error - previous_error) / dt
        correction = KP * error + KI * integral + KD * derivative

        drivetrain.set_effort(base_effort - correction, base_effort + correction)
        time.sleep(0.5)

        previous_error = error
        previous_time = current_time

    def find_line_probe(self, initial_angle=30, max_angle=180, angle_increment=30, probe_speed=0.35, timeout=10):
        """
        Probe left and right with increasing angles until line is found.
        """
        DIFF_THRESHOLD = 0.15
        global_start = time.ticks_ms()
        probe_angle = initial_angle

        while time.ticks_diff(time.ticks_ms(), global_start) < timeout * 1000:
            print("Probing with angle:", probe_angle)
            probe_time = (probe_angle / 90) * 1.0

            # === PROBE LEFT ===
            print("Probing left...")
            found, diff = self._probe_direction(probe_speed, -probe_speed, probe_time, DIFF_THRESHOLD)
            if found and diff > DIFF_THRESHOLD:
                print("Found line (left dark while turning left)")
                drivetrain.set_effort(0,0)
                drivetrain.turn(20,0.4)
                drivetrain.set_effort(0,0)
                return True
            self._stop_and_pause()

            # === RETURN TO CENTER ===
            self._return_to_center(-probe_speed, probe_speed, probe_time)

            # === PROBE RIGHT ===
            print("Probing right...")
            found, diff = self._probe_direction(-probe_speed, probe_speed, probe_time, DIFF_THRESHOLD)
            if found:
                if diff < -DIFF_THRESHOLD:
                    print("Found line (right dark while turning right)")
                    drivetrain.set_effort(0,0)
                    drivetrain.turn(-20,0.4)
                    drivetrain.set_effort(0,0)
                    return True
                elif diff > DIFF_THRESHOLD:
                    print("Found line (left dark while turning right - was on it)")
                    drivetrain.set_effort(0,0)
                    drivetrain.turn(20,0.4)
                    drivetrain.set_effort(0,0)
                    return True
            self._stop_and_pause()

            # === RETURN TO CENTER ===
            self._return_to_center(probe_speed, -probe_speed, probe_time)

            # === INCREASE ANGLE FOR NEXT ITERATION ===
            probe_angle = min(probe_angle + angle_increment, max_angle)
            print("Line not found, expanding to", probe_angle)

        print("Timeout - line not found!")
        return False

    def step_return_to_line(self):
        """
        1) Retrace the distance from plate back to the line.
        2) Turn 90 to face along the line again.
        3) Follow the line until it ends, then go to STATE_ENTER_ROOM.
        """

        # --- Phase 1: go back the way we came ---
        if not self.return_done:
            dist_forward = self.sensor.get_average_distance_traveled()
            print("Distance from line to plate:", dist_forward, "cm")

            # Reverse that distance
            reverse_dist = -dist_forward
            print("Returning by driving:", reverse_dist, "cm")

            drivetrain.straight(reverse_dist-4, max_effort=0.25)
            time.sleep(1)
            # Turn back 90 degrees (opposite direction we used to enter)
            # If you turned -90 to go to the plate, use +90 here.
            drivetrain.turn(-90, max_effort=0.4)
            time.sleep(1)

            # Mark that we've completed the return motion
            self.return_done = True

            # Reset line-lost counter for the next phase
            self.line_lost_counter = 0
            return
        call_line_follow = self.find_line_probe()
        if call_line_follow == True:
            return True
        # --- Phase 2: follow the line, no searching ---
        # self.line_follow()  # reuse your PID line-following

        # Detect when the line ends: no contrast between sensors

    # ------------------------------------------------------------------------
    # OBJECT DETECTION AND IDENTIFICATION
    # ------------------------------------------------------------------------

    def pick_up_basket(self):
        """
        Align to and pick up the bucket.
        Assumes:
          - We are already in front of the bucket area
          - We *recently* saw id == 2 before calling this
        Does NOT spin/search if nothing is seen.
        Uses continuous bounding box updates to align, then grabs.
        """
        print("=== pick_up_basket called ===")
        robot.sensor.reset_odometry()

        wait_counter = 0
        missing_frames = 0
        MAX_MISSING_FRAMES = 15  # how long we tolerate losing the object

        while True:
            bbox = self.comm.get_bbox_info(min_score=0)

            # --- If we completely lose detection for a while, abort ---
            if bbox is None:
                missing_frames += 1
                print("No detection, missing_frames =", missing_frames)
                drivetrain.set_effort(0, 0)
                if missing_frames > MAX_MISSING_FRAMES:
                    drivetrain.straight(10, max_effort=Robot.MAX_EFFORT)
                    missing_frames = 0
                time.sleep(0.05)
                continue
            else:
                missing_frames = 0  # we see something again

            # From here, we don't care if id is 1 or 2, it's the same object
            x_center = bbox["x_center"]
            y_center = bbox["y_center"]
            print("Bucket center:", x_center, y_center, "id:", bbox["id"])

            # --- Step 1: Align X (left/right) ---
            if x_center < (Robot.BASKET_X_TARGET - Robot.BASKET_X_DEADZONE):
                # bucket is LEFT of desired center -> small left turn
                drivetrain.set_effort(-Robot.LINEUP_TURN_EFFORT, Robot.LINEUP_TURN_EFFORT)
                wait_counter = 0
                time.sleep(0.05)
                continue

            elif x_center > (Robot.BASKET_X_TARGET + Robot.BASKET_X_DEADZONE):
                # bucket is RIGHT of desired center -> small right turn
                drivetrain.set_effort(Robot.LINEUP_TURN_EFFORT, -Robot.LINEUP_TURN_EFFORT)
                wait_counter = 0
                time.sleep(0.05)
                continue

            else:
                # X is good
                drivetrain.set_effort(0, 0)
                print("X aligned")
                time.sleep(1)

            # --- Step 2: Align Y (distance) ---
            if y_center < (Robot.BASKET_Y_TARGET - Robot.BASKET_Y_DEADZONE):
                # bucket too far -> move forward a bit
                print("Bucket far, moving forward slightly")
                drivetrain.straight(-5, max_effort=Robot.DRIVE_EFFORT)
                wait_counter = 0
                continue

            elif y_center > (Robot.BASKET_Y_TARGET + Robot.BASKET_Y_DEADZONE):
                # bucket too close -> back up a bit
                print("Bucket too close, backing up slightly")
                drivetrain.straight(5, max_effort=Robot.DRIVE_EFFORT)
                wait_counter = 0
                continue

            else:
                # X and Y are both
                wait_counter += 1
                print("Aligned (X,Y) stable frames:", wait_counter)

                # Require a few stable frames to avoid jitter
                if wait_counter < 5:
                    continue

                # Stable alignment -> proceed to pickup
                break

        # --- Step 3: pickup sequence ---

        print("Lowering arm")
        servo_one.set_angle(Robot.SERVO_PICKUP)
        time.sleep(0.5)

        print("Driving into basket...")
        drivetrain.straight(-Robot.PICKUP_DISTANCE, max_effort=Robot.MAX_EFFORT)

        print("Lifting basket...")
        period = 1.0 / Robot.PICKUP_INCREMENTS
        for i in range(Robot.PICKUP_INCREMENTS):
            angle = ((Robot.SERVO_CARRY - Robot.SERVO_PICKUP) * (period * i)) + Robot.SERVO_PICKUP
            servo_one.set_angle(angle)
            time.sleep(period)
        servo_one.set_angle(Robot.SERVO_CARRY)
        time.sleep(1.0)

        print("=== Basket picked up successfully ===")

        # Back up a little
        drivetrain.straight(15, max_effort=Robot.DRIVE_EFFORT)

        # enusre we have it, otherwise try again
        time.sleep(5)
        seen_buckets = 0
        for i in range(25):
            time.sleep(0.2)
            info = self.comm.get_bbox_info(min_score=0)
            print("Seen bucket loop")
            if info is not None and info["id"] == Robot.BoundingBoxId.APPROACHABLE_BUCKET:
                seen_buckets += 1

        if seen_buckets >= 10:
            print("bucket still there, call pickup again.")
            self.pick_up_basket()

        drivetrain.turn(-90, max_effort=0.4)
                
        return True

    # ------------------------------------------------------------------------
    # MOVEMENT HELPERS
    # ------------------------------------------------------------------------
    # def back_to_wall(self):
    #     """
    #     Drive back to the position we were at when PICKUP_BUCKET started.
    #     Assumes reset_odometry() was called just before pick_up_basket().
    #     """
    #
    #
    #     self._align_to_wall()
    #
    #     return True

    def _stop_and_pause(self, pause_time=0.1):
        """Stop drivetrain and pause briefly."""
        drivetrain.stop()
        time.sleep(pause_time)

    def _probe_direction(self, left_effort, right_effort, probe_time, threshold=0.15):
        """Probe in one direction for line detection."""
        drivetrain.set_effort(left_effort, right_effort)
        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < probe_time * 1000:
            diff = self.sensor.get_reflectance_difference()
            print("diff:", diff)

            if diff > threshold or diff < -threshold:
                drivetrain.stop()
                return True, diff
            time.sleep(0.02)
        return False, 0

    def _return_to_center(self, left_effort, right_effort, probe_time):
        """Return robot to center position after probing."""
        drivetrain.set_effort(left_effort, right_effort)
        time.sleep(probe_time)
        self._stop_and_pause()

    # ------------------------------------------------------------------------
    # NAVIGATION AND WALL FOLLOWING
    # ------------------------------------------------------------------------
    def _align_to_wall(self):
        print("=== Aligning to wall ===")
        ROTATION_STEP = 10
        MAX_STEPS = 9
        best_dist = self.sensor.get_avg_distance(5, sleep=3)
        best_rot = 0

        steps_taken = 0
        for i in range(1, MAX_STEPS + 1):
            drivetrain.turn(ROTATION_STEP, max_effort=0.4)
            steps_taken = i
            dist = self.sensor.get_avg_distance(5, sleep=1)
            if dist < best_dist:
                best_dist = dist
                best_rot = i
            elif dist > best_dist:
                break

        drivetrain.turn(-ROTATION_STEP * steps_taken, max_effort=0.4)
        time.sleep(1)

        steps_taken = 0
        for i in range(1, MAX_STEPS + 1):
            drivetrain.turn(-ROTATION_STEP, max_effort=0.4)
            steps_taken = i
            dist = self.sensor.get_avg_distance(5, sleep=1)
            if dist < best_dist:
                best_dist = dist
                best_rot = -i
            elif dist > best_dist:
                break

        drivetrain.turn(ROTATION_STEP * (best_rot + steps_taken), max_effort=0.4)
        time.sleep(1)

        current_distance = self.sensor.get_avg_distance(5, sleep=1)
        drivetrain.turn(90, max_effort=0.4)
        error = current_distance - Robot.TARGET_WALL_DISTANCE
        drivetrain.straight(error, max_effort=0.4)
        drivetrain.turn(-90, max_effort=0.4)



        print(f"=== Aligned at {best_rot * ROTATION_STEP} (dist: {best_dist:.1f}cm) ===")


    def _scan_left_for_obj(self, step, angle, search_id):
        assert(search_id == Robot.BoundingBoxId.APPROACHABLE_BUCKET or search_id == Robot.BoundingBoxId.ARROW)
        scan_angle = 0
        num_sightings = 0

        while scan_angle < abs(angle):

            if search_id == Robot.BoundingBoxId.APPROACHABLE_BUCKET:
                time.sleep(2)
                drivetrain.turn(step, 0.5)
                time.sleep(1)

            for i in range(20):
                print(num_sightings, num_sightings)
                info = self.comm.get_bbox_info(min_score=0)

                if search_id == Robot.BoundingBoxId.APPROACHABLE_BUCKET:
                    if info is not None and info["id"] == Robot.BoundingBoxId.APPROACHABLE_BUCKET:
                        print("seen", info["id"])
                        num_sightings += 1
                        if num_sightings >= 3:
                            break

                    if info is not None and info["id"] == Robot.BoundingBoxId.ANGLED_BUCKET:
                        self.bucket_in_view = True

                elif search_id == Robot.BoundingBoxId.ARROW:
                    if info is not None and info["id"] == Robot.BoundingBoxId.ARROW:
                        num_sightings = num_sightings + 1
                        if num_sightings >= 3:
                            break

            if num_sightings >= 3:
                self.state = Robot.State.ALIGN_TO_ARROW if search_id == Robot.BoundingBoxId.ARROW else Robot.State.PICKUP_BUCKET
                return True

            if search_id == Robot.BoundingBoxId.ARROW:
                time.sleep(2)
                drivetrain.turn(step, 0.5)
                time.sleep(1)

            if id_type == Robot.BoundingBoxId.ARROW:
                time.sleep(2)
                drivetrain.turn(step, 0.4)

            scan_angle += step

        return False

    def wall_follow_with_search(self, target_id):
        """
        Test program: follow wall, scan left, correct distance, repeat.
        """
        time.sleep(0.3)
        align_interval = 3
        step_count = 0
        step_distance = 15
        step_distance_when_bucket_visible = 10
        while True:
            step_count += 1
            print(f"\n=== Step {step_count} ===")
            print("GOING FORWARD")
            is_in_bucket_proximity = self.bucket_in_view and self.state == Robot.State.EXPLORE_FOR_BUCKET
            drivetrain.straight(-step_distance_when_bucket_visible if is_in_bucket_proximity else -step_distance, max_effort=0.5)
            print("Scanning")
            found = self._scan_left_for_obj(90, 90, target_id)
            if found:
                return True
            print("Checking wall distance...")
            if self.sensor.get_avg_distance(sleep=1) >= Robot.OBSTACLE_THRESHOLD: #???
                drivetrain.turn(-90, 0.4)

            self._align_to_wall()

    def drive_onto_pressure_plate(self):
        """
        Drive forward (into the plate area) until the plate bounding box
        is not visible for several consecutive frames.
        This prevents stopping too early due to a single lost frame.
        """
        print("Driving into pressure plate area...")

        missing_frames = 0
        max_missing_frames = 2
        drivetrain.reset_encoder_position()
        while True:
            info = self.comm.get_bbox_info(min_score=0.3)

            # --- Case: Plate not detected or wrong ID ---
            if info is None or info["id"] != Robot.BoundingBoxId.PRESSURE_PLATE:
                missing_frames += 1
                print("Plate missing, frame", missing_frames, "/", max_missing_frames)

                if missing_frames >= max_missing_frames:
                    print("Plate gone for several frames  driving forward.")
                    drivetrain.set_effort(0, 0)
                    break

            else:
                # Plate seen again  reset the counter
                missing_frames = 0

            # --- Move forward in small steps ---
            drivetrain.straight(-8, max_effort=0.5)
            time.sleep(0.05)

        # You are now "inside" the plate zone
        self.state = Robot.State.GO_PLATE

    def step_line_follow_and_search(self):
        """
        1) Follow the line.
        2) Every N cycles, stop, turn 90, look at plate.
        3) If plate is centered, drive into it.
        """

        def _check_for_pressure_plate():
            """
            Return True if we see the pressure plate and its bounding box
            is roughly centered horizontally in the image.
            """
            frame_counter = 0
            info = self.comm.get_bbox_info(min_score=0)
            print(info)
            for i in range(4):
                if frame_counter == 4:
                    return False
                info = self.comm.get_bbox_info(min_score=0)

                if info is None or info["id"] != Robot.BoundingBoxId.PRESSURE_PLATE:
                    print("info is none")
                    frame_counter += 1
                    break
                else:
                    print("checking center")
                    x_center = info["x_center"]
                    # Is the plate in the middle of the image?
                    if abs(x_center - Robot.PLATE_X_CENTER) < Robot.PLATE_X_DEADZONE:
                        print("Pressure plate centered! x_center =", x_center)
                        return True

            # print("Plate seen but not centered, x_center =", x_center)
            return False

        self.line_follow()
        print("following line finished")
        self.scan_counter += 1
        if self.scan_counter < 1:
            return  # just keep following line

        # reset counter and do a 90 scan
        self.scan_counter = 0

        print("Stopping to scan for plate...")
        drivetrain.set_effort(0, 0)
        time.sleep(0.1)

        # Turn 90 degrees (choose left or right depending on your setup)
        drivetrain.turn(90, max_effort=0.35)
        time.sleep(3)

        # Update Coral and check if plate is centered

        if _check_for_pressure_plate():
            print("Plate is found")
            # If centered: drive into it until it disappears
            self.drive_onto_pressure_plate()
            return
        else:
            print("No centered plate detected, turning back to line.")

        # Not centered / not found -> turn back and resume line following
        drivetrain.turn(-90, max_effort=0.35)
        time.sleep(0.2)

    def drive_to_arrow(self):
        """
        Align to the arrow (id 3) and drive towards the exit.
        Centers on the arrow horizontally, then drives forward until
        the arrow is no longer visible (we've passed through the exit).
        """
        print("=== drive_to_arrow called ===")

        ARROW_X_TARGET = 0.5
        ARROW_X_DEADZONE = 0.035
        ARROW_TURN_EFFORT = 0.2
        ARROW_DRIVE_EFFORT = 0.4
        DRIVE_INCREMENT = 8  # cm per step

        MAX_MISSING_FRAMES = 15
        missing_frames = 0

        while True:
            info = self.comm.get_bbox_info(min_score=0)

            # --- Handle missing detections ---
            if info is None or info["id"] != Robot.BoundingBoxId.ARROW:
                missing_frames += 1
                print(f"Arrow not detected, missing_frames = {missing_frames}/{MAX_MISSING_FRAMES}")
                drivetrain.set_effort(0, 0)

                if missing_frames >= MAX_MISSING_FRAMES:
                    print("Arrow gone - assuming we passed through exit")
                    return True

                time.sleep(0.05)
                continue
            else:
                missing_frames = 0

            x_center = info["x_center"]
            y_center = info["y_center"]
            print(f"Arrow center: x={x_center:.2f}, y={y_center:.2f}")

            # --- Align horizontally to arrow ---
            if x_center < (ARROW_X_TARGET - ARROW_X_DEADZONE):
                # Arrow is LEFT -> turn left
                print("Arrow left, turning left")
                drivetrain.set_effort(-ARROW_TURN_EFFORT, ARROW_TURN_EFFORT)
                time.sleep(0.05)
                continue

            elif x_center > (ARROW_X_TARGET + ARROW_X_DEADZONE):
                # Arrow is RIGHT -> turn right
                print("Arrow right, turning right")
                drivetrain.set_effort(ARROW_TURN_EFFORT, -ARROW_TURN_EFFORT)
                time.sleep(0.05)
                continue

            else:
                # Centered - stop turning and drive forward
                drivetrain.set_effort(0, 0)
                print("Arrow centered, driving forward")
                time.sleep(0.1)

                # Drive forward a small increment
                drivetrain.straight(-DRIVE_INCREMENT, max_effort=ARROW_DRIVE_EFFORT)
                time.sleep(0.1)

        return True

    def follow_line_until_wall(self):
        while True:
            drivetrain.set_effort(0, 0)
            time.sleep(0.5)

            self.line_follow()

            # Check right-side distance (sensor is mounted on the right)
            dist = self.sensor.get_distance()
            print("Right distance:", dist, "cm")

            # Stop if obstacle is too close
            if dist < 20.0:
                print("Obstacle within 20 cm on the right, stopping.")
                drivetrain.stop()
                drivetrain.straight(-15, max_effort=0.4)
                return True

    def exit_maze(self):
        """Placeholder for exiting maze."""
        pass

    def run(self):
        while True:
            # --- STATE 0: FOLLOW LINE + SEARCH FOR PRESSURE PLATE ---
            if robot.state == Robot.State.FOLLOW_LINE:
                robot.step_line_follow_and_search()

            # --- STATE 1: AT PRESSURE PLATE ---
            elif robot.state == Robot.State.GO_PLATE:
                time.sleep(5)
                robot.state = Robot.State.RETURN_TO_LINE

            # --- STATE 2: RETURN BACK TO LINE AFTER PLATE ---
            elif robot.state == Robot.State.RETURN_TO_LINE:
                is_entered = robot.step_return_to_line()
                time.sleep(4)
                print(is_entered)
                if is_entered:
                    robot.state = Robot.State.ENTER_MAZE

            elif robot.state == Robot.State.ENTER_MAZE:
                is_at_wall = robot.follow_line_until_wall()
                if is_at_wall:
                    robot.state = Robot.State.EXPLORE_FOR_BUCKET

            elif robot.state == Robot.State.EXPLORE_FOR_BUCKET:
                is_bucket_seen = robot.wall_follow_with_search(target_id=Robot.BoundingBoxId.APPROACHABLE_BUCKET)
                if is_bucket_seen:
                    robot.state = Robot.State.PICKUP_BUCKET

            elif robot.state == Robot.State.PICKUP_BUCKET:
                is_basket_picked = robot.pick_up_basket()
                if is_basket_picked:
                    robot.state = Robot.State.BACK_TO_WALL

            elif robot.state == Robot.State.BACK_TO_WALL:
                # robot.back_to_wall()
                is_arrow_found = robot.wall_follow_with_search(target_id=Robot.BoundingBoxId.ARROW)
                if is_arrow_found:
                    robot.state = Robot.State.ALIGN_TO_ARROW

            elif robot.state == Robot.State.ALIGN_TO_ARROW:
                is_arrow_out_of_view = robot.drive_to_arrow()
                if is_arrow_out_of_view:
                    drivetrain.straight(-30, max_effort=0.5)
                    self.find_line_probe()
                    while True:
                        self.line_follow()
                        drivetrain.set_effort(0,0)
                        time.sleep(1)


# ============================================================================
# MAIN EXECUTION
# ============================================================================

robot = StateMachine()
robot.run()




