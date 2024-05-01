import matplotlib.pyplot as plt
import numpy as np
import cv2
import time
import typing
from cortexvision import detect_target


cap = cv2.VideoCapture('dji_car_track.mp4')

# Frame rate
frame_rate = cap.get(cv2.CAP_PROP_FPS)

# Define the codec and create VideoWriter object for mp4
fourcc = cv2.VideoWriter_fourcc(*'mp4v')

# Get video dimensions
width = int(cap.get(3))
height = int(cap.get(4))

# Define the output file with .mp4 extension
out = cv2.VideoWriter('output.mp4', fourcc, frame_rate, (width, height))


class Line:
    def __init__(self, x1:int, y1:int, x2:int, y2:int, color:tuple=(255, 255, 255), thickness:int=1) -> None:
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2 
        self.y2 = y2
        self.color = color
        self.thickness = thickness

    def update(self, x1:int, y1:int, x2:int, y2:int) -> None:
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2 
        self.y2 = y2

    def get_division_point(self, division_ratio:float) -> tuple:
        x = self.x1 + int((self.x2 - self.x1) * division_ratio)
        y = self.y1 + int((self.y2 - self.y1) * division_ratio)
        return (x, y)
    

    
    def draw(self, frame):
        cv2.line(frame, (self.x1, self.y1), (self.x2, self.y2), self.color, self.thickness)

    # Destructor
    def __del__(self):
        print("Line object deleted")


class CenterLine(Line):
    def __init__(self, cx:int, cy:int, length:int, angle:float, color:tuple=(255, 255, 255), thickness:int=1):
        self.cx = cx
        self.cy = cy
        self.length = length
        # Angle into radians
        self.angle = np.deg2rad(angle)
        self.color = color
        self.thickness = thickness

        self.x1 = self.cx - int(self.length * np.cos(self.angle) * .5)
        self.y1 = self.cy - int(self.length * np.sin(self.angle) * .5)
        self.x2 = self.cx + int(self.length * np.cos(self.angle) * .5)
        self.y2 = self.cy + int(self.length * np.sin(self.angle) * .5)
        super().__init__(self.x1, self.y1, self.x2, self.y2, self.color, self.thickness)

    def update(self, cx:int, cy:int, length:int, angle:float) -> None:
        self.cx = cx
        self.cy = cy
        self.length = length
        # Angle into radians
        self.angle = np.deg2rad(angle)

        self.x1 = self.cx - int(self.length * np.cos(self.angle) * .5)
        self.y1 = self.cy - int(self.length * np.sin(self.angle) * .5)
        self.x2 = self.cx + int(self.length * np.cos(self.angle) * .5)
        self.y2 = self.cy + int(self.length * np.sin(self.angle) * .5)


class CornerLine(Line):
    def __init__(self, x1:int, y1:int, length:int, angle:int, color=(255, 255, 255), thickness=1):
        self.x1 = x1
        self.y1 = y1
        self.length = length
        # Angle into radians
        self.angle = np.deg2rad(angle)
        self.color = color
        self.thickness = thickness

        self.x2 = self.x1 + int(self.length * np.cos(self.angle))
        self.y2 = self.y1 + int(self.length * np.sin(self.angle))
        super().__init__(self.x1, self.y1, self.x2, self.y2, self.color, self.thickness)

    def update(self, x1:int, y1:int, length:int, angle:int, color=(255, 255, 255), thickness=1) -> None:
        self.x1 = x1
        self.y1 = y1
        self.length = length
        # Angle into radians
        self.angle = np.deg2rad(angle)

        self.color = color
        self.thickness = thickness

        self.x2 = self.x1 + int(self.length * np.cos(self.angle))
        self.y2 = self.y1 + int(self.length * np.sin(self.angle))


class Scale():
    def __init__(self, line:Line, divisions: int):
        self.main_line = line   

        self.angle = np.arctan2(self.main_line.y2 - self.main_line.y1, self.main_line.x2 - self.main_line.x1)
        # Convert angle to degrees
        self.angle = np.rad2deg(self.angle)

        # Get the vertical angle to the main line
        self.scale_angle = self.angle + 90

        self.divisions = divisions

        self.division_lines = []

        for i in range(0, self.divisions+1):
            center_point = self.main_line.get_division_point(i/self.divisions)
            self.division_lines.append(CenterLine(center_point[0], center_point[1], 10, self.scale_angle))
        
    def draw(self, frame):
        self.main_line.draw(frame)
        for line in self.division_lines:
            line.draw(frame)


class Circle:
    def __init__(self, x, y, radius, color=(255, 255, 255), thickness=2):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.thickness = thickness
    
    def draw(self, frame):
        cv2.circle(frame, (self.x, self.y), self.radius, self.color, self.thickness)

# Class Dot is subclass of Circle
class Dot(Circle):
    def __init__(self, x, y, radius=1, color=(255, 255, 255), thickness=2):
        super().__init__(x, y, radius, color, thickness)

class TrackerMarker:
    def __init__(self, x1, y1, x2, y2, color=(0, 255, 0), thickness=1):
        
        self.width_ratio = .2
        self.height_ratio = .2
        
        # Estimate the center of the marker
        self.cx = (x1 + x2) // 2
        self.cy = (y1 + y2) // 2

        # Estimate the width and height of the marker
        self.width = x2 - x1
        self.height = y2 - y1

        # Estimate the 4 corners of the marker
        self.t_l_x = x1
        self.t_l_y = y1

        self.t_r_x = x2
        self.t_r_y = y1

        self.b_l_x = x1
        self.b_l_y = y2

        self.b_r_x = x2
        self.b_r_y = y2

        self.color = color

        self.thickness = thickness

        self.t_l_h_line = CornerLine(self.t_l_x, self.t_l_y, int(self.width * self.width_ratio), 0, self.color, self.thickness)
        self.t_l_v_line = CornerLine(self.t_l_x, self.t_l_y, int(self.height * self.height_ratio), 90, self.color, self.thickness)

        self.t_r_h_line = CornerLine(self.t_r_x, self.t_r_y, int(self.width * self.width_ratio), 180, self.color, self.thickness)
        self.t_r_v_line = CornerLine(self.t_r_x, self.t_r_y, int(self.height * self.height_ratio), 90, self.color, self.thickness)
        
        self.b_l_h_line = CornerLine(self.b_l_x, self.b_l_y, int(self.width * self.width_ratio), 0, self.color, self.thickness)
        self.b_l_v_line = CornerLine(self.b_l_x, self.b_l_y, int(self.height * self.height_ratio), 270, self.color, self.thickness)

        self.b_r_h_line = CornerLine(self.b_r_x, self.b_r_y, int(self.width * self.width_ratio), 180, self.color, self.thickness)
        self.b_r_v_line = CornerLine(self.b_r_x, self.b_r_y, int(self.height * self.height_ratio), 270, self.color, self.thickness)


        # All lines
        self.lines = [self.t_l_h_line, self.t_r_h_line, self.b_l_h_line, self.b_r_h_line, self.t_l_v_line, self.t_r_v_line, self.b_l_v_line, self.b_r_v_line]
        

    def update(self, x1, y1, x2, y2, color=(0, 255, 0), thickness=1):

        self.cx = (x1 + x2) // 2
        self.cy = (y1 + y2) // 2

        self.color = color
        self.thickness = thickness

        # Estimate the width and height of the marker
        self.width = x2 - x1
        self.height = y2 - y1

        # Estimate the 4 corners of the marker
        self.t_l_x = x1
        self.t_l_y = y1

        self.t_r_x = x2
        self.t_r_y = y1

        self.b_l_x = x1
        self.b_l_y = y2

        self.b_r_x = x2
        self.b_r_y = y2

        # Update the lines
        self.t_l_h_line.update(self.t_l_x, self.t_l_y, int(self.width * self.width_ratio), 0, self.color, self.thickness)
        self.t_l_v_line.update(self.t_l_x, self.t_l_y, int(self.height * self.height_ratio), 90, self.color, self.thickness)

        self.t_r_h_line.update(self.t_r_x, self.t_r_y, int(self.width * self.width_ratio), 180, self.color, self.thickness)
        self.t_r_v_line.update(self.t_r_x, self.t_r_y, int(self.height * self.height_ratio), 90, self.color, self.thickness)

        self.b_l_h_line.update(self.b_l_x, self.b_l_y, int(self.width * self.width_ratio), 0, self.color, self.thickness)
        self.b_l_v_line.update(self.b_l_x, self.b_l_y, int(self.height * self.height_ratio), 270, self.color, self.thickness)

        self.b_r_h_line.update(self.b_r_x, self.b_r_y, int(self.width * self.width_ratio), 180, self.color, self.thickness)
        self.b_r_v_line.update(self.b_r_x, self.b_r_y, int(self.height * self.height_ratio), 270, self.color, self.thickness)

    def draw(self, frame):
        for line in self.lines:
            line.draw(frame)

class Stat:
    def __init__(self, frame, key, x, y, color=(255, 255, 255), thickness=1):
        self.frame = frame
        self.key = key
        self.value = ""
        self.text = f"{self.key}: {self.value}"

        self.x = x
        self.y = y

        self.color = color
        self.thickness = thickness

    def update(self, value):
        self.value = value
        self.text = f"{self.key}: {self.value}"
    
    def draw(self, frame):
        cv2.putText(frame, self.text, (self.x, self.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color, self.thickness)

class AltitudeMeter(Stat):
    def __init__(self, frame):
        # Put the altitude meter on the top left corner offsetting 5% of the frame width and height
        self.x = int(frame.shape[1] * .05)
        self.y = int(frame.shape[0] * .05)
        super().__init__(frame, "ALTITUDE", self.x, self.y)

class TargetDistance(Stat):
    def __init__(self, frame):
        # Put the target distance on top left corner offsetting 5% of the frame width 10% of the frame height
        self.x = int(frame.shape[1] * .05)
        self.y = int(frame.shape[0] * .1)
        super().__init__(frame, "EST_DISTANCE", self.x, self.y)


class Reticle:
    def __init__(self, frame):
        self.frame = frame

        self.lines = []

        self.targeting_zone_ratio = .25
        self.reticle_start_ratio = .1

        # Get the frame width and height
        self.frame_width = int(frame.shape[1])
        self.frame_height = int(frame.shape[0])


        # Frame center coordinates
        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2

        # Target zone rectangle coordinates: 40% of frame width and height
        self.target_zone_width = int(self.frame_width * self.targeting_zone_ratio)
        self.target_zone_height = int(self.frame_height * self.targeting_zone_ratio)

        # Reticle start coordinates: 10% of frame width and height
        self.reticle_start_h_offset = int(self.frame_width * self.reticle_start_ratio)
        self.reticle_start_v_offset = int(self.frame_height * self.reticle_start_ratio)

        # Target zone rectangle coordinates
        self.target_zone_x1 = self.frame_center_x - self.target_zone_width // 2
        self.target_zone_y1 = self.frame_center_y - self.target_zone_height // 2
        self.target_zone_x2 = self.frame_center_x + self.target_zone_width // 2
        self.target_zone_y2 = self.frame_center_y + self.target_zone_height // 2

        # Horizontal and vertical lines avoiding the target zone
        self.h_line1 = Line(self.reticle_start_h_offset, self.frame_center_y, self.target_zone_x1, self.frame_center_y)
        self.h_line2 = Line(self.target_zone_x2, self.frame_center_y, self.frame_width - self.reticle_start_h_offset, self.frame_center_y)
        self.v_line1 = Line(self.frame_center_x, self.reticle_start_v_offset, self.frame_center_x, self.target_zone_y1)
        self.v_line2 = Line(self.frame_center_x, self.target_zone_y2, self.frame_center_x, self.frame_height - self.reticle_start_v_offset)

        # Scales avoiding the target zone
        h_scale_line1 = Scale(self.h_line1, 4)
        h_scale_line2 = Scale(self.h_line2, 4)
        v_scale_line1 = Scale(self.v_line1, 4)
        v_scale_line2 = Scale(self.v_line2, 4)

        # Dots
        self.dot_center = Dot(self.frame_center_x, self.frame_center_y)

        self.dots = [self.dot_center]

        self.scales = [h_scale_line1, h_scale_line2, v_scale_line1, v_scale_line2]


    def check_inside_target_zone(self, x, y):
        if x > self.target_zone_x1 and x < self.target_zone_x2 and y > self.target_zone_y1 and y < self.target_zone_y2:
            return True
        else:
            return False
        
    def check_inside_killzone(self, trackerMarker:TrackerMarker):
        # Check whether the area of the tracker marker is bigger than the target zone area and the center of the tracker marker is closer than 30% of the target zone radius
        # if trackerMarker.width * trackerMarker.height > self.target_zone_width * self.target_zone_height and np.sqrt((trackerMarker.cx - self.frame_center_x)**2 + (trackerMarker.cy - self.frame_center_y)**2) < np.sqrt(self.target_zone_width**2 + self.target_zone_height**2) * .3:
        if np.sqrt((trackerMarker.cx - self.frame_center_x)**2 + (trackerMarker.cy - self.frame_center_y)**2) < np.sqrt(self.target_zone_width**2 + self.target_zone_height**2) * .1:
            return True
        else:
            return False

    # def estimate_distance(self, trackerMarker:TrackerMarker):
    #     # Try to estimate the distance from the target according to the area of the tracker marker
    #     # If the area is bigger than 50% of frame area, the distance is 1


    def draw(self, frame):
        for scale in self.scales:
            scale.draw(frame)
        for dot in self.dots:
            dot.draw(frame)



class DroneVision:
    def __init__(self, cap: cv2.VideoCapture):
        self.cap = cap

        self.zoom = 1

        # Get the frame width and height
        self.frame_width = int(cap.get(3) * self.zoom) 
        self.frame_height = int(cap.get(4) * self.zoom)

        # Initial Frame is Black RGB
        self.frame = np.zeros((self.frame_height, self.frame_width, 3), np.uint8)


        self.reticle = Reticle(self.frame)
        self.reticle.draw(self.frame)

        self.tracker_marker = TrackerMarker(100, 100, 200, 200)
        self.tracker_marker.draw(self.frame)

        self.altitude_meter = AltitudeMeter(self.frame)
        self.altitude_meter.draw(self.frame)

        self.distance_estimator = TargetDistance(self.frame)
        self.distance_estimator.draw(self.frame)



    def update_frame(self):
        ret, frame = self.cap.read()
        # Convert frame to grayscale
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # # Convert frame to negative
        # frame = cv2.bitwise_not(frame)
        # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        # Resize frame
        # frame = cv2.resize(frame, (self.frame_width, self.frame_height))

        highest_conf = 0
        highest_conf_box = None
        for box in detect_target.inference_img(frame):
            # Select the box with the highest confidence
            if box['conf'] > highest_conf:
                highest_conf = box['conf']
                highest_conf_box = box
        
        if highest_conf_box is not None:
            if self.reticle.check_inside_target_zone(highest_conf_box['cx'], highest_conf_box['cy']):
                self.tracker_marker.update(highest_conf_box['x1'], highest_conf_box['y1'], highest_conf_box['x2'], highest_conf_box['y2'], (0, 0, 255), 1)
                if self.reticle.check_inside_killzone(self.tracker_marker):
                    self.tracker_marker.update(highest_conf_box['x1'], highest_conf_box['y1'], highest_conf_box['x2'], highest_conf_box['y2'], (0, 0, 255), 3)
            else:
                self.tracker_marker.update(highest_conf_box['x1'], highest_conf_box['y1'], highest_conf_box['x2'], highest_conf_box['y2'])
        else:
            self.tracker_marker.update(0, 0, 0, 0)
        # self.frame = frame
        self.reticle.draw(frame)

        # Update the tracker marker coordinates according to mouse position
        

        self.tracker_marker.draw(frame)

        self.altitude_meter.draw(frame)

        self.distance_estimator.draw(frame)
        self.frame = frame

if __name__ == "__main__":

    cv2.namedWindow("Drone Vision")
    drone_vision = DroneVision(cap)
    while True:
        # drone_vision.update_frame()
        try:
            drone_vision.update_frame()
            cv2.imshow("Drone Vision", drone_vision.frame)
            out.write(drone_vision.frame)
        except Exception as e:
            print(e)
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()