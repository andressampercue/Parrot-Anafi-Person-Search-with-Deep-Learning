# Parrot-Anafi-Person-Search-with-Deep-Learning
The main objective of this project is use a Parrot Anafi Drone to search people who are missing and their health state is unknown. Temporarly the project can be tested with Parrot-Sphinx simulator on Linux. Any contribution is accepted. The project is developed under ROS2 and C++, with YOLO v8. 

# Logic 
- A flightplan is uploaded on drone's memory (flightplans can be created with QGroundControl and do a conversion from .flightplan to .mavlink).
- The drone starts to follow the flightplan from it's starting point.
- The drone's camera points to the ground via a command.
- The frames are proccesed with the person recognition model created with YOLOv8.
- If a person is detected, the drone relocates itself with the coordinate of the nose of the persone, calculating distances according to the focal lenght of the camera, height, etc.
- When the drone is already on the pont, and the event is confirmed, the drone descends to a safety height above the person's face, and takes vital signs, for example, heartrate and oximetry, with the help of another external package that uses computer vision to procces RGB signals to extract and calculate this data.
- The health state of the person is sent to the master computer and verified.
- The drone reubicates itself to the point where the person was detected, and continues with the rest of the flightplan in search of other people.
