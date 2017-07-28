# Rotation_Tracking
Track the camera's rotation through an image sequence(Video). It is used to detected head orientation changes of a person.

	
## Usage

Requirement:

	1.OpenCV 3.0 or later (for IO and ORB features, necessary)

	2.Eigen library to do matrix calculation.

Build:

        1.mkdir & cd build

	2.cmake ..
	
	3.make
	
Note:
        
	The output is in the following format. 
	
	Frame NO.  YAW_C   PITCH_C   ROLL_C
	           YAW_M   PITCH_M   ROLL_M
	
	where the frame NO. is the number of the frame, the rest are euler rotation angles. Posfix C indicating the result comes from camera while M suggests motion sensor (IMU).
	
	The video file and the parameter file should be of your own. As the video file is too large, I don't upload the original data.
