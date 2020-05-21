all:
	g++ -std=c++11 main.cpp -lopencv_objdetect -lopencv_imgcodecs -lwiringPi  -lopencv_imgproc -lopencv_videoio -lopencv_core -lopencv_highgui -o test 
