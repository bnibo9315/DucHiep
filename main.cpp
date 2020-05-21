#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <stdio.h>
#include <chrono>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdlib.h>
//Enable LCD
#define I2C_ADDR   0x27 // I2C device address

// Define some device constants
#define LCD_CHR  1 // Mode - Sending data
#define LCD_CMD  0 // Mode - Sending command

#define LINE1  0x80 // 1st line
#define LINE2  0xC0 // 2nd line

#define LCD_BACKLIGHT   0x08  // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE  0b00000100 // Enable bit
void lcd_init(void);
void lcd_byte(int bits, int mode);
void lcd_toggle_enable(int bits);

#define motorDC 4
//Define value
void typeInt(int i);
void typeFloat(float myFloat);
void lcdLoc(int line); //move cursor
void ClrLcd(void); // clr LCD return home
void typeln(const char *s);
void typeChar(char val);
int fd;  // seen by all subroutines

//Detect cam
using namespace std::chrono;
unsigned long long getCurMs() {
  milliseconds ms = duration_cast< milliseconds >(
    system_clock::now().time_since_epoch()
  );
  return ms.count();
}
using namespace std;
using namespace cv;

cv::VideoCapture cam(1);
cv::Mat pic;
cv::Mat takePicture(){
    while (!cam.isOpened()) {
        std::cout << "Failed to make connection to cam" << std::endl;
        cam.open(0);
    }
    cam>>pic;
    return pic;
}
int main(int n , char *arv[]){
    //cv::waitKey(1000);
    if (wiringPiSetup () == -1) exit (1);
    fd = wiringPiI2CSetup(I2C_ADDR);
    lcd_init();
    pinMode(motorDC, OUTPUT);
    digitalWrite(motorDC, LOW);
    string faceCascadePath = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml";
    CascadeClassifier faceCascade;
    faceCascade.load( faceCascadePath );
    cv::Mat img;
    lcdLoc(LINE1);
    typeln(" THAI DUC HIEP");
    lcdLoc(LINE2);
    typeln("MSSV : ");
    while (true) {
      img = takePicture();
      vector<Rect> faces;
      Mat  gray;
      cvtColor(img, gray, CV_BGR2GRAY);
      auto startMs = getCurMs();
      faceCascade.detectMultiScale(gray, faces, 1.3, 5);
      auto endMs = getCurMs();
      printf("detect %u faces, exe time=%llu\n", faces.size(), endMs- startMs);
      lcdLoc(LINE1);
      typeln("  THAI DUC HIEP ");
      lcdLoc(LINE2);
      typeln(" Detect ");
      typeInt(faces.size());
      typeln(" Faces");
      if(faces.size() > 0)
      {
         digitalWrite(motorDC, HIGH);
      }
      else
      {
       digitalWrite(motorDC,LOW);
      }
     // for ( size_t i = 0; i < faces.size(); i++ )
     // {
       //  int x1 = faces[i].x;
        // int y1 = faces[i].y;
        // int x2 = faces[i].x + faces[i].width;
        // int y2 = faces[i].y + faces[i].height;
         //cv::Point pt1(x1, y1);
         //cv::Point pt2(x2, y2);
         //cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0));
     // }

      //imwrite("camera.jpg", img);
      //cv::waitKey();
    }
}

void lcd_init()   {
  // Initialise display
  lcd_byte(0x33, LCD_CMD); // Initialise
  lcd_byte(0x32, LCD_CMD); // Initialise
  lcd_byte(0x06, LCD_CMD); // Cursor move direction
  lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  lcd_byte(0x01, LCD_CMD); // Clear display
  delayMicroseconds(500);
}
void typeFloat(float myFloat)   {
  char buffer[20];
  sprintf(buffer, "%4.2f",  myFloat);
  typeln(buffer);
}

// int to string
void typeInt(int i)   {
  char array1[20];
  sprintf(array1, "%d",  i);
  typeln(array1);
}

// clr lcd go home loc 0x80
void ClrLcd(void)   {
  lcd_byte(0x01, LCD_CMD);
  lcd_byte(0x02, LCD_CMD);
}

// go to location on LCD
void lcdLoc(int line)   {
  lcd_byte(line, LCD_CMD);
}

// out char to LCD at current position
void typeChar(char val)   {

  lcd_byte(val, LCD_CHR);
}

void typeln(const char *s)   {

  while ( *s ) lcd_byte(*(s++), LCD_CHR);

}

void lcd_byte(int bits, int mode)   {

  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
  delayMicroseconds(500);
}
