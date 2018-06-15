#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include <iostream>


#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include "opencv/cxcore.h"

#include "opencv2/core/core.hpp"

#include <stdlib.h>
#include "opencv2/imgproc/imgproc_c.h"

#include "opencv2/opencv.hpp"
#include <cstring>

#include <iostream>
#include <ctime>

#include <fstream>

#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <signal.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <thread>
#include <chrono>
#define PORT 20000
#define LENGTH 5000


////////RGB-H-CbCr + YCrCb color model SKIN DETECTION & FINGER CONTOUR.v2


using namespace cv;

using std::cout;
using std::endl;

std::ofstream inf("text.txt");
int piNum=2;
char* imageName = "image2.jpg";
char* ipAddress = "192.168.0.66";
int bright = 10;

int checking=0;
int secCheck5(int currSec){
    if(currSec==0){
        checking++;
        return checking;
    }else if (currSec==5){
        checking++;
        return checking;
    }else if(currSec==10){
        checking++;
        return checking;
    }else if (currSec==15){
        checking++;
        return checking;
    }else if(currSec==20){
        checking++;
        return checking;
    }else if (currSec==25){
        checking++;
        return checking;
    }else if (currSec==30){
        checking++;
        return checking;
    }else if(currSec==35){
        checking++;
        return checking;
    }else if (currSec==40){
        checking++;
        return checking;
    }else if(currSec==45){
        checking++;
        return checking;
    }else if (currSec==50){
        checking++;
        return checking;
    }else if (currSec==55){
        checking++;
        return checking;
    }else {
        checking=0;
        return checking;
    }
}

int secCheck10(int currSec){
    if(currSec==0){
        checking++;
        return checking;
    }else if (currSec==10){
        checking++;
        return checking;
    }else if(currSec==20){
        checking++;
        return checking;
    }else if (currSec==30){
        checking++;
        return checking;
    }else if(currSec==40){
        checking++;
        return checking;
    }else if (currSec==50){
        checking++;
        return checking;
    }else {
        checking=0;
        return checking;
    }
}

int secCheck3(int currSec){
    if(currSec==0){
        checking++;
        return checking;
    }else if (currSec==3){
        checking++;
        return checking;
    }else if(currSec==6){
        checking++;
        return checking;
    }else if (currSec==9){
        checking++;
        return checking;
    }else if(currSec==12){
        checking++;
        return checking;
    }else if (currSec==15){
        checking++;
        return checking;
    }else if (currSec==18){
        checking++;
        return checking;
    }else if(currSec==21){
        checking++;
        return checking;
    }else if (currSec==24){
        checking++;
        return checking;
    }else if(currSec==27){
        checking++;
        return checking;
    }else if (currSec==30){
        checking++;
        return checking;
    }else if (currSec==33){
        checking++;
        return checking;
    }else if (currSec==36){
        checking++;
        return checking;
    }else if(currSec==39){
        checking++;
        return checking;
    }else if (currSec==42){
        checking++;
        return checking;
    }else if(currSec==45){
        checking++;
        return checking;
    }else if (currSec==48){
        checking++;
        return checking;
    }else if (currSec==51){
        checking++;
        return checking;
    }else if(currSec==54){
        checking++;
        return checking;
    }else if (currSec==57){
        checking++;
        return checking;
    }else {
        checking=0;
        return checking;
    }
}


int secCheck2(int currSec){
    if(currSec==0){
        checking++;
        return checking;
    }else if (currSec==2){
        checking++;
        return checking;
    }else if(currSec==4){
        checking++;
        return checking;
    }else if (currSec==6){
        checking++;
        return checking;
    }else if (currSec==8){
        checking++;
        return checking;
    }else if(currSec==10){
        checking++;
        return checking;
    }else if (currSec==12){
        checking++;
        return checking;
    }else if (currSec==14){
        checking++;
        return checking;
    }else if(currSec==16){
        checking++;
        return checking;
    }else if (currSec==18){
        checking++;
        return checking;
    }else if(currSec==20){
        checking++;
        return checking;
    }else if (currSec==22){
        checking++;
        return checking;
    }else if (currSec==24){
        checking++;
        return checking;
    }else if (currSec==26){
        checking++;
        return checking;
    }else if(currSec==28){
        checking++;
        return checking;
    }else if (currSec==30){
        checking++;
        return checking;
    }else if(currSec==32){
        checking++;
        return checking;
    }else if (currSec==34){
        checking++;
        return checking;
    }else if (currSec==36){
        checking++;
        return checking;
    }else if(currSec==38){
        checking++;
        return checking;
    }else if (currSec==40){
        checking++;
        return checking;
    }else if(currSec==42){
        checking++;
        return checking;
    }else if (currSec==44){
        checking++;
        return checking;
    }else if (currSec==46){
        checking++;
        return checking;
    }else if(currSec==48){
        checking++;
        return checking;
    }else if (currSec==50){
        checking++;
        return checking;
    }else if(currSec==52){
        checking++;
        return checking;
    }else if (currSec==54){
        checking++;
        return checking;
    }else if (currSec==56){
        checking++;
        return checking;
    }else if(currSec==58){
        checking++;
        return checking;
    }else {
        checking=0;
        return checking;
    }
}


void tcp(){

    /* Variable Definition */
    int sockfd;
    int nsockfd;
    char revbuf[LENGTH];
    struct sockaddr_in remote_addr;

    /* Get the Socket file descriptor */
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        fprintf(stderr, "ERROR: Failed to obtain Socket Descriptor! (errno = %d)\n",errno);
        exit(1);
    }

    /* Fill the socket address struct */
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, ipAddress, &remote_addr.sin_addr);
    bzero(&(remote_addr.sin_zero), 8);

    /* Try to connect the remote */
    if (connect(sockfd, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr)) == -1)
    {
        fprintf(stderr, "ERROR: Failed to connect to the host! (errno = %d)\n",errno);
        exit(1);
    }
    else {
        printf("[Client] Connected to server at port %d...ok!\n", PORT);

        char* fs_name = "text.txt";
        char sdbuf[LENGTH];
        printf("[Client] Sending %s to the Server... ", fs_name);
        FILE *fs = fopen(fs_name, "r");
        if(fs == NULL)
        {
            printf("ERROR: File %s not found.\n", fs_name);
            exit(1);
        }

        bzero(sdbuf, LENGTH);
        int fs_block_sz;
        while((fs_block_sz = fread(sdbuf, sizeof(char), LENGTH, fs)) > 0)
        {
            if(send(sockfd, sdbuf, fs_block_sz, 0) < 0)
            {
                fprintf(stderr, "ERROR: Failed to send file %s. (errno = %d)\n", fs_name, errno);
                break;
            }
            bzero(sdbuf, LENGTH);
        }
        printf("Ok File %s from Client was Sent!\n", fs_name);
   }
    
    close(sockfd);
    
    printf("[Client] Connection lost.\n");
}


bool R1(int R, int G, int B) {
	/*
    bool e1 = (R>130) && (G>130) && (B>20) && ((max(R,max(G,B)) - min(R, min(G,B)))>15) && (abs(R-G)>15) && (R>G) && (R>B);
    bool e2 = (R>220) && (G>210) && (B>170) && (abs(R-G)<=15) && (R>B) && (G>B);
    */

    bool e1 = (R>100) && (G>100) && (B>10) && ((max(R,max(G,B)) - min(R, min(G,B)))>15) && (abs(R-G)>15) && (R>G) && (R>B);
    bool e2 = (R>220) && (G>210) && (B>170) && (abs(R-G)<=15) && (R>B) && (G>B);


    return (e1||e2);
}

bool R2(float Y, float Cr, float Cb) {
    bool e3 = Cr <= 1.5862*Cb+20;
    bool e4 = Cr >= 0.3448*Cb+76.2069;
    bool e5 = Cr >= -4.5652*Cb+234.5652;
    bool e6 = Cr <= -1.15*Cb+301.75;
    bool e7 = Cr <= -2.2857*Cb+432.85;
    return e3 && e4 && e5 && e6 && e7;
}

bool R3(float H, float S, float V) {
    return (H<25) || (H > 230);
}

Mat GetSkin(Mat const &src) {
    // allocate the result matrix
    Mat dst = src.clone();

    Vec3b cblack = Vec3b::all(0);

    Mat src_ycrcb, src_hsv;
    // OpenCV scales the YCrCb components, so that they
    // cover the whole value range of [0,255], so there's
    // no need to scale the values:
    cvtColor(src, src_ycrcb, CV_BGR2YCrCb);
    // OpenCV scales the Hue Channel to [0,180] for
    // 8bit images, so make sure we are operating on
    // the full spectrum from [0,360] by using floating
    // point precision:
    src.convertTo(src_hsv, CV_32FC3);
    cvtColor(src_hsv, src_hsv, CV_BGR2HSV);
    // Now scale the values between [0,255]:
    normalize(src_hsv, src_hsv, 0.0, 255.0, NORM_MINMAX, CV_32FC3);

    for(int i = 0; i < src.rows; i++) {
        for(int j = 0; j < src.cols; j++) {

            Vec3b pix_bgr = src.ptr<Vec3b>(i)[j];
            int B = pix_bgr.val[0];
            int G = pix_bgr.val[1];
            int R = pix_bgr.val[2];
            // apply rgb rule
            bool a = R1(R,G,B);

            Vec3b pix_ycrcb = src_ycrcb.ptr<Vec3b>(i)[j];
            int Y = pix_ycrcb.val[0];
            int Cr = pix_ycrcb.val[1];
            int Cb = pix_ycrcb.val[2];
            // apply ycrcb rule
            bool b = R2(Y,Cr,Cb);

            Vec3f pix_hsv = src_hsv.ptr<Vec3f>(i)[j];
            float H = pix_hsv.val[0];
            float S = pix_hsv.val[1];
            float V = pix_hsv.val[2];
            // apply hsv rule
            bool c = R3(H,S,V);

            if(!(a&&b&&c))
                dst.ptr<Vec3b>(i)[j] = cblack;
        }
    }
    return dst;
}


Point2f clickPoint = Point2f(0.0,0.0); //BLUE circle point

int frameUnit=3, frameUnitMid=1;  // 3 frames 단위  +-3.5 오차범위
float errorRange= 3.5;

int comparePoints(std::vector<Point2f> points){
	time_t curr_time;
    struct tm *curr_tm;
    curr_time = time(NULL);
    curr_tm = localtime(&curr_time);



    std::vector<Point2f> differ;
    if(points.size()==frameUnit){
        for(int i=0; i<(frameUnit-1); i++){
            differ.push_back(Point2f(fabs(points[i+1].x-points[i].x),fabs(points[i+1].y-points[i].y)));
        }

        float differX=0.0, differY=0.0;
        for(int i=0; i<(frameUnit-1); i++){
            differX += differ[i].x;
            differY += differ[i].y;
        }
        differX = differX/(frameUnit-1);
        differY = differY/(frameUnit-1);

        if(0.0<=differX && differX<errorRange){
            if(0.0<=differY && differY<=errorRange){
                clickPoint = points[frameUnitMid];
                std::cout << piNum << "/" << curr_tm->tm_hour << "/" << curr_tm->tm_min << "/" << curr_tm->tm_sec << "/" << points[frameUnitMid].x << "/" << points[frameUnitMid].y << endl;


                inf<<piNum;
                inf<<"/";
                inf<<curr_tm->tm_hour;
                inf<<"/";
                inf<<curr_tm->tm_min;
                inf<<"/";
                inf<<curr_tm->tm_sec;
                inf<<"/";
                inf<<points[frameUnitMid].x;
                inf<<"/";
                inf<<points[frameUnitMid].y<<std::endl;
            }
        }
        differ.clear();
        return 1;
    } else
        return 0;
}



int main()
{


	Mat tmpImg, handImg, mask;
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	std::vector<Point2f> points;
	
	
	time_t curr_time, curr_time2;
    	struct tm *curr_tm, *curr_tm2;
    	int startMinute = 0;

    	std::cout << "Enter bright(int):";
    	std::cin >> bright;
	
	std::cout << "Enter start minute(0~59):";
    	std::cin >> startMinute;

    	while(1){
        	curr_time = time(NULL);
        	curr_tm = localtime(&curr_time);
        	if(curr_tm->tm_min==startMinute){
            		break;
        	}
    	}



	VideoCapture video(0);
	if (!video.isOpened()) return 0;

    //VideoCapture video("C:/Users/macbook/Desktop/record3/new_ul01.h264"); //dl005 dr002bigerror dr005error 사각형 사이즈 범위를 정하는게 좋을듯
	//VideoCapture video("new_dr00.h264");
    Mat image;



    //namedWindow("hand1_image", CV_WINDOW_AUTOSIZE);
	//namedWindow("hand2_image", CV_WINDOW_AUTOSIZE);
	namedWindow("original_image", CV_WINDOW_AUTOSIZE);

	while (true)
	{
		curr_time2 = time(NULL);
        	curr_tm2 = localtime(&curr_time2);

		if(secCheck5(curr_tm2->tm_sec)==1){
			inf.close();

			tcp();
			std::cout << "do tcp() at "<< curr_tm2->tm_sec << "sec" << endl;

			inf.open("text.txt");
		}

	    	video >> image;
		if (image.empty()) break;
	//video.read(image);


		image = image(Rect(100,100,510,210));
		imwrite(imageName, image);
		
		image = image + Scalar(bright, bright, bright);

        tmpImg = GetSkin(image);
        //tmpImg = image;

		cvtColor(tmpImg, handImg, CV_BGR2YCrCb);
		inRange(handImg, Scalar(0, 133, 77), Scalar(255, 173, 127), handImg);

		mask = handImg.clone();

		findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, Point(0,0));

		int largestContour = 0;
		for (int i = 0; i < contours.size(); i++) {
            //double a = contourArea(contours[i]);
			if (contourArea(contours[i]) > contourArea(contours[largestContour])) {
				largestContour = i;
			}
		}

		drawContours(image, contours, largestContour, Scalar(0, 255, 255), 1, 8, std::vector <Vec4i>(), 0, Point());	//YELLOW contour 洹몃━湲?
        //5th index must be -1 for drawing only one contour from present Contours variable)


        // loop through the contours/hierarchy
        for (int i=0; i<contours.size(); i++) {
            if(i==largestContour && contourArea(contours[i])>100){  //선택된 윤곽선만 blue 사각형 그리기
                if (hierarchy[i][3]==-1 && arcLength(contours[i], true)!=arcLength(contours[i], false)) {
                    std::vector<std::vector<Point> >hull(1);
                    convexHull(Mat(contours[i]), hull[0], false);
                    drawContours(image, hull, 0, Scalar(0, 255, 0), 1, 8, std::vector<Vec4i>(), 0, Point());		//GREEN 점 잇기
                    if(1){
                        std::vector<RotatedRect> minRect( contours.size() );
                        minRect[i] = minAreaRect( Mat(contours[i]) );
                        Point2f rect_points[4];
                        minRect[i].points( rect_points );
                        for( int j = 0; j < 4; j++ )
                            line( image, rect_points[j], rect_points[(j+1)%4], Scalar(255, 0, 0), 1, 8 );   //BLUE 사각형 그리기
                        //std::cout<<rect_points[2]<<std::endl;          //point 좌표 값 print하기

                        points.push_back(Point2f(rect_points[2].x,rect_points[2].y));
                        if(comparePoints(points)){
                            if(clickPoint.x!=0.0 && clickPoint.y!=0.0){
                                circle(image, clickPoint, 10, Scalar(255, 0, 0), 10); //BLUE circle
                                clickPoint = Point2f(0.0,0.0);
                            }
                            points.clear();
                        }
                    }
                }
            }
        }

        //Size(960,540) , Size(800,450) , Size(720,405)
		//resize(image, image, Size(960,540), 0, 0, CV_INTER_LINEAR);
		//resize(tmpImg, tmpImg, Size(720,405), 0, 0, CV_INTER_LINEAR);
        //resize(handImg, handImg, Size(720,405), 0, 0, CV_INTER_LINEAR);

		//imshow("hand1_image", tmpImg);
		//imshow("hand2_image", handImg);
		imshow("original_image", image);

		if (waitKey(10)>0)
			break;
	}

	video.release();
	tmpImg.release();
    handImg.release();
    image.release();

	destroyAllWindows();

	inf.close();

	return 0;
}
