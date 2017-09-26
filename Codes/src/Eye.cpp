/*
  The Hub : Central communication system of the without
  Robot Arena, controls information flow in the system

  Modified: 20.06.2015 by Mehmet Efe Tiryaki (m.efetiryaki@gmail.com)
  Created : 18.05.2015 by Yunus Emre Badem (yebadem@gmail.con)
*/

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ctime>
#include <math.h>
#include "ros/ros.h"
#include "without/Vector4.h"
#include "without/Map.h"
#include "without/flatten2DArray.h"
#include "without/EyeData.h"
#include <vector>

#define PI 3.14159265
using namespace cv;
using namespace std;
//using namespace ros;
using namespace without; // WOW such namespace much C++

class Eye
{ public:
    // image arrays to hold threshold and dilated images
    Mat img,dilated_img,small_img,bg_img; 
    Mat markers,stats,centroids,small_markers,small_stats,small_centroids;
    double dWidth;
    double dHeight;
    // vectors to hold robot positions
    vector<Vector4> robot;
    VideoCapture cap;


    Eye();
    vector<Vector4> locate();
    void background();
    bool sendLocation(without::EyeData::Request&,
                      without::EyeData::Response&);

};
Eye::Eye()
{

  // Open Camera 
  cap=VideoCapture(0);
  cap.set(4,1200);
  cap.set(3,1600);
  cap.read(img);

  //get the width of frames of the video
  dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 

   //get the height of frames of the  video
  dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

}

/*
    This Function locates robots and robot a vector<>
    containing Vector4()
*/
vector<Vector4> Eye::locate()
{

    // take image from camare
    cap.read(img);

    // Crop area of interest (The Table)
    Rect ROI(100,250,dWidth-100,750);
    img= img(ROI);

   // Gray scale image
    cvtColor(img,img,CV_BGR2GRAY);
    
    // Threshold 
    threshold(img,img,100, 255, THRESH_BINARY_INV );
    
    // Dilate to close the white regions in stickers
    dilate(img,dilated_img, Mat(),Point(-1,-1),1,1,1);
   
    // Find Components 
    connectedComponentsWithStats(dilated_img, markers, stats, centroids);
    
    // Generate robot positions
    int numberOfArea=stats.size().height;
    // Characteristic length of the component
    int length=0;
    //  Number of robot found
    int robotNo=0;
    // [position,ID] of robots 
    vector<Vector4> robot;

    // Check every Component if they are the Big Marker on a sticker
    for(int i=1 ;i<numberOfArea;i++){
      // Characteristic length calculation
      length=sqrt(stats.at<int>(i,4));

        // Check if one of the other components is the Small Marker of same Sticker
        for (int j=1;j<=numberOfArea;j++){
            if (j!=i){
              // Calculate distance between Markers
              int distance= pow(centroids.at<double>(i,0)-centroids.at<double>(j,0),2 )+
                           pow( centroids.at<double>(i,1)-centroids.at<double>(j,1),2 );
              // If Big Marker's Char. Length is greater than distance between centroid
              // it is a Sticker
              if(length*length>distance ){
                  int x,y,w,h;
                  x=centroids.at<double>(i,0)-stats.at<int>(i,2);
                  y=centroids.at<double>(i,1)-stats.at<int>(i,3);
                  w=stats.at<int>(i,2)*2;
                  h=stats.at<int>(i,3)*2;
      	          // After locating a Sticker, find the identity
                  // Check the position to avoid from crop faileures
            	    if(x>0&& y>0 && x+w<img.cols && y+h<img.rows){
                          // Crop the related area around sticker to aviod costy component search
            	    	      Rect ROI( x,y,w,h);
                        	small_img=img(ROI);
            	    	      connectedComponentsWithStats(small_img,small_markers,small_stats,small_centroids);
                        	Vector4 seen_robot=Vector4();
            		          seen_robot.w=small_stats.size().height;
                          // calculate X, Y positions
                        	seen_robot.x=centroids.at<double>(i,0);
                        	seen_robot.y=centroids.at<double>(i,1);
                          // calculate heading
                        	seen_robot.z=180/PI*-1*atan2(centroids.at<double>(j,1)-centroids.at<double>(i,1),
                                              centroids.at<double>(j,0)-centroids.at<double>(i,0));
                          // Add to robot vector
                        	robot.push_back(seen_robot);
                          // increase number of robots
                        	robotNo++;
                        	break;
            	    }
              }
          }
      }
  }
	return robot;
}

/*
    This Fucntion take a image of the background and 
    keep in memory
*/
void Eye::background()
{
    // take image from camare 10 times to have clear
    // image of the empty field
    for(int i=0;i<10;i++){
  	cap.read(img);
    }

    // Crop area of interes 
    Rect ROI(100,250,dWidth-100,750);
    img= img(ROI);

    // Gray scale image
    cvtColor(img,img,CV_BGR2GRAY);
    
    // Threshold
    threshold(img,bg_img,100, 255, THRESH_BINARY_INV );

}

/*
    This is the service handler of Eye
    Only response to Hub and have 2 modes
*/
bool Eye::sendLocation(without::EyeData::Request  &req,
                      without::EyeData::Response &res)
{

    std::string Hub ("Hub");
    // Take command 
    uint8_t cmd=req.cmd;
    
    // Cheack if it is Hub
    if(req.ID.compare(Hub)==0){
        if (cmd==1){
            // take backgroun image mode
            background();
        	  Mat send_map= bg_img.reshape(0,1);
        	  vector<uint8_t> arrayint;
        	  arrayint.assign((uint8_t*)send_map.datastart,(uint8_t*)send_map.dataend);
        	  vector<float> array(arrayint.begin(),arrayint.end());
        	  vector<flatten2DArray> pMap;
        	  pMap.push_back(flatten2DArray());
        	  pMap.at(0).Array=array;
        	  vector<uint16_t> size;
        	  size.push_back(bg_img.cols);
        	  size.push_back(bg_img.rows);
        	  res.background.parallelMap=pMap;
        	  res.background.size=size;
        }else if (cmd==2){
            // locate robot mode
            res.robots=locate();
        }
    }
    return true;

}



int main(int argc,char* argv[])
{
  // Initialize Eye
  Eye eye;

  // Init Ros Node 
  cout<<"Ros init!!\n";
  ros::init(argc, argv, "The_Eye");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("EyeService",  &Eye::sendLocation ,&eye);
  ros::spin();
  
  return 0;
}













