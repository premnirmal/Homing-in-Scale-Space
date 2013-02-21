/* PREM NIRMAL
   Fordham RCV Lab
   Fordham University
   Bronx NY 10458
*/
/*
  07/2012
  An implemtation of Churchill and Vardy's VHiSS algorithm,
  using David Lowe's SIFT feature extractor.
*/

#include <cstdio>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "Aria.h"

#include "defs.h"

using namespace cv;
using namespace std;
using namespace Eigen;

ArDPPTU *myPTZ=0;
ArRobot *myRobot=0;

#define HOMEIMAGE "data/home.pgm"
#define HOMEKEY "data/home.key"
#define IMG_WIDTH 1024
#define IMG_HEIGHT 768
#define PANAMT 0
#define DEFTILT 5
#define DISTANCE 600
#define EPS 2

/* -------------------- Local function prototypes ------------------------ */

Vector2d FindMatches(Image im1, Keypoint keys1, Image im2, Keypoint keys2, int imageCount);
Keypoint CheckForMatch(Keypoint key, Keypoint klist);
int DistSquared(Keypoint k1, Keypoint k2);
Image CombineImagesHorizontally(Image im1, Image im2);
/*----------------------------- Routines ----------------------------------*/
int main (int argc, char **argv)
{
  Image im1 = NULL, im2 = NULL;
  Keypoint k1 = NULL, k2 = NULL;
  int imageCount=1;
  char imageName[100], keypointName[100], command[256];
  double alpha;
  Vector2d move;

  /* ROBOT DECLARATIONS */
  ArArgumentParser parser(&argc, argv); // set up our parser
  ArSimpleConnector simpleConnector(&parser); // set up our simple connector
  ArRobot robot; // declare robot object
  //  ArSonarDevice sonar;
  
  /* INITIALIZATION OF CONNECTION TO ROBOT */
  Aria::init(); // mandatory init
  parser.loadDefaultArguments(); // load the default arguments 
  if (!simpleConnector.parseArgs() // check command line args
      || !parser.checkHelpAndWarnUnparsed(1))
    {    
      simpleConnector.logOptions();
      Aria::shutdown();
      return 1;
    }
  if (!simpleConnector.connectRobot(&robot)) // ask for connection to robot
    {
      printf("Could not connect to robot... exiting\n");
      Aria::shutdown();
      return 1;
    }

  ArDPPTU thePTZ(&robot); // pan-tilt base, mode DPPU
  myPTZ = &thePTZ;
  myPTZ->reset(); 
  ArUtil::sleep(1000);

  /* INITIALIZATION OF ROBOT*/
  robot.runAsync(true); // commands processed in separate thread
  robot.enableMotors(); // turn the power to the motors on
  //  robot.addRangeDevice(&sonar); // add sonar (THIS IS UNNECCESARY FOR VH)
  ArUtil::sleep(1000); // sleep time allows robot to initialise sonar, motors, etc

  if (myPTZ==0) {
    printf("PTZ unit not initialized\n");
    return 1;
  }

  robot.setRotVelMax(30);
  robot.setTransVelMax(80);

  /*** CAPTURE HOME IMAGE ***/
  // cout<<endl<<"Align robot towards goal location, to capture home image.."<<endl
  //     <<"Then press ENTER.";
  // cin.get();
  // myPTZ->panTilt(PANAMT,DEFTILT);
  // ArUtil::sleep(1000);
  // Mat homeframe;
  // IplImage *homeImage, *homeGray;
  // VideoCapture cap(0);
  // if(!cap.isOpened())  // check whether camera has been found or not
  //   {
  //     cerr << "Error: Cannot open image\n";
  //     return -1;
  //   }
  // /* capture webcam into frame */
  // cout<<"Capturing home image.."<<endl;
  // cap >> homeframe;
  // ArUtil::sleep(1000);
  // /* capture webcam and store it as temphome.jpg */
  // imwrite("data/temphome.jpg",homeframe);
  // homeImage=cvLoadImage("data/temphome.jpg",1);
  // homeGray=cvCreateImage(cvGetSize(homeImage),IPL_DEPTH_8U,1);
  // cvCvtColor(homeImage,homeGray,CV_RGB2GRAY);
  // cvSaveImage("data/home.pgm",homeGray);
  // remove("data/temphome.jpg");

  // myPTZ->reset(); 
  
  // /*** OBTAIN SIFT FEATURES ***/
  // system("./sift <data/home.pgm > data/home.key");
  // /*** -------------------- ***/

  // cap.release();
  
  /******************************/


  cout<<endl<<"Now position robot away from goal location, "
      <<"and press ENTER to home";
  cin.get();

  Mat frame;
  IplImage *image,*gray;
  char buf[100];

  //  bool homingComplete=false;

  do
    {  
      robot.stop();
      myPTZ->panTilt(PANAMT,DEFTILT);
      ArUtil::sleep(1000);
      /*** OPENCV VIDEO CAPTURE ***/
      VideoCapture cap(0); // open the default camera, store in cap
      if(!cap.isOpened())  // check whether camera has been found or not
	{
	  cerr << "Error: Cannot open image\n";
	  return -1;
	}
      cout<<"Step #"<<imageCount<<endl;
      /* capture webcam into frame */
      cout<<"Capturing image.."<<endl;
      cap >> frame;
      sprintf(buf,"data/temp.jpg");
      /* capture webcam and store it as temp.jpg */
      imwrite(buf,frame);
      /* load temp.jpg into img */
      image=cvLoadImage(buf,1);
      ArUtil::sleep(1000);
      gray=cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);
      cvCvtColor(image,gray,CV_RGB2GRAY);
      sprintf(imageName,"data/image%d.pgm",imageCount);
      cvSaveImage(imageName,gray);
      sprintf(keypointName,"data/image%d.key",imageCount);
      /*** END OPENCV VIDEO CAPTURE ***/

      myPTZ->reset(); 

      /*** OBTAIN SIFT FEATURES ***/
      snprintf(command,256,"./sift <%s >%s",imageName,keypointName);
      system(command);
      /*** -------------------- ***/

      remove("data/temp.jpg");

      cap.release();

      im1=ReadPGMFile(HOMEIMAGE);
      k1=ReadKeyFile(HOMEKEY);
      im2=ReadPGMFile(imageName);
      k2=ReadKeyFile(keypointName);
  
      /* determine movement vector using SIFT features */
      move=FindMatches(im1, k1, im2, k2,imageCount);
      alpha=-move(0);
      alpha*=180/(M_PI);      

      if(alpha!=alpha)
	{
	  cout<<"Alpha = "<<alpha<<endl;
	  cout<<"Unable to complete homing. Exiting.."<<endl;
	  return -1;
	}
      else// if(fabs(alpha)>=EPS && alpha==alpha)
	{
	  cout<<"Robot orienting by "<<alpha<<"degrees. ";
	  if(move(1)>0)
	    cout<<"and moving forwards"<<endl;
	  else
	    cout<<"and moving backwards"<<endl;

	  ArUtil::sleep(1000);
	  robot.setDeltaHeading(alpha); // orient robot towards goal
	  ArUtil::sleep(2000);
	  robot.move(DISTANCE*move(1)); // move forwards or backwards depending on move(1)
	  ArUtil::sleep(4000+(DISTANCE*10));
	}
      imageCount++;
    }
  while(true);

  return 0;
}



/* Given a pair of images and their keypoints, pick the first keypoint
   from one image and find its closest match in the second set of
   keypoints.
*/
Vector2d FindMatches(Image im1, Keypoint keys1, Image im2, Keypoint keys2, int imageCount)
{
  Keypoint k,match;
  int count=0;
  double alpha=0,delta=0;
  //  vector< Vector2d,aligned_allocator<Vector2d> > unitVec;
  Image result;
  result=CombineImagesHorizontally(im1,im2);

  vector<Keypoint> matchedK1, matchedK2;
  vector<Keypoint> mPOS1, mPOS2, mNEG1, mNEG2;
  vector<double> thetaPOS, thetaNEG;
  Vector2d move;

  /* Match the keys in list keys1 to their best matches in keys2 */
  for(k=keys1;k!=NULL;k=k->next)
    {
      match=CheckForMatch(k,keys2);  
      if(match!=NULL) 
	{
	  delta = k->scale - match->scale;

	  DrawLine(result, (int) k->row, (int) k->col,
		   (int) (match->row),(int) match->col + im1->cols);

	  // STORE MATCHED KEYPOINT VECTOR
	  matchedK1.push_back(k);
	  matchedK2.push_back(match);

	  // STORE mPOS and mNEG
	  if(delta>=0)
	    {
	      mPOS1.push_back(k);
	      mPOS2.push_back(match);
	      // 70 degrees field of view
	      thetaPOS.push_back((k->col*(70/IMG_WIDTH)*(M_PI/180)) - (35*M_PI/180));
	    }
	  if(delta<0)
	    {
	      mNEG1.push_back(k);
	      mNEG2.push_back(match);
	      // 70 degrees field of view
	      thetaNEG.push_back((k->col*(70/IMG_WIDTH)*(M_PI/180)) - (35*M_PI/180));
	    }
	  count++;
	}//end if(match..
    }//end for(k=keys1..

  FILE *matched;
  char matchfileName[100];
  sprintf(matchfileName,"data/matched%d.pgm",imageCount);
  matched=fopen(matchfileName,"w");
  WritePGM(matched, result);
  free_img(result);
  fclose(matched);
  cout<<"Wrote '"<<matchfileName<<"'.\n";

  if(count==0)
    {
      cout<<"No matches found. Exiting..\n";
      abort();
    }
  
  fprintf(stderr,"Found a total of %d matches.\n", count);

  double POSthetaAverage, NEGthetaAverage;
  vector<double>::iterator pit,nit;
  double sintheta=0, costheta=0;
  for(pit=thetaPOS.begin();pit!=thetaPOS.end();pit++)
    {
      sintheta+=sin(*pit);
      costheta+=cos(*pit);
    }
  if(sintheta!=0 && costheta!=0)
    POSthetaAverage=atan(sintheta/costheta);
  else
    POSthetaAverage=0;

  sintheta=0; costheta=0;

  for(nit=thetaNEG.begin();nit!=thetaNEG.end();nit++)
    {
      sintheta+=sin(*nit);
      costheta+=cos(*nit);
    }
  if(sintheta!=0 && costheta!=0)
    NEGthetaAverage=atan(sintheta/costheta);
  else
    NEGthetaAverage=0;

  sintheta=0; costheta=0;

  double s,c;

  s=thetaPOS.size()*sin(POSthetaAverage) + thetaNEG.size()*(sin(NEGthetaAverage)+M_PI);
  c=thetaPOS.size()*cos(POSthetaAverage) + thetaNEG.size()*(cos(NEGthetaAverage)+M_PI);

  cout<<"thetaPOS.size(): "<<thetaPOS.size()<<" thetaNEG.size(): "<<thetaNEG.size()<<endl;

  double sign=0;
  if(fabs(atan2(s,c))>=(M_PI/2))
    {
      sign=-1;
      alpha=-1;
    }
  else
    {
      sign=1;
      alpha=1;
    }

  /** CLEAR MEMORY **/
  matchedK1.clear(); matchedK2.clear();
  mPOS1.clear(); mPOS2.clear();
  mNEG1.clear(); mNEG2.clear();
  thetaPOS.clear(); thetaNEG.clear();

  alpha*=atan2(s,c);

  move << alpha,sign;
  return move;

}//end void FindMatches..


/* This searches through the keypoints in klist for the two closest
   matches to key.  If the closest is less than 0.6 times distance to
   second closest, then return the closest match.  Otherwise, return
   NULL.
*/
Keypoint CheckForMatch(Keypoint key, Keypoint klist)
{
  int dsq, distsq1 = 100000000, distsq2 = 100000000;
  Keypoint k, minkey = NULL;

  /* Find the two closest matches, and put their squared distances in
     distsq1 and distsq2.
  */
  for (k = klist; k != NULL; k = k->next) {
    dsq = DistSquared(key, k);

    if (dsq < distsq1) {
      distsq2 = distsq1;
      distsq1 = dsq;
      minkey = k;
    } else if (dsq < distsq2) {
      distsq2 = dsq;
    }
  }

  /* Check whether closest distance is less than 0.6 of second. */
  if (10 * 10 * distsq1 < 6 * 6 * distsq2)
    return minkey;
  else return NULL;
}


/* Return squared distance between two keypoint descriptors.
 */
int DistSquared(Keypoint k1, Keypoint k2)
{
  int i, dif, distsq = 0;
  unsigned char *pk1, *pk2;

  pk1 = k1->descrip;
  pk2 = k2->descrip;

  for (i = 0; i < 128; i++) {
    dif = (int) *pk1++ - (int) *pk2++;
    distsq += dif * dif;
  }
  return distsq;
}

Image CombineImagesHorizontally(Image im1, Image im2)
{
  int rows, cols, r, c;
  Image result;
  rows = MAX(im1->rows,im2->rows);
  cols = im1->cols+im2->cols;
  result = CreateImage(rows,cols);
  /* Set all pixels to 0,5, so that blank regions are grey. */
  for (r = 0; r < rows; r++)
    for (c = 0; c < cols; c++)
      result->pixels[r][c] = 0.5;
  /* Copy images into result. */
  for (r = 0; r < im1->rows; r++)
    for (c = 0; c < im1->cols; c++)
      result->pixels[r][c] = im1->pixels[r][c];
  for (r = 0; r < im2->rows; r++)
    for (c = 0; c < im2->cols; c++)
      result->pixels[r][c+im1->cols] = im2->pixels[r][c];

  return result;
}

