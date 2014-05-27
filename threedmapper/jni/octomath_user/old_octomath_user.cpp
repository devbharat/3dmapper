#include <jni.h>

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <octomap.h>
#include <octomap_timing.h>

#define PIXEL_LENGTH 0.000006
#define FOCAL_LENGTH 611.739321
#define CAM_DIST_X -0.079115608 
#define CAM_DIST_Y -0.000558381
#define CAM_DIST_Z  0.000922792
#define Uo 343.228336
#define Vo 244.798311

using namespace std;
using namespace cv;
using namespace octomap;

extern "C" {
string convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

bool camOrientation(string & data_line, int & frame,Point3f & translation,Point3f & r_vec){
	stringstream lineStream(data_line);
	string cell;
	vector<float> data;
	while(getline(lineStream,cell,','))
	    {
		data.push_back(::atof(cell.c_str()));
	    }
	//cout << data[3] << " " << data[4]<<" "<< data[5]<<" "<< data[6]<< endl;
	if(data[2] + data[3] + data[4] + data[5] + data[6] + data[7] == 0){
	cout <<"No position: Vicon not initialized"<<endl;
	return false;
	}else{
		frame = data[0];
		cout << frame<<endl;
		translation.x = data[2];
		translation.y = data[3];
		translation.z = data[4];

		r_vec.x = data[5];//data[5]
		r_vec.y = data[6];//data[6]
		r_vec.z = data[7];//data[7]

		return true;
	}
	
}

bool skipframe(int ctr, int skipstart,int skipend){
	if(ctr >= skipstart && ctr <= skipend){return false;}
	else {return true;}
}

jlong Java_pl_morgwai_ndktutorial_Native_fun
    (JNIEnv* env, jobject o, jint ii) {
  int start_s=clock();

  ifstream viconFile;
  ofstream myfile;
  
  string imageName,pathToImage,imageExt,pathToPointFolder,pointFileName, pathToViconFile, viconFileName;
  pathToImage = "/sdcard/Notes/Images/";
  imageExt = ".png";
  pathToPointFolder  = "/sdcard/Notes/";
  pathToViconFile = "/sdcard/Notes/";

  viconFileName = pathToViconFile + "log.csv";
  viconFile.open(viconFileName.c_str());

  pointFileName = pathToPointFolder + "loc" + ".txt";
  myfile.open (pointFileName.c_str());

  int frame=0;
  int start_image, end_image;
  start_image = 280;
  end_image = 280;

  int i = 1;
  Point3f cam_trans, r_vec;
  Mat image;

  Mat Q = (Mat_<double>(4,4) << 1, 0, 0, -Uo, 0, 1, 0, -Vo,0,0,0,FOCAL_LENGTH, 0,0,1/abs(CAM_DIST_X),0);

  string camLine;
  getline(viconFile,camLine);//see off the first field line

    while(i<= end_image){
	    if(getline(viconFile,camLine)){
		    
		    if(camOrientation(camLine,frame,cam_trans,r_vec) && frame==i && i>=start_image && skipframe(i,218,250)){		

			    imageName = pathToImage + convertInt(i) + imageExt;
			    image = imread(imageName,0); // Read the file, greyscale

			    if(! image.data )                      // Check for invalid input
			    {
				cout <<  "Could not open or find the image " <<imageName<< std::endl ;
			    }else{

				    Mat cropedImage = image(Rect(Point2f(40,40),Point2f(600,440)));//Mat cropedImage = image(Rect(Point2f(40,40),Point2f(600,440)));

				    myfile <<"NODE "<<-cam_trans.x<< " "<<cam_trans.z<< " "<<cam_trans.y<< " "<<r_vec.y<< " "<<r_vec.z<< " "<<r_vec.x<<"\n";//ordering important! transxzy rotyzx
				    //Mat _3dimage = cropedImage;
				    //reprojectImageTo3D(cropedImage, _3dimage, Q,0,-1); //replacement for the for loop below. Does not save any more time than cascaded for loop. 
				    //myfile <<_3dimage;

				    uchar *gt;
				    int rows =cropedImage.rows;
				    int cols =cropedImage.cols;
				    Point3f cloc, gloc;
				    int intensity;
				    for(int l = 0;l < rows;++l){
					    gt = cropedImage.ptr(l);
					    for(int k = 0;k < cols;++k){
						*gt++;
						intensity = *gt/8;
						//cout <<intensity[0]<<endl;

						if(intensity > 8){//ignore points detected too far, mostly junk!
							
							cloc.z = FOCAL_LENGTH*abs(CAM_DIST_X)/intensity;
							cloc.x = (k - Uo)*cloc.z/FOCAL_LENGTH;
							cloc.y = (l - Vo)*cloc.z/FOCAL_LENGTH;
							myfile <<cloc.x<< " "<< cloc.y<< " "<< cloc.z<< "\n";
		
						}
					    }
				    
				    }
				    	    				
			    }			
		}//camorientation_works, frame==i, i==start_image
		i++;	
		

	    }else{
		 cout <<"Getline doesnt work: No more data in vicon file ?"<<endl;
		 break;			
	    }//getline doesnot work
    }//while
  myfile.close ();











//Here starts octomap part
  string logFilename = "/sdcard/Notes/loc.txt";
  string graphFilename = "/sdcard/Notes/mapout.graph";
  
  ScanGraph* graph = new ScanGraph();
  graph->readPlainASCII(logFilename);

  int stop_s=clock();
  
  //graph->writeBinary(graphFilename);

  double res = 0.1;
  string treeFilename = "/sdcard/Notes/mapout.bt";
  double maxrange = -1;
  int max_scan_no = -1;
  bool detailedLog = false;
  bool simpleUpdate = false;
  bool discretize = true;
  unsigned char compression = 1;

  OcTree emptyTree(res);
  double clampingMin = emptyTree.getClampingThresMin();
  double clampingMax = emptyTree.getClampingThresMax();
  double probMiss = emptyTree.getProbMiss();
  double probHit = emptyTree.getProbHit();

  timeval start; 
  timeval stop; 

  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {

    pose6d frame_origin = (*scan_it)->pose;
    point3d sensor_origin = frame_origin.inv().transform((*scan_it)->pose.trans());

    (*scan_it)->scan->transform(frame_origin);
    point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
    (*scan_it)->pose = pose6d(transformed_sensor_origin, octomath::Quaternion());

  }

  OcTree* tree = new OcTree(res);

  tree->setClampingThresMin(clampingMin);
  tree->setClampingThresMax(clampingMax);
  tree->setProbHit(probHit);
  tree->setProbMiss(probMiss);

  gettimeofday(&start, NULL);  // start timer
  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;

  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
    if (max_scan_no > 0) cout << "("<<currentScan << "/" << max_scan_no << ") " << flush;
    else cout << "("<<currentScan << "/" << numScans << ") " << flush;

    if (simpleUpdate)
      tree->insertPointCloudRays((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange);
    else
      tree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);

    if (compression == 2){
      tree->toMaxLikelihood();
      tree->prune();
    }

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;
  }

  double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);

  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;

  tree->toMaxLikelihood();
  tree->prune();
  tree->writeBinary(treeFilename);

  gettimeofday(&stop, NULL);  // stop timer

  delete tree;


  
  long long t = (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000;
  return t;

}
}
