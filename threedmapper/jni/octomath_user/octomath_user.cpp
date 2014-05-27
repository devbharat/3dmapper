/*
 * Author: Bharat Tak
 */

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

/* Silly function to convert int to string */
string convertInt(int number)
{
   stringstream ss;
   ss << number;
   return ss.str();
}

/* Parses lines from VICON datafile and assigns camera Pose.
 * 
 * data_line := 'line' from the VICON datafile
 * frame := image frame associated with the Pose data. Must match with image number.
 * translation := is assigned the position (x,y,z) of the camera. Output.
 * r_vec := is assigned the attitude of the camera (roll pitch yaw). Output. 
 * 
 * Assumed input format of vicon datafile (as in dataset3)
 * frame time viconx vicony viconz viconroll viconpitch viconyaw ... \n
 *
 * return 'true' if data successfully parsed and assigned, else false.
 */
bool camOrientation(string & data_line, int & frame,Point3f & translation,Point3f & r_vec){
	stringstream lineStream(data_line);
	string cell;
	vector<float> data;
	while(getline(lineStream,cell,','))
	    {
		data.push_back(::atof(cell.c_str()));
	    }

	//hack to skip the first few lines of VICON datafile that are all zeros
	if(data[2] + data[3] + data[4] + data[5] + data[6] + data[7] == 0){
	cout <<"No position: Vicon not initialized"<<endl;
	return false;
	}else{
		frame = data[0];
		cout << frame<<endl;
		translation.x = data[2];
		translation.y = data[3];
		translation.z = data[4];

		r_vec.x = data[5];
		r_vec.y = data[6];
		r_vec.z = data[7];
		return true;
	}
}

/* Function to skip a range of images for whome either pose data is corrupt
 * or the disparity data is corrupt.
 * 
 * ctr := Its the image number being considered to process or skip
 * skipstart, skipend := define the range of images to be skipped 
 *
 * returns 'true' when crt is within the skip range given by skipstart and skipend
 * else 'false'
 */
bool skipframe(int ctr, int skipstart,int skipend){
	if(ctr >= skipstart && ctr <= skipend){return false;}
	else {return true;}
}


/* The main Function that computes and writes the map file from a sequence of numbered images.
 * startImage := Image number to start processing with (first image in the map)
 * endImage := The final image number to be included in the map
 * imagePath := The absolute path of the folder in the android filesystem containing images (eg. "sdcard/Notes/dataset/dataset3/")
 * 
 * returns an array containing number of images processed and runtime in microseconds
 */
JNIEXPORT jintArray Java_eth_threedphoto_threedmapper_Native_map
    (JNIEnv* env, jobject o, jint startImage, jint endImage, const jstring imagePath) {

  int start_s=clock();

  char const * image_path = env->GetStringUTFChars(imagePath , NULL ) ;
  
  ScanGraph* graph = new ScanGraph();
  ScanNode* currentNode = NULL;

  ifstream viconFile;
  
  string imageName,imageExt,pathToViconFile,viconFileName;

  string pathToImage(image_path); 
  imageExt = ".png";

  pathToViconFile = "/sdcard/Notes/dataset/dataset3/";
  viconFileName = pathToViconFile + "log.csv";
  viconFile.open(viconFileName.c_str());


  int i = 1;         //image counter (in numbered Images)
  int frame=0;       //frame counter (in vicon datafile)

  int start_image, end_image;
  start_image = startImage;
  end_image = endImage;

  Point3f cam_trans, r_vec;
  Mat image;

  //disparity to depth matix
  Mat Q = (Mat_<double>(4,4) << 1, 0, 0, -Uo, 0, 1, 0, -Vo,0,0,0,FOCAL_LENGTH, 0,0,1/abs(CAM_DIST_X),0);

  string camLine;
  getline(viconFile,camLine);//see off the first field line (not containing data)

    while(i<= end_image){
	    if(getline(viconFile,camLine)){
		    if(camOrientation(camLine,frame,cam_trans,r_vec) && frame==i && i>=start_image && skipframe(i,218,250)){
			    imageName = pathToImage + convertInt(i) + imageExt;
			    image = imread(imageName,0); 
			    if(! image.data )
			    {
				cout <<  "Could not open or find the image " <<imageName<< std::endl ;
			    }else{

				    Mat cropedImage = image(Rect(Point2f(40,40),Point2f(600,440)));//Crop image for Region of Interest (top left to bottom right)

				    //Include previous points in the graph 
				    if (currentNode){
            				graph->nodes.push_back(currentNode);
            				graph->connectPrevious();
					cout<<"ScanNode "<< currentNode->pose << " done, size: "<< currentNode->scan->size()<<endl;
				    }
				    
				    //Prepare octomap scangraph for new set of datapoints
				    currentNode = new ScanNode();
				    currentNode->scan = new Pointcloud();

				    //Ordering and signs important! 
                                    //Represent pre rotation of vicon frame to the octomap input coordinate frame. 
                                    //Change at your own risk!
				    pose6d pose(-cam_trans.x, cam_trans.z, cam_trans.y, r_vec.y, r_vec.z,r_vec.x);
				    currentNode->pose = pose;

                                    /* Compact function for the for loop below. 
                                     *
                                     * Mat _3dimage = cropedImage;
                                     * reprojectImageTo3D(cropedImage, _3dimage, Q,0,-1);
                                     */


				    uchar *gt;
				    int rows = cropedImage.rows;
				    int cols = cropedImage.cols;
				    Point3f cloc, gloc;
				    int intensity;
				    for(int l = 0;l < rows;++l){
					    gt = cropedImage.ptr(l);
					    for(int k = 0;k < cols;++k){
						*gt++;
						intensity = *gt/8; // Given intensity prescaling in images.

                                                //Ignore points detected too far, mostly junk!
						if(intensity > 8){
							
							cloc.z = FOCAL_LENGTH*abs(CAM_DIST_X)/intensity;
							cloc.x = (k - Uo)*cloc.z/FOCAL_LENGTH;
							cloc.y = (l - Vo)*cloc.z/FOCAL_LENGTH;

							//Insert 3D point in the scangraph 
							currentNode->scan->push_back(cloc.x,cloc.y,cloc.z);
						}
					    }
				    }				
			    }			
		}
		i++;	
	    }else{
		 cout <<"Getline doesnt work: No more data in vicon file ?"<<endl;
		 break;			
	    }
    }

  //Final itiration of the scangraph added to the octomap graph object. Input for mapping function.
  graph->nodes.push_back(currentNode);
  graph->connectPrevious();
  cout<<"Final ScanNode "<< currentNode->pose << " done, size: "<< currentNode->scan->size()<<endl;


  //Incase you need to write the scangraph file. Slows the code down.
  //Output graphfile name and folder
  //string graphFilename = "/sdcard/Notes/mapout.graph";
  //graph->writeBinary(graphFilename);



  //desided map resolution
  double res = 0.1;

  //tree file name and folder
  string treeFilename = "/sdcard/Notes/mapout.bt";


  //read the previous occupancy tree that needs to be modified with new data
  OcTree* readtree = new OcTree(treeFilename);


  //set default tree parameters
  double maxrange = -1;
  int max_scan_no = -1;
  bool detailedLog = false;
  bool simpleUpdate = false;
  bool discretize = true;
  unsigned char compression = 1;

  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {

    pose6d frame_origin = (*scan_it)->pose;
    point3d sensor_origin = frame_origin.inv().transform((*scan_it)->pose.trans());

    (*scan_it)->scan->transform(frame_origin);
    point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
    (*scan_it)->pose = pose6d(transformed_sensor_origin, octomath::Quaternion());

  }

  /*
  OcTree* tree = new OcTree(res);

  tree->setClampingThresMin(clampingMin);
  tree->setClampingThresMax(clampingMax);
  tree->setProbHit(probHit);
  tree->setProbMiss(probMiss);*/
  readtree->setOccupancyThres(0.75);
  readtree->setResolution(0.05);

  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;

  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
    if (max_scan_no > 0) cout << "("<<currentScan << "/" << max_scan_no << ") " << flush;
    else cout << "("<<currentScan << "/" << numScans << ") " << flush;

    if (simpleUpdate)
      readtree->insertPointCloudRays((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange);
    else
      readtree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);

    if (compression == 2){
      readtree->toMaxLikelihood();
      readtree->prune();
    }

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;
  }

  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;

  readtree->toMaxLikelihood();
  readtree->prune();
  readtree->writeBinary(treeFilename);

  delete readtree;


  int stop_s=clock();
  int  t = (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000;

  jintArray result;
  result = env->NewIntArray(2);
  if (result == NULL) {
    return NULL; /* out of memory error thrown */
  }
 
  jint fill[2]; 
  fill[0] = t;
  fill[1] = end_image - start_image + 1;

  env->SetIntArrayRegion(result, 0, 2, fill);
  return result;

  }
}
