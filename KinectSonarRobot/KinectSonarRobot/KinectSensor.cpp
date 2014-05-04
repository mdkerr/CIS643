//Samuel Fike
//Chuohao Jiang

#include "KinectSensor.h"
#include "testing.h"
#include <windows.h>
#include <iostream>
using namespace std;

#define NUM_POINTS 640

/// <summary>
/// Constructor
/// </summary>
KinectSensor::KinectSensor()
{
	if( TESTING ) cout << "Initializing kinect sensor" << endl;
	//setup vars needed for connection and data
	depthStreamHandle = INVALID_HANDLE_VALUE;
	nuiSensor = NULL;
	depthTextureInterface = NULL;
	imageFrame = new NUI_IMAGE_FRAME();
	localObstacleData = new Vector4[NUM_POINTS];
}

/// <summary>
/// Sets the argument to true if there are obstacles close to the Kinect, otherwise false
/// Return value is whether or not the detection was valid (false if an error is encountered)
/// </summary>
boolean KinectSensor::CenterDetect(boolean* obstacles)
{
	if( TESTING ) cout << "starting center detect" << endl;
	NUI_LOCKED_RECT depthData;

	//get depth data
	if(!GetAndLockDepthData(&depthData))
	{
		if( TESTING ) cout << "Something went wrong when getting depth data" << endl;
		//something went wrong when getting depth data, so return false to show that the data is invalid
		return false;
	}

	if( TESTING ) cout << "Getting center pixel from depth image" << endl;
	//get the pixel at the center of the image
	NUI_DEPTH_IMAGE_PIXEL * centerPixel = ((NUI_DEPTH_IMAGE_PIXEL*) (depthData.pBits)) + 640 * 240 + 320;
	
	//convert pixel to real-world coordinates
	Vector4 skeletonCord = NuiTransformDepthImageToSkeleton(
		320,
		240,
		centerPixel->depth << 3,
		NUI_IMAGE_RESOLUTION_640x480
		);

	//check for obstacles
	*obstacles = centerPixel->depth < 500;

	//release depth data from lock
	ReleaseDepthData();

	//return true since the function completed without errors
	if( TESTING ) cout << "Kinect center detect finished" << endl;
	return true;
}

/// <summary>
/// Returns a dynamically sized array of Vector4s which represent the coordinates of the pixels
/// in the Kinect's depth image along the center horizontal line.
/// </summary>
boolean KinectSensor::GetObstacleData(Vector4 ** obstacleData, int * obstacleDataSize)
{
	if( TESTING ) cout << "Starting GetObstacleData" << endl;
	NUI_LOCKED_RECT depthData;

	//get depth data
	if(!GetAndLockDepthData(&depthData))
	{
		//something went wrong when getting depth data, so return false to show that the data is invalid
		if( TESTING ) cout << "Something went wrong when getting depth data" << endl;
		return false;
	}

	//get real-world coordinates for a configurable amount of points along the horizon line
	int leftBuffer = 8;
	for(int i = 0; i < NUM_POINTS; i++)
	{
		if( TESTING ) cout << "Analyzing point on horizon line" << endl;
		
		//prepare arguments for conversion function
		double percentWidth = i / (double) (NUM_POINTS-1);
		int totalWidth = (639 - leftBuffer);
		int imgX = (int) (percentWidth * totalWidth + leftBuffer);

		//convert to skeleton coordinates
		Vector4 skeletonCord = DepthToSkeletonPos( 
			imgX, 
			239, 
			&depthData 
			);

		//store in array
		localObstacleData[i] = skeletonCord;
	}

	if( TESTING ) cout << "Finished analyzing kinect image horizon line" << endl;

	//set out variable
	*obstacleDataSize = NUM_POINTS;
	*obstacleData = localObstacleData;

	//release locked depth data
	ReleaseDepthData();

	//return true since the function completed without errors
	return true;
}

//helper function for converting depth to real-world position
Vector4 KinectSensor::DepthToSkeletonPos(int x, int y, NUI_LOCKED_RECT * depthImage)
{
	if( TESTING ) cout << "Started DepthToSkeletonPos" << endl;
	 return NuiTransformDepthImageToSkeleton(
				x,
				y,
				GetDepthAt(x, y, depthImage) << 3,
				NUI_IMAGE_RESOLUTION_640x480
				);
}

//helper function for getting depth at a point on the image
USHORT KinectSensor::GetDepthAt(int x, int y, NUI_LOCKED_RECT * depthImage)
{
	if( TESTING ) cout << "Started GetDepthAt" << endl;
	int offset = 640 * y + x;
	return (((NUI_DEPTH_IMAGE_PIXEL*) (depthImage->pBits)) + offset)->depth;
}

/// <summary>
/// Destructor
/// </summary>
KinectSensor::~KinectSensor()
{
	if( TESTING ) cout << "Deconstructing KinectSensor" << endl;

	//free resources used for imageFrame
	delete imageFrame;

	//shutdown and release nuiSensor if it exists
    if (nuiSensor)
    {
		//shutdown and release nuiSensor
		if( TESTING ) cout << "Shutting down and releasing nuiSensor" << endl;
        nuiSensor->NuiShutdown();
		nuiSensor->Release();
    }

	//free resources used for imageFrame
	delete localObstacleData;
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
boolean KinectSensor::Connect(void)
{
	if( TESTING ) cout << "Connecting to Kinect" << endl;
	int numSensors = 0;
	INuiSensor * tempNuiSensor;
    HRESULT result;
    
	//check for sensor(s) and return false if none are found
	result = NuiGetSensorCount(&numSensors);
    if (FAILED(result) || numSensors == 0)
    {
		if( TESTING ) cout << "No kinects found" << endl;
        return false;
    }

    //create sensor
    result = NuiCreateSensorByIndex(0, &tempNuiSensor);
    if (FAILED(result))
    {
		if( TESTING ) cout << "Something went wrong creating nui sensor" << endl;
        return false;
    }

    //check status of sensor before initializing, release sensor if status is not OK
    result = tempNuiSensor->NuiStatus();
    if (result != S_OK)
    {
		if( TESTING ) cout << "Status of nui sensor was not OK" << endl;
		tempNuiSensor->Release();
		return false;
    }
     
    //init kinect and set the uses_depth flag
    result = tempNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH); 
    if (!FAILED(result))
    {
		if( TESTING ) cout << "Open stream from Kinect" << endl;

        // Open a depth image stream to receive depth frames
        result = tempNuiSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_DEPTH,
            NUI_IMAGE_RESOLUTION_640x480,
            0,
            2,
            NULL,
            &depthStreamHandle);
    }

	if( TESTING ) cout << "Finished connecting to Kinect" << endl;
	//since initialization completed successfully, we will use this sensor and return true
	nuiSensor = tempNuiSensor;
    return true;
}

/// <summary>
/// Sets the argument to true if there are obstacles close to the Kinect, otherwise false
/// Return value is whether or not the detection was valid (false if an error is encountered)
/// </summary>
boolean KinectSensor::GetAndLockDepthData(NUI_LOCKED_RECT * lockedFrameData)
{
	BOOL nearMode = false;
	NUI_LOCKED_RECT lockedRect;

	//wait up to 1000ms for the next image frame
	HRESULT status = nuiSensor->NuiImageStreamGetNextFrame(depthStreamHandle, 10000, imageFrame);
    if (FAILED(status))
    {
		if( TESTING ) cout << "Something went wrong when getting kinect image" << endl;
        return false;
    }

	//get the frame texture from the image frame
	status = nuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthStreamHandle, imageFrame, &nearMode, &depthTextureInterface);
    if (FAILED(status))
    {
		if( TESTING ) cout << "Something went wrong when getting depth data" << endl;
        nuiSensor->NuiImageStreamReleaseFrame(depthStreamHandle, imageFrame);
		return false;
    }

	//lock the frame texture so the kinect doesn't modify it
    depthTextureInterface->LockRect(0, &lockedRect, NULL, 0);

	if( TESTING ) cout << "Finished getting and locking kinect depth data" << endl;
	//set out variable and return successful
	*lockedFrameData = lockedRect;
	return true;
}

/// <summary>
/// Unlocks and releases the depth texture interface
/// </summary>
void KinectSensor::ReleaseDepthData(void)
{
	if( TESTING ) cout << "Releasing depth data" << endl;
	//unlock and release depth data and frame
	depthTextureInterface->UnlockRect(0);
    depthTextureInterface->Release();
	nuiSensor->NuiImageStreamReleaseFrame(depthStreamHandle, imageFrame);
}