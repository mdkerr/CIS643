//Samuel Fike
//Chuohao Jiang

#include "KinectSensor.h"
#include <windows.h>
#include <iostream>
using namespace std;

#define NUM_POINTS 160

/// <summary>
/// Constructor
/// </summary>
KinectSensor::KinectSensor()
{
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
	NUI_LOCKED_RECT depthData;

	//get depth data
	if(!GetAndLockDepthData(&depthData))
	{
		//something went wrong when getting depth data, so return false to show that the data is invalid
		return false;
	}

	//get the pixel at the center of the image
	NUI_DEPTH_IMAGE_PIXEL * centerPixel = ((NUI_DEPTH_IMAGE_PIXEL*) (depthData.pBits)) + 640 * 240 + 320;
	
	
	Vector4 skeletonCord = NuiTransformDepthImageToSkeleton(
		320,
		240,
		centerPixel->depth << 3,
		NUI_IMAGE_RESOLUTION_640x480
		);

	cout << "d: " << centerPixel->depth << endl;
	cout << "x: " << skeletonCord.x << endl;
	cout << "y: " << skeletonCord.y << endl;
	cout << "z: " << skeletonCord.z << endl;

	//check for obstacles
	*obstacles = centerPixel->depth < 500;

	ReleaseDepthData();

	//return true since the function completed without errors
	return true;
}

/// <summary>
/// Returns a dynamically sized array of Vector4s which represent the coordinates of the pixels
/// in the Kinect's depth image along the center horizontal line.
/// </summary>
boolean KinectSensor::GetObstacleData(Vector4 ** obstacleData, int * obstacleDataSize)
{
		NUI_LOCKED_RECT depthData;

	//get depth data
	if(!GetAndLockDepthData(&depthData))
	{
		//something went wrong when getting depth data, so return false to show that the data is invalid
		return false;
	}

	int leftBuffer = 8;
	for(int i = 0; i < NUM_POINTS; i++)
	{
		double percentWidth = i / (double) (NUM_POINTS-1);
		int totalWidth = (639 - leftBuffer);
		int imgX = (int) (percentWidth * totalWidth + leftBuffer);

		Vector4 skeletonCord = DepthToSkeletonPos( 
			imgX, 
			239, 
			&depthData 
			);

		localObstacleData[i] = skeletonCord;
	}

	*obstacleDataSize = NUM_POINTS;
	*obstacleData = localObstacleData;
	ReleaseDepthData();

	//return true since the function completed without errors
	return true;
}

Vector4 KinectSensor::DepthToSkeletonPos(int x, int y, NUI_LOCKED_RECT * depthImage)
{
	 return NuiTransformDepthImageToSkeleton(
				x,
				y,
				GetDepthAt(x, y, depthImage) << 3,
				NUI_IMAGE_RESOLUTION_640x480
				);
}

USHORT KinectSensor::GetDepthAt(int x, int y, NUI_LOCKED_RECT * depthImage)
{
	int offset = 640 * y + x;
	return (((NUI_DEPTH_IMAGE_PIXEL*) (depthImage->pBits)) + offset)->depth;
}

/// <summary>
/// Destructor
/// </summary>
KinectSensor::~KinectSensor()
{
	delete imageFrame;

    if (nuiSensor)
    {
        nuiSensor->NuiShutdown();
		nuiSensor->Release();
    }

	delete localObstacleData;
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
boolean KinectSensor::Connect(void)
{
	int numSensors = 0;
	INuiSensor * tempNuiSensor;
    HRESULT result;
    
	//check for sensor(s) and return false if none are found
	result = NuiGetSensorCount(&numSensors);
    if (FAILED(result) || numSensors == 0)
    {
        return false;
    }

    //create sensor
    result = NuiCreateSensorByIndex(0, &tempNuiSensor);
    if (FAILED(result))
    {
        return false;
    }

    //check status of sensor before initializing, release sensor if status is not OK
    result = tempNuiSensor->NuiStatus();
    if (result != S_OK)
    {
		tempNuiSensor->Release();
		return false;
    }
     
    //init kinect and set the uses_depth flag
    result = tempNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH); 
    if (!FAILED(result))
    {
        // Create an event that will be signaled when depth data is available
        //m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

        // Open a depth image stream to receive depth frames
        result = tempNuiSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_DEPTH,
            NUI_IMAGE_RESOLUTION_640x480,
            0,
            2,
            NULL,
            &depthStreamHandle);
    }

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

	//wait for a new depth frame event for 1000 ms
    //if (nuiSensor == NULL || WaitForSingleObject(m_hNextDepthFrameEvent, 1000) != WAIT_OBJECT_0)
    //{
    //    return false;
    //}

	//wait up to 1000ms for the next image frame
	HRESULT status = nuiSensor->NuiImageStreamGetNextFrame(depthStreamHandle, 10000, imageFrame);
    if (FAILED(status))
    {
        return false;
    }

	//get the frame texture from the image frame
	status = nuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthStreamHandle, imageFrame, &nearMode, &depthTextureInterface);
    if (FAILED(status))
    {
        nuiSensor->NuiImageStreamReleaseFrame(depthStreamHandle, imageFrame);
		return false;
    }

	//lock the frame texture so the kinect doesn't modify it
    depthTextureInterface->LockRect(0, &lockedRect, NULL, 0);

	//set out variable and return successful
	*lockedFrameData = lockedRect;
	return true;
}

/// <summary>
/// Unlocks and releases the depth texture interface
/// </summary>
void KinectSensor::ReleaseDepthData(void)
{
	depthTextureInterface->UnlockRect(0);
    depthTextureInterface->Release();
	nuiSensor->NuiImageStreamReleaseFrame(depthStreamHandle, imageFrame);
}