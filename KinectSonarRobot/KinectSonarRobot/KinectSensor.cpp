//Samuel Fike
//Chuohao Jiang

#include "KinectSensor.h"
#include <windows.h>
#include <iostream>
using namespace std;

/// <summary>
/// Constructor
/// </summary>
KinectSensor::KinectSensor()
{
	depthStreamHandle = INVALID_HANDLE_VALUE;
	nuiSensor = NULL;
	depthTextureInterface = NULL;
	imageFrame = new NUI_IMAGE_FRAME();
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

    //if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    //{
    //    CloseHandle(m_hNextDepthFrameEvent);
    //}
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
	
	//debug
	cout << centerPixel->depth;

	//check for obstacles
	*obstacles = centerPixel->depth < 500;

	ReleaseDepthData();

	//return true since the function completed without errors
	return true;
}

//EXPERIMENTAL
boolean KinectSensor::GetObstacleData(const ObstacleData * obstacleData, int * obstacleDataSize)
{
	NUI_LOCKED_RECT depthData;

	//try and get depth data
	if(!GetAndLockDepthData(&depthData))
	{
		return false;
	}

	cout << "-----------------------" << endl;

	//start at top left of image
	NUI_DEPTH_IMAGE_PIXEL * currentPixel = (NUI_DEPTH_IMAGE_PIXEL *) depthData.pBits;
	NUI_DEPTH_IMAGE_PIXEL * pixelBoundary = currentPixel + 640 * 480;

	//how many pixels to skip each iteration
	int pixelStep = 50;
	int offset = 0;

	Vector4 pixelPosition;
	while (currentPixel < pixelBoundary) {
		pixelPosition = NuiTransformDepthImageToSkeleton(offset % 640, offset / 640, ((NUI_DEPTH_IMAGE_PIXEL *)(currentPixel))->depth, NUI_IMAGE_RESOLUTION_640x480);
	
		//print real coordinates of close objects
		if(pixelPosition.z > 0 && pixelPosition.z < 300)
		{
			cout << pixelPosition.z << '/t' << pixelPosition.x << '/t' << pixelPosition.y << '/t' << pixelPosition.z << '/t' << pixelPosition.w << endl; //int( 100 * pixelPosition.x )/100
		}

		currentPixel += pixelStep;
	}

	/*
	//print depths of images in the plane of the camera that are:
	//	up to 6 ft away
	//	from 0-1 ft below kinect reference point
	//	within 1 ft horizontally from kinect reference point

	cout << "--------------------------\n";
	cout << endl << "Points inside limited area:" << endl << endl;

	for(int row = 0; row < 640; row += rowStep)
	{
		for(int col = 0; col < 480; col += colStep)
		{
			currentPixel = (NUI_DEPTH_IMAGE_PIXEL*) depthData.pBits + row * 640 + col;
			pixelPosition = NuiTransformDepthImageToSkeleton(row, col, currentPixel->depth, NUI_IMAGE_RESOLUTION_640x480);

			if(	pixelPosition.z < 1800	&&
				pixelPosition.y < 0		&& 
				pixelPosition.y > -300	&& 
				pixelPosition.x > -150	&&
				pixelPosition.x < 150)
			{
				cout << currentPixel->depth << '/t' << pixelPosition.x << '/t' << pixelPosition.y << '/t' << pixelPosition.z << '/t' << pixelPosition.w << endl;
			}
		}
		
	}
	*/
	cout << endl << "------------------" << endl;

	//return value doesn't mean anything, still testing function
	ReleaseDepthData();
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