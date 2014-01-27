//Samuel Fike
//Chuohao Jiang

#pragma once
#include <Windows.h>
#include <tuple>
#include "NuiApi.h"

/// <summary>
/// Represents a single obstacle detected by the Kinect
/// </summary>
struct ObstacleData {
	float depth;
	float leftSide;
	float rightSide;
};

class KinectSensor
{

public:

    /// <summary>
    /// Constructor
    /// </summary>
    KinectSensor();

    /// <summary>
    /// Destructor
    /// </summary>
    ~KinectSensor();

	/// <summary>
    /// Attempt to connect to a Kinect
    /// </summary>
	boolean Connect(void);

	/// <summary>
    /// Detect obstacle 500mm in front of the Kinect
    /// </summary>
	boolean CenterDetect(boolean *);

	/// <summary>
    /// *EXPERIMENTAL* Gets positions of potential obstacles from the Kinect
	/// Returns a pointer to an array
    /// </summary>
	boolean GetObstacleData(const ObstacleData *, int *);

private:
    INuiSensor *		nuiSensor;
	INuiFrameTexture *	depthTextureInterface;
    HANDLE				depthStreamHandle;
	NUI_IMAGE_FRAME	*	imageFrame;
    //HANDLE				m_hNextDepthFrameEvent;

	boolean				GetAndLockDepthData(NUI_LOCKED_RECT *);
	void				ReleaseDepthData(void); 
};
