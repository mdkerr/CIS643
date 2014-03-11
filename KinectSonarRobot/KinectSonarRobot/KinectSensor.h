//Samuel Fike
//Chuohao Jiang

#pragma once
#include <Windows.h>
#include <tuple>
#include "NuiApi.h"

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
	boolean GetObstacleData(Vector4 **, int *);

private:
    INuiSensor *			nuiSensor;
	INuiFrameTexture *		depthTextureInterface;
    HANDLE					depthStreamHandle;
	NUI_IMAGE_FRAME	*		imageFrame;
	Vector4 *				localObstacleData;

	boolean					GetAndLockDepthData(NUI_LOCKED_RECT *);
	void					ReleaseDepthData(void); 
	Vector4					DepthToSkeletonPos(int, int, NUI_LOCKED_RECT *);
	USHORT					GetDepthAt(int, int, NUI_LOCKED_RECT *);
};
