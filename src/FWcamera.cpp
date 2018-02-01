#include "FWcamera.hpp"

static char * filename;

FWcamera :: FWcamera (void) {
	mIsInitialized = false;		// the cam. has not been initialized at this stage

	stopCapture = true;
	mIsGrabbingVideo = false; 	//note - it is never set to true

	nCard = 0; // this might change from 0 to 1

   	mShutter = 1085;
   	mGain = 0;
   	mBrightness = 490;
}

bool FWcamera :: isInitialized(void) {
	return mIsInitialized;
}

bool FWcamera :: initialize() {
	if (mIsInitialized)			// if the cam. has been initialized
		return true;

	dc1394error_t err;
	dc1394camera_list_t *list;

	md = dc1394_new(); /* Initialize libdc1394, Creates a new context in which cameras can be searched and used. This should be called before using any other libdc1394 function. */
	if (!md)
		return false;

	err = dc1394_camera_enumerate ( md, &list );                                /* Find cameras */
	DC1394_ERR_RTN (err,"Failed to enumerate cameras");

	/* Verify that we have at least one camera */
	if (list->num == 0) {
		dc1394_log_error("No cameras found");
		return false;
	}

	mCamera = dc1394_camera_new (md, FOCULUS_TC);
	if (!mCamera) {
		dc1394_log_error("Failed to initialize camera with guid ??\n", FOCULUS_TC);
    	return false;
	}
    dc1394_camera_free_list (list);

	err = dc1394_video_set_iso_speed (mCamera, DC1394_ISO_SPEED_400);
	if (err != 0) {
		g_print("Error in setting video ISO speed. Code : %i.\n",err);
		return false;
	}

	err = dc1394_video_set_mode(mCamera, DC1394_VIDEO_MODE_640x480_MONO8);
	if (err != 0) {
		g_print("Error in setting video mode. Code : %i.\n",err);
		return false;
	}

	DC1394_ERR_CLN_RTN (err, dc1394_camera_free (mCamera), "cannot choose camera format");
	printf ("I: video mode is set\n");

	uint32_t channel;
	dc1394_video_get_iso_channel(mCamera,&channel);
	printf ("ISO channel: %i\n", channel);

	mIsInitialized = true;

	if(!setGain(mGain)) {
		mIsInitialized = false;
		return false;
	}

	if(!setShutter(mShutter)) {
		mIsInitialized = false;
		return false;
	}

	if(!setBrightness(mBrightness)) {
		mIsInitialized = false;
		return false;
	}

	uint32_t band = 0;
	err = dc1394_video_get_bandwidth_usage(mCamera,&band);
	g_print("Top Bandwidth usage: %i\n", band);
	return true;
}

bool FWcamera::setShutter(int s) {
	mShutter = s;
	dc1394error_t err;
	if(mIsInitialized)
	{
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_SHUTTER, mShutter);
		if (err != 0)
		{
			g_print("Error in setting top shutter. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setGain(int g) {
	mGain = g;
	dc1394error_t err;
	if(mIsInitialized)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_GAIN, mGain);
		if (err != 0)
		{
			g_print("Error in setting top gain. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setBrightness(int g) {
	mBrightness = g;
	dc1394error_t err;
	if(mIsInitialized)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_BRIGHTNESS, mBrightness);
		if (err != 0)
		{
			g_print("Error in setting top brightness. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setFPS(int g) {
	mFPS = g;
	dc1394error_t err;
	if(mIsInitialized)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_FRAME_RATE, mFPS);
		if (err != 0)
		{
			g_print("Error in setting top FPS. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

FWcamera::~FWcamera(void)
{
	deinitialize();
}

void FWcamera::deinitialize( void ) {
	if (!mIsInitialized)			// if cam. is not initialized at this stage
		return;

	g_print("Camera deinitializing\n");
	mIsInitialized = false;
	stopCapture = true;
	mIsGrabbingVideo = false;

	dc1394_camera_free (mCamera);                                           /* cleanup and exit */

	dc1394_free(md);
	g_print("Camera deinitialized\n");
}

bool FWcamera::startGrabbingVideo() {
	dc1394error_t err;
	err=dc1394_capture_setup(mCamera, 8, DC1394_CAPTURE_FLAGS_DEFAULT);     /* Setup capture (camera, #images in buffer, capture flags) */
	if (err != 0)
	{
		g_print("Error in starting capture for top camera. Code: %i.\n",err);//-1 is DC1394_FAILURE
		return false;
	}
	err=dc1394_video_set_transmission(mCamera, DC1394_ON);                  /* Start transmission (camera, power) */
	if (err != 0)
	{
		g_print("Error in setting top camera transmission. Code: %i.\n",err);
		return false;
	}

	return true;
}

unsigned char * FWcamera::grabAframe(void) {
	int i=0;
	int whichFrame = 0; //which frame to return, 1 or 2.
	//unsigned char* src = (unsigned char*)malloc(sizeof(unsigned int)*640*480*1);
	dc1394error_t err;

	i = 0;
	do 	//this loop clears out old frames from the buffer to ensure we are getting the most recent frame.
	{	//It solves a problem where sections of the image are old and some are new when the buffer is not empty.
		err = dc1394_capture_dequeue(mCamera, DC1394_CAPTURE_POLICY_POLL, &mFrame);/* Capture */
		if (err != 0)
		{
			g_print("Error in top camera dequeue. Code: %i.\n",err);
			//return NULL;
		}
		if(mFrame != NULL)
			dc1394_capture_enqueue(mCamera, mFrame); //enqueue if successfully grabbed
		if( (mFrame != NULL) ) //if we got mFrame successfully, try again with mFrame2
		{

			err = dc1394_capture_dequeue(mCamera, DC1394_CAPTURE_POLICY_POLL, &mFrame2);/* Capture */
			if (err != 0)
			{
				g_print("Error in top camera dequeue. Code: %i.\n",err);
				//return NULL;
			}
			if(mFrame2 != NULL)
				dc1394_capture_enqueue(mCamera, mFrame2); //enqueue if successfully grabbed
			else
			{
				whichFrame = 1;
				break; //if mFrame success but mFrame2 fail
			}
		}
		i++;
	}while(   (mFrame == NULL)  ); //repeat while the first capture was unsuccessful

	if(whichFrame ==1)
		return mFrame->image;
	else
		return mFrame2->image;
}

bool FWcamera::isGrabbingVideo(void) {
	return mIsGrabbingVideo;
}

void FWcamera::stopGrabbingVideo() {
	printf("Stopping iso transmission...\n");
	dc1394error_t err;

	dc1394_capture_enqueue(mCamera,mFrame);
	usleep(1e3);
	err=dc1394_video_set_transmission(mCamera, DC1394_OFF);
	if (err != 0)
	{
		g_print("Error in top transmission set to OFF. Code: %i.\n",err);
		//return;
	}
	usleep(1e3);
	err=dc1394_capture_stop(mCamera);
	if (err != 0)
	{
		g_print("Error in stopping top capture. Code: %i.\n",err);
		//return;
	}

	mIsGrabbingVideo = false;
	printf("Iso transmission stopped...\n");
}
