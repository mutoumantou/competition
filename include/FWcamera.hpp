#ifndef FWCAMERA
#define FWCAMERA

#include "general_header.hpp"
#include <dc1394/dc1394.h>

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>
//#include <inttypes.h>

#define FOCULUS_TC 0x00097eff42802240
#define FOCULUS_TB 0x00097eff42909500
//#define FOCULUS 0x00097eff40300024

////#define PIXELINK 396320271505170
//#define PIXELINK 0x0001687381001712
//#define PIKE 0x000a470110071013

class FWcamera {
	public:
		FWcamera();			// constructor
		~FWcamera();		// destructor
		bool initialize();
		bool isInitialized();
		bool startGrabbingVideo();
		unsigned char * grabAframe(void);
		void stopGrabbingVideo();
		void startThreads();
		bool isGrabbingVideo();
		void deinitialize();
		bool setShutter(int s);
		bool setGain(int s);
		bool setBrightness(int g);
		bool setFPS(int f);

	private:
		bool mIsGrabbingVideo;
		bool mIsInitialized;			// whether or not the cam. has been initialized
		bool mIsInitialized_xz;

		// dc1394 stuff
		dc1394camera_t * mCamera;
		dc1394_t *md;
		dc1394video_frame_t * mFrame, *mFrame2;

		// camera settings
		int format;
		int mode;
		int frameRate;
		int bytesPerPixel;
		const char* device;
		int nCard;
		int cameraMode;
		bool showCap;
		bool stopCapture;
		char *sname;

		int mShutter;
		int mGain;
		int mBrightness;
		int mFPS;

		pthread_t t;
};

#endif
