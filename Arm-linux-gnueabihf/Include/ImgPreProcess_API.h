#ifndef APIIMGPREPROCESS_H
#define APIIMGPREPROCESS_H

#ifdef PS_EXPORT_ON
    #ifdef _WIN32
        #define VZENSE_API_EXPORT __declspec(dllexport)
    #else
        #define VZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#else
    #ifdef _WIN32
        #define VZENSE_API_EXPORT __declspec(dllimport)
    #else
        #define VZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#endif

#ifdef __cplusplus
#define VZENSE_C_API_EXPORT extern "C" VZENSE_API_EXPORT
#else
#define VZENSE_C_API_EXPORT VZENSE_API_EXPORT
#endif

typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

enum ALG_DepthRange { Near, Mid, Far, XNear, XMid, XFar, XXNear, XXMid, XXFar, NeedInit };
enum ALG_ElemType { ElemNone, U8C1, U8C3, U16C1, ElemMAX };

#pragma pack (push, 1)
struct ALG_CameraParams
{
	double tofIntrinsic[9];
	double tofDistortion[8];
	double clorIntrinsic[9];
	double colorDistortion[8];
	double rotation[9];
	double transfer[3];
	double e[9];
	double f[9];
};

struct ALG_Frame 
{
	unsigned char* pBuf;
	ALG_ElemType type;
	unsigned int w;
	unsigned int h;
};

struct ALG_LuminanceAdjustReturn
{
	// IR luminance adjust
	bool bIrLumiAdjust;  // Whether to adjust IR luminance. If adjust,get the uiIrGain.
	uint16_t uiIrGain;

	//RGB luminance adjust
	bool bRgbLumiAdjust; //Whether to adjust RGB luminance. If adjust, get the ucWeightMatrix[5][5]
	uint8_t ucWeightMatrix[5][5];

};

struct ALG_Vector3f
{
	float x, y, z;
};

/* Depth Image Coordination Vector*/
struct ALG_DepthVector3
{
	int          depthX;    // x in pixel
	int          depthY;    // y in pixel
	unsigned short depthZ;    // z in mm
};

/**
* @brief Specifies the image pixel format.
*/
typedef enum {
	ALGPixelFormatDepthMM16,        //!< Depth image pixel format, 16 bits per pixel in mm.
	ALGPixelFormatGray16,           //!< IR image pixel format, 16 bits per pixel.
	ALGPixelFormatGray8,            //!< Gray image pixel format, 8 bits per pixel.

   //Color
   ALGPixelFormatRGB888,           //!< Color image pixel format, 24 bits per pixel RGB format.
   ALGPixelFormatBGR888,           //!< Color image pixel format, 24 bits per pixel BGR format.
   ALGPixelFormatNV12,
   ALGPixelFormatYUV420P
}ALG_PixelFormat;


typedef struct
{
	unsigned short timeFilterThreshold;
	unsigned short timeShiftRatio;
	unsigned short fillHoleThreshold;
	unsigned short badBlockThreshold;
	unsigned short firstSpatialThreshold;
	unsigned short twiceSpatialThrehold;
	unsigned short spatialSwitchThreshold;
	unsigned short spatialSwitchThresholdEdge;
	unsigned short spatialCount;
	unsigned short fillHoleCount;
	unsigned short outputAscend;
}ALG_FilterThreshold;

#pragma pack (pop)


class APIImgPreProcess
{

public:

	virtual ~APIImgPreProcess() {};
	/*!
	*  Init
	*  @Parameters:
	*      const ALG_CameraParams& params[In]: internal parameters, external parameters, distortion parameters of tof and color lens
    *      const char* calibFilePath:   the  Calib File Path for camera which index is deviceIndex.
	*  @Return: 0: OK
	*/
	virtual int PreProcInit(const ALG_CameraParams&  params, const char* pCalibFilePath)=0;



	/*!
	*  PreProcSetIntrinsic
	*  @Parameters:
	*      int type[In]:  camera type   0: TOF         1:RGB
	*      double *IntrinsicData    the pointer of the Intrinsic Array.
	*      int ImgWidth,int ImgHeight: the width and height of image(tof or rgb).
	*  @Return: 0: OK
	*/
	virtual int PreProcSetIntrinsic(int type, double *IntrinsicData,int ImgWidth,int ImgHeight) = 0;


	/*!
	*  SetPlaneCorrectionParam
	*  @Parameters:
	*      int type[In]: PlaneCorrection Methods indication type.
	*      float value[][In] : the plane correction Coefficient
	*      int length [In]:   the  length of plane correction Coefficient.
	*  @Return: 0: OK
	*/
	virtual void SetPlaneCorrectionParam(int type, float value[], int length)=0;

	/*!
	*  SetFilterThreshold
	*  @Parameters:
	*  ALG_FilterThreshold  threshold [In]:  The filter Parameters  which we want.
	*  @Return: 0: OK
	*/
	virtual void SetFilterThreshold(ALG_FilterThreshold  threshold)=0;

	/*!
	*  SetSaveComputeDepthTab
	*  set the SaveRealDepthTab enable.
	*  @Return: 0: OK
	*/
	virtual void SetSaveComputeDepthTab()=0;

	/*!
	*  PreProcGetCameraParams
	*     ALG_CameraParams& params[In/Out]: internal parameters, external parameters, distortion parameters of tof and color lens
	*  @Return: 0: OK
	*/
	virtual int PreProcGetCameraParams(ALG_CameraParams&  params)=0;
	/*!
	*  Shutdown
	*  @Parameters:void
	*  @Return: 0: OK
	*/
	virtual int PreProcShutDown(void)=0;

	/*!
	*  Set the working mode of the tof camera and the effective measurement range
	*  @Parameters:
	*      const ALG_DepthRange range[In]: work mode
	*      const unsigned int depthMax[In]: max depth
	*      const unsigned int depthMin[In]: min depth, Generally set to zero
	*  @Return: 0: OK
	*/
	virtual int PreProcSetMeasuringRange(const ALG_DepthRange range, const unsigned int depthMax, const unsigned int depthMin)=0;

	//IR
	/*!
	*  IR image anti-distortion setting switch
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetIrUnDistortionEnabled(bool enable)=0;
	virtual bool PreProcIsIrUnDistortionEnabled()=0;
	/*!
	*  IR image processing function
	*  @Parameters:
	*      ALG_Frame& ir[In\Out]:Ir is the incoming IR image to be processed, and the pre-processed result is also returned with this parameter;
	*  @Return: 0: OK
	*/
	virtual int PreProcIR(ALG_Frame& ir)=0;

	//RGB
	/*!
	*  RGB image correction function
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetRGBCorrectionEnabled(bool enable)=0;
	virtual bool PreProcIsRGBCorrectionEnabled()=0;
	/*!
	*  RGB image anti-distortion setting switch
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetRGBUnDistortionEnabled(bool enable)=0;
	virtual bool PreProcIsRGBUnDistortionEnabled()=0;

	/*!
	*  RGB image ConvertFormat
	*  @Parameters:
	*     ALG_Frame& rgb[In\Out]:rgb is the incoming IR image to be processed, and the pre-processed result is also returned with this parameter;
	*     int decodetype:    0: mjpeg mode already decoded to RGB or BGR    1: h264 mode(ffmpeg decode),nv12 or yuv420p
	*     ALG_PixelFormat srcformat:
	*     ALG_PixelFormat destformat:
	*  @Return: 0: OK
	*/
	virtual	int PreProcRGBConvertFormat(ALG_Frame& rgb, int decodetype, ALG_PixelFormat srcformat, ALG_PixelFormat destformat) = 0;

	/*!
	*  RGB image processing function
	*  @Parameters:
	*     ALG_Frame& rgb[In\Out]:rgb is the incoming IR image to be processed, and the pre-processed result is also returned with this parameter;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB(ALG_Frame& rgb)=0;

	//Depth
	/*!
	*  Depth image correction switch setting function
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetRealDepthCorrectionEnabled(bool enable)=0;
	virtual bool PreProcIsRealDepthCorrectionEnabled()=0;

	/*!
	*  Depth image anti-distortion setting switch
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetDepthUnDistortionEnabled(bool enable)=0;
	virtual bool PreProcIsDepthUnDistortionEnabled()=0;

	/*!
	*  Set whether the Depth image uses filtering
	*  @brief:
	*      Only this function is set to true, PreProcSetTimeFilterEnabled
	*      and PreProcSetSpatialFilterEnable are  configurable.
	*  @Parameters:
	*      bool isUsed[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetUsedDepthFilter(bool isUsed)=0;
	virtual bool PreProcIsUsedDepthFilter()=0;

	/*!
	*  Set the Depth image time filter switch function
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetTimeFilterEnabled(bool enable)=0;
	virtual bool PreProcIsTimeFilterEnabled()=0;
	/*!
	*  Set the Depth image spatial filter switch function
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetSpatialFilterEnabled(bool enable)=0;
	virtual bool PreProcIsSpatialFilterEnabled()=0;


	/*!
	*  Set the Depth image Spatial Filter DownSampling switch funtion
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetSpatialFilterDownSampleEnabled(bool enable)=0;
	virtual bool PreProcIsSpatialFilterDownSampleEnabled()=0;


	/*!
	*  PreProcSetSpecialCase function
	*  @Parameters:
	*      int index:    0: common use            1:special case(T_X)
	*  @Return: 0: OK
	*/
	virtual int PreProcSetSpecialCase(int index) = 0;

	/*!
	*  PreProcGetSpecialCase function
	*  @ReturnValue[out]:  the Special Case Index:   0: common use            1:special case(T_X)
	*/
	virtual int PreProcGetSpecialCase() = 0;

	/*!
	*  Depth image processing function
	*  @Parameters:
	*      ALG_Frame& depth[In\Out]:depth is depth image to be processed, and the pre-processed result is also returned with this parameter;
	*  @Return: 0: OK
	*/


	virtual int PreProcDepth(ALG_Frame& depth)=0;

	//Map
	/*!
	*  input the coordinate of depth ,then get the corresponding coordinate of rgb,
	*  @Parameters:
	*      const int input_x:input depth x;
	*      const int input_y:input depth y;
	*      const unsigned short input_depth:input depth value;
	*      int& output_x:output rgb x
	*      int& output_y:output rgb y
	*      const float scaleFactor:scale factor of
	*  @Return: 0: OK
	*/
	virtual int PreProcDepthCoordinate2RGB(const int input_x, const int input_y, const unsigned short input_depth, int& output_x, int& output_y, const float scaleFactor)=0;

	/*!
	*  Depth frame with 16bits per pixel in mm that is mapped to RGB camera space and resolution is same as RGB frame's,
	*  @Parameters:
	*      const ALG_Frame& rgb[In]:input rgb data;
	*      const ALG_Frame& depth[In]:input depth data;
	*      ALG_Frame& dest[Out]:out mapped depth data;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB2Depth(const ALG_Frame& rgb, const ALG_Frame& depth, ALG_Frame& dest)=0;

	/*!
	*  RGB frame with 24bits that is mapped to depth camera space and resolution is same as depth frame's,
	*  @Parameters:
	*      const ALG_Frame& depth[In]:input depth data;
	*      const ALG_Frame& rgb[In]:input rgb data;
	*      ALG_Frame& dest[Out]:out mapped rgb data;
	*  @Return: 0: OK
	*/
	virtual int PreProcDepth2RGB(const ALG_Frame& depth, const ALG_Frame& rgb, ALG_Frame& dest)=0;

	/*!
	*  IR frame with 16bits per pixel that is mapped to RGB camera space and resolution is same as RGB frame's,
	*  @Parameters:
	*      const ALG_Frame& rgb[In]:input rgb data;
	*      const ALG_Frame& depth[In]:input depth data;
	*      const ALG_Frame& ir[In]:input ir data;
	*      ALG_Frame& out_ir[Out]:out mapped ir data;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB2IR(const ALG_Frame& rgb, const ALG_Frame& depth, const ALG_Frame& ir, ALG_Frame& out_ir) = 0;

	/*!
	*  IR frame and Depth frame with 16bits per pixel that is mapped to RGB camera space and resolution is same as RGB frame's,
	*  @Parameters:
	*      const ALG_Frame& rgb[In]:input rgb data;
	*      const ALG_Frame& depth[In]:input depth data;
	*      const ALG_Frame& ir[In]:input ir data;
	*      ALG_Frame& out_depth[Out]:out mapped depth data;
	*      ALG_Frame& out_ir[Out]:out mapped ir data;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB2IRandDepth(const ALG_Frame& rgb, const ALG_Frame& depth, const ALG_Frame& ir, ALG_Frame& out_depth, ALG_Frame& out_ir) = 0;
	virtual int PreProcRGB2IRandDepthOptimize(const ALG_Frame& rgb, const ALG_Frame& depth, const ALG_Frame& ir, ALG_Frame& out_depth, ALG_Frame& out_ir) = 0;
	/*!
	*  YUV -> RGB
	*  @Parameters:
	*      const ALG_Frame& nv12[In]:input NV12 data;
	*      ALG_Frame& rgb[Out]:output rgb data;
	*  @Return: 0: OK
	*/
	virtual int PreProcNV122RGB(const ALG_Frame& nv12, ALG_Frame& rgb)=0;

	/*!
	*  YUV -> BGR
	*  @Parameters:
	*      const ALG_Frame& nv12[In]:input NV12 data;
	*      ALG_Frame& bgr[Out]:output bgr data;
	*  @Return: 0: OK
	*/
	virtual int PreProcNV122BGR(const ALG_Frame& nv12, ALG_Frame& bgr)=0;

	/*!
	*  YUV -> RGB
	*  @Parameters:
	*      const ALG_Frame& nv21[In]:input NV21 data;
	*      ALG_Frame& rgb[Out]:output rgb data;
	*  @Return: 0: OK
	*/
	virtual int PreProcNV212RGB(const ALG_Frame& nv21, ALG_Frame& rgb)=0;

	/*!
	*  NV21 -> BGR
	*  @Parameters:
	*      const ALG_Frame& nv21[In]:input NV21 data;
	*      ALG_Frame& bgr[Out]:output bgr data;
	*  @Return: 0: OK
	*/
	virtual int PreProcNV212BGR(const ALG_Frame& nv21, ALG_Frame& bgr)=0;


	/*!
	*  Flip Img
	*  @Parameters:
	*      const ALG_Frame& img[In\Out]:img data;
	*      int type[In]:a flag to specify how to flip the array; 0 means
	*flipping around the x-axis and positive value (for example, 1) means
	*flipping around y-axis. Negative value (for example, -1) means flipping
	*around both axes
	*  @Return: 0: OK
	*/
	virtual int PreProcFlip(ALG_Frame& img, int type)=0;

	/*!
	*  Converts the input points from the World coordinate system to the Depth coordinate system
	*  @Parameters:
	*	pWorldVector[In]: pointer to the buffer which stored the x,y,z value of world coordinate of the input points to be converted, measured in millimeters
	*	pDepthVect[Out]: pointer to the buffer to store the output x,y,z value of depth coordinate
	*	                 (x,y) is measured in pixels with (0,0) at the top left of the image
	*	                 z is measured in millimeters, same as the reference of PsPixelFormat of depth frame
	*	pointCount[In]: the point count to be converted
	*	pTofIntrinsic[In]: input pointer to the TOF lens Internal reference
	*  @Return: 0: OK
	*/
	virtual int PreProcConvertWorldToDepth(ALG_Vector3f* pWorldVector, ALG_DepthVector3* pDepthVector, unsigned int pointCount, double* pTofIntrinsic)=0;

	/*!
	*  Converts the input points from the Depth coordinate system to the World coordinate system
	*  @Parameters:
	*	pDepthVector[In]: pointer to the buffer which stored the x,y,z value of depth coordinate of the input points to be converted
	*	                  (x,y) is measured in pixels with (0,0) at the top left of the image
	*	                  z is measured in millimeters, same as the reference of PsPixelFormat of depth frame
	*	pWorldVector[Out]: pointer to the buffer to store the output x,y,z value of world coordinate, measured in millimeters
	*	pointCount[In]: the point count to be converted
	*	pTofIntrinsic[In]: input pointer to the TOF lens Internal reference
	*  @Return: 0: OK
	*/
	virtual int PreProcConvertDepthToWorld(ALG_DepthVector3* pDepthVector, ALG_Vector3f* pWorldVector, unsigned int pointCount, double* pTofIntrinsic)=0;


	/*!
	*  Resize the img resolution
	*  @Parameters:
	*   ALG_Frame& img[In][Out]:input and output img data
	*	destWidth[in]:		dest resolution width
	*	destHeigh[in]:		dest resolution height
	*   intertype[in]:      interpolation type  0: nearest interpolation   not 0:bilinear interpolation
	*  @Return:				0: OK
	*/

	virtual int PreProcImgResize(ALG_Frame& img, int destWidth, int destHeigh,int intertype=1) = 0;

	/*!
	*  Cut the img
	*  @Parameters:
	*   ALG_Frame& img[In][Out]:input and output img data
	*   int originX    the origin X coordinate of Cut region
	*   int originY    the origin Y coordinate of Cut region
	*	destWidth[in]:		dest resolution width
	*	destHeigh[In]:		dest resolution height
	*  @Return:				0: OK
	*/

	virtual int PreProcImgCut(ALG_Frame& img, int originX,int originY, int destWidth, int destHeigh) = 0;

	/*!
	*  Flip Img
	*  @Parameters:
	*       ALG_Frame& img[In\Out]:img data;
	*       int type[In]:a flag to specify how to rotate the image
	*  0:counterclock 90 degree     1:counterclock 180 degree     2:counterclock 270degree
	*  @Return: 0: OK
	*/
	virtual int PreProcRotateImg(ALG_Frame& img, int type)=0;


	/*!
	*  Set the Getting the depth image only used time filter Function
	*  @Parameters:
	*      bool enable[In]:true:enable; false:disable;
	*  @Return: 0: OK
	*/
	virtual int PreProcSetOnlyTimeFilterDepth(bool enable)=0;
	virtual bool PreProcIsOnlyTimeFilterDepth()=0;

	/*!
	*  PreProcGetOnlyTimeFilterDepth
	*  @Parameters:
	*      ALG_Frame& depth[In\Out]:depth is the  Only TimeFiltered
	*  @Return: 0: OK
	*/


	virtual int PreProcGetOnlyTimeFilterDepth(ALG_Frame& depth)=0;

	/*!
	*  PreProcSetOutofMaxRangeHandle
	*  @Parameters:
	*      int index: 0: set the range out of MaxRange to  0xFFFF.         1:set the range out of MaxRange to 0.
	*  @Return: 0: OK
	*/


	virtual int PreProcSetOutofMaxRangeHandle(int index) = 0;


	/*!
	*  PreProcGetOutofMaxRangeHandle
	*  @ReturnValue[out]:  OutofMaxRangeHandle Index:   0: set the range out of MaxRange to  0xFFFF      1:set the range out of MaxRange to 0.
	*/
	virtual int PreProcGetOutofMaxRangeHandle() = 0;


	/*!
	*  PreProcAdjustLuminance
	*[in] ALG_Frame *depthFrame   the pointer of depth Image
	*[in] ALG_Frame *rgbFrame     the pointer of rgb Image, if only adjust the luminance of IR , the pointer is set to Null.
	*[in] int type    0: only adjust IR   1: only adjust rgb        2: adjust IR and rgb.
	*[in] uint16_t mindepth, uint16_t maxdepth : the  mindepth and maxdepth of the focus application
	*[in/out] ALG_LuminanceAdjustReturn &lumiadjust : the Luminance Adjust ReturnValue
	*/
	virtual int PreProcAdjustLuminance(ALG_Frame *depthFrame,ALG_Frame *rgbFrame, int type,uint16_t mindepth, uint16_t maxdepth, ALG_LuminanceAdjustReturn &lumiadjust) = 0;

	/*!
	*  PreProcIRU16ToU8
	*[in] int height, int width: IR Image height and IR Image Width
	*[in] unsigned char* pSource : the data pointer of the source U16 IR Image
	*[out] unsigned char* pDest: the data pointer of the dest U8 IR Image.
	*/
	virtual int PreProcIRU16ToU8(int height, int width, unsigned char* pDest, unsigned char* pSource) = 0;
	
	int mSensorID;

};


VZENSE_C_API_EXPORT APIImgPreProcess * Get_APIImgPreProcess();
//class DLL_API UserImgPreProcess
//{
//   public:
//	static APIImgPreProcess * Get_APIImgPreProcess();
//};

#endif // APIIMGPREPROCESS_H

