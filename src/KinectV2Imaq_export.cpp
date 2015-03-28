#include <mwadaptorimaq.h>
#include <Kinect.h>

#include <comdef.h>

#include "../include/ColourAdapter.h"
#include "../include/DepthAdapter.h"
#include "../include/InfraredAdapter.h"
#include "../include/LongExposureInfraredAdapter.h"
#include "../include/KinectDeviceInfo.h"

void initializeAdaptor(){

}

void addKinectDevicetoHW(imaqkit::IHardwareInfo *hwInfo, IKinectSensor *kinect, int sensorId);

void getAvailHW(imaqkit::IHardwareInfo* hardwareInfo){
	
	//Multiple sensor support was removed from release sdk
	/*IKinectSensorCollection *kinectCollection;

	if (GetKinectSensorCollection(&kinectCollection) != S_OK) {
		imaqkit::adaptorError(nullptr, "KinectV2Imaq:DeviceEnumeration", "Unable to get kinect sensor collection.");
		return;
	}

	IEnumKinectSensor *kinectList;
	if (kinectCollection->get_Enumerator(&kinectList) != S_OK) {
		imaqkit::adaptorError(nullptr, "KinectV2Imaq:DeviceEnumeration", "Unable to get kinect sensor listing.");
		return;
	}

	IKinectSensor *kinectSensor;
	int deviceCount = 1;
	while (kinectList->GetNext(&kinectSensor) == S_OK) {
		//Add sensor devices
		addKinectDevicetoHW(hardwareInfo, kinectSensor, deviceCount++);
	}*/

	IKinectSensor* sensor;
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		imaqkit::adaptorError(nullptr, "KinectV2Imaq:DeviceEnumeration", "Unable to get kinect sensor.");
		return;
	}

	addKinectDevicetoHW(hardwareInfo, sensor, 1);

}

void addKinectDevicetoHW(imaqkit::IHardwareInfo *hwInfo, IKinectSensor *kinect, int sensorId) {

	int idBufferSize = 256;
	WCHAR wid[50];
	memset(wid, 0, 50 * sizeof(WCHAR));

	if (kinect->get_UniqueKinectId(idBufferSize, wid) != S_OK) {
		imaqkit::adaptorError(nullptr, "KinectV2Imaq:DeviceConstruction", "Unable to get kinect uid.");
		return;
	}

	_bstr_t b(wid);
	const char * id = b;
	
	const char* colourIdFormat = "Kinect v2 (%s) Colour Sensor";
	char * colourId = (char*)malloc(sizeof(char)* strlen(colourIdFormat) - 2 + strlen(id) + 2);
	sprintf(colourId, colourIdFormat, id);
	imaqkit::IDeviceInfo* colourInfo =
		hwInfo->createDeviceInfo((sensorId-1)*4+1, colourId);

	KinectDeviceInfo *colourKinectInfo = new KinectDeviceInfo();
	colourKinectInfo->setDevice(kinect);
	colourKinectInfo->setFrameSourceType(FrameSourceTypes::FrameSourceTypes_Color);

	colourInfo->setAdaptorData(colourKinectInfo);

	free(colourId);

	char *colorFormatNames[5] = { "RGB32_1920x1080", "YUV_UYVY_1920x1080", "BGR32_1920x1080", "BAYER_GRBG_1920x1080", "YUV_YUY2_1920x1080" };

	imaqkit::IDeviceFormat *colourFormat[5];
	for (int i = 0; i < 5; i++) {
		colourFormat[i] = colourInfo->createDeviceFormat(i + 1, colorFormatNames[i]);
		colourInfo->addDeviceFormat(colourFormat[i], i == 0);
	}

	hwInfo->addDevice(colourInfo);

	const char* depthIdFormat = "Kinect v2 (%s) Depth Sensor";
	char * depthId = (char*)malloc(sizeof(char)* strlen(depthIdFormat) - 2 + strlen(id) + 2);
	sprintf(depthId, depthIdFormat, id);

	imaqkit::IDeviceInfo* depthInfo =
		hwInfo->createDeviceInfo((sensorId - 1) * 4 + 2, depthId);

	KinectDeviceInfo *depthKinectInfo = new KinectDeviceInfo();
	depthKinectInfo->setDevice(kinect);
	depthKinectInfo->setFrameSourceType(FrameSourceTypes::FrameSourceTypes_Depth);

	depthInfo->setAdaptorData(depthKinectInfo);

	free(depthId);

	imaqkit::IDeviceFormat* depthFormat = depthInfo->createDeviceFormat(1, "MONO12_512x423");
	depthInfo->addDeviceFormat(depthFormat);

	hwInfo->addDevice(depthInfo);

	const char* infraredIdFormat = "Kinect v2 (%s) Infrared Sensor";
	char * infraredId = (char*)malloc(sizeof(char)* strlen(infraredIdFormat) - 2 + strlen(id) + 2);
	sprintf(infraredId, infraredIdFormat, id);

	imaqkit::IDeviceInfo* infraredInfo =
		hwInfo->createDeviceInfo((sensorId - 1) * 4 + 3, infraredId);

	KinectDeviceInfo *infraredKinectInfo = new KinectDeviceInfo();
	infraredKinectInfo->setDevice(kinect);
	infraredKinectInfo->setFrameSourceType(FrameSourceTypes::FrameSourceTypes_Infrared);

	infraredInfo->setAdaptorData(infraredKinectInfo);

	free(infraredId);

	imaqkit::IDeviceFormat*infraredFormat = depthInfo->createDeviceFormat(1, "MONO16_512x423");
	infraredInfo->addDeviceFormat(infraredFormat);

	hwInfo->addDevice(infraredInfo);

	const char* leInfraredIdFormat = "Kinect v2 (%s) Long Exposure Infrared Sensor";
	char * leInfraredId = (char*)malloc(sizeof(char)* strlen(leInfraredIdFormat) - 2 + strlen(id) + 2);
	sprintf(leInfraredId, leInfraredIdFormat, id);

	imaqkit::IDeviceInfo* leInfraredInfo =
		hwInfo->createDeviceInfo((sensorId - 1) * 4 + 4, leInfraredId);

	KinectDeviceInfo *leInfraredKinectInfo = new KinectDeviceInfo();
	leInfraredKinectInfo->setDevice(kinect);
	leInfraredKinectInfo->setFrameSourceType(FrameSourceTypes::FrameSourceTypes_LongExposureInfrared);

	leInfraredInfo->setAdaptorData(leInfraredKinectInfo);

	free(leInfraredId);

	imaqkit::IDeviceFormat*leInfraredFormat = depthInfo->createDeviceFormat(1, "MONO16_512x423");
	leInfraredInfo->addDeviceFormat(leInfraredFormat);

	hwInfo->addDevice(leInfraredInfo);
}

void getDeviceAttributes(const imaqkit::IDeviceInfo* deviceInfo,
	const char* formatName,
	imaqkit::IPropFactory* devicePropFact,
	imaqkit::IVideoSourceInfo* sourceContainer,
	imaqkit::ITriggerInfo* hwTriggerInfo){

	// Create a video source
	sourceContainer->addAdaptorSource("KinectV2Source", 1);

}

imaqkit::IAdaptor* createInstance(imaqkit::IEngine* engine, const
	imaqkit::IDeviceInfo* deviceInfo, const
	char* formatName){

	KinectDeviceInfo *info = dynamic_cast<KinectDeviceInfo*>(deviceInfo->getAdaptorData());

	if (info == nullptr) {
		imaqkit::adaptorError(nullptr, "KinectV2Imaq:createInstance", "Unable to get device info.");
		return nullptr;
	}

	imaqkit::IAdaptor *ca;
	switch (info->getFrameSourceType()) {
		case FrameSourceTypes::FrameSourceTypes_Color:
			ca = new ColourAdapter(engine, info, formatName);
			break; 

		case FrameSourceTypes::FrameSourceTypes_Depth:
			ca = new DepthAdapter(engine, info, formatName);
			break;

		case FrameSourceTypes::FrameSourceTypes_Infrared:
			ca = new InfraredAdapter(engine, info, formatName);
			break;

		case FrameSourceTypes::FrameSourceTypes_LongExposureInfrared:
			ca = new LongExposureInfraredAdapter(engine, info, formatName);
			break;

		default:
			ca = nullptr;
			break;
	}

	return ca;
}

void uninitializeAdaptor(){

}