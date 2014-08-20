#pragma once
#include <mwadaptorimaq.h>
#include <Kinect.h>
class KinectDeviceInfo :
	public imaqkit::IMAQInterface
{
public:
	KinectDeviceInfo(void);
	~KinectDeviceInfo(void);

	IKinectSensor *getDevice(void) const;
	void setDevice(IKinectSensor *device);

	int getFrameSourceType(void) const;
	void setFrameSourceType(int);

private:
	int m_frameSourceType;
	IKinectSensor *m_device;
};

