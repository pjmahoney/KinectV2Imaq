#include "../include/KinectDeviceInfo.h"


KinectDeviceInfo::KinectDeviceInfo()
{
}


KinectDeviceInfo::~KinectDeviceInfo()
{
}

IKinectSensor *KinectDeviceInfo::getDevice(void) const {
	return m_device;
}

void KinectDeviceInfo::setDevice(IKinectSensor *device) {
	m_device = device;
}

int KinectDeviceInfo::getFrameSourceType(void) const {
	return m_frameSourceType;
}

void KinectDeviceInfo::setFrameSourceType(int type) {
	m_frameSourceType = type;
}