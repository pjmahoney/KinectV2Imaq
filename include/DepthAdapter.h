#pragma once
#pragma once

#include <mwadaptorimaq.h>
#include <Kinect.h>

#include "KinectDeviceInfo.h"

class DepthAdapter :
	public imaqkit::IAdaptor
{
public:
	DepthAdapter(imaqkit::IEngine* engine,
		const KinectDeviceInfo *deviceInfo,
		const char* formatName);
	~DepthAdapter();

	//Driver information
	virtual const char* getDriverDescription() const override;
	virtual const char* getDriverVersion() const override;

	//Device control functions
	virtual bool openDevice() override;
	virtual bool closeDevice() override;

	//Device frame information
	virtual imaqkit::frametypes::FRAMETYPE getFrameType() const override;
	virtual int getMaxHeight() const override;
	virtual int getMaxWidth() const override;
	virtual int getNumberOfBands() const override;

	virtual WAITABLE_HANDLE getFrameEvent();
	IKinectSensor *getSensor() const;
	IDepthFrameSource *getSource() const;
	IDepthFrameReader *getReader() const;
	unsigned int getFrameSize() const;

	bool aquireFrame() const;
	void notifyCaptureFinished();

	//Device capture control
	virtual bool startCapture() override;
	virtual bool stopCapture() override;

private:
	IKinectSensor *m_sensor;
	IDepthFrameSource *m_source;
	IDepthFrameReader *m_reader; 

	static DWORD WINAPI aquireThread(void* param);
	HANDLE m_aquireThread;
	DWORD m_aquireThreadID;

	WAITABLE_HANDLE m_frameEvent;

	unsigned int m_frameSize;

	bool m_aquireFrame;
	bool m_captureFinished;
};

