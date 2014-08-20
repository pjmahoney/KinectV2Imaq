#include "../include/DepthAdapter.h"

DepthAdapter::DepthAdapter(imaqkit::IEngine* engine,
	const KinectDeviceInfo *deviceInfo,
	const char* formatName) 
	:imaqkit::IAdaptor(engine),
	m_frameEvent() {

		m_sensor = deviceInfo->getDevice();
	}

DepthAdapter::~DepthAdapter() {}

const char* DepthAdapter::getDriverDescription() const {
	return "KinectV2Depth_Driver";
}
const char* DepthAdapter::getDriverVersion() const {
	return "0.0.1";
}

WAITABLE_HANDLE DepthAdapter::getFrameEvent() {
	return m_frameEvent;
}

IKinectSensor *DepthAdapter::getSensor() const {
	return m_sensor;
}

IDepthFrameSource *DepthAdapter::getSource() const {
	return m_source;
}

IDepthFrameReader *DepthAdapter::getReader() const {
	return m_reader;
}

unsigned int DepthAdapter::getFrameSize() const {
	return m_frameSize;
}

bool DepthAdapter::aquireFrame() const {
	return m_aquireFrame;
}

void DepthAdapter::notifyCaptureFinished() {
	m_captureFinished = true;
}

bool DepthAdapter::openDevice() {

	if (isOpen()) {
		return true;
	}

	BOOLEAN open;

	if (FAILED(m_sensor->get_IsOpen(&open))) {
		imaqkit::adaptorError(this, "DepthAdapter:openDevice", "Unable to determine kinect device status.");
	}
	
	if (!open && FAILED(m_sensor->Open())) {
		imaqkit::adaptorError(this, "DepthAdapter:openDevice", "Unable to open kinect device.");
		closeDevice();
		return false;
	}

	if (FAILED(m_sensor->get_DepthFrameSource(&m_source))) {
		imaqkit::adaptorError(this, "DepthAdapter:openDevice", "Unable to get depth frame source from kinect device.");
		m_source = nullptr;
		closeDevice();
		return false;
	}

	if (FAILED(m_source->OpenReader(&m_reader))) {
		imaqkit::adaptorError(this, "DepthAdapter:openDevice", "Unable to get frame reader from depth source.");
		m_reader = nullptr;
		closeDevice();
		return false;
	}

	m_aquireThread = CreateThread(NULL, 0, aquireThread, this, 0, &m_aquireThreadID);
	if (m_aquireThread == NULL) {
		closeDevice();
		return false;
	}

	while (PostThreadMessage(m_aquireThreadID, WM_USER + 1, 0, 0) == 0)
		Sleep(1);

	return true;
}

DWORD WINAPI DepthAdapter::aquireThread(void* param) {
	DepthAdapter *adaptor = reinterpret_cast<DepthAdapter*>(param);

	MSG msg;

	IFrameDescription *desc;
	adaptor->getSource()->get_FrameDescription(&desc);

	unsigned int bpp, lip;
	int h, w;

	desc->get_BytesPerPixel(&bpp);
	desc->get_LengthInPixels(&lip);
	desc->get_Height(&h);
	desc->get_Width(&w);

	int frameSize = lip * bpp / 2;

	UINT16 *data = new UINT16[frameSize];

	while (GetMessage(&msg, NULL, 0, 0) > 0) {
		switch (msg.message) {
		case WM_USER:
			while (adaptor->isAcquisitionNotComplete() && adaptor->aquireFrame()) {
				WAITABLE_HANDLE frameEvent = adaptor->getFrameEvent();
				HANDLE handles[] = { reinterpret_cast <HANDLE>(frameEvent) };
				int idx;

				idx = WaitForMultipleObjects(1, handles, FALSE, 100);
				switch (idx)
				{
				case WAIT_TIMEOUT:
					continue;
				case WAIT_OBJECT_0:
					IDepthFrameArrivedEventArgs *args;
					if (FAILED(adaptor->getReader()->GetFrameArrivedEventData(frameEvent, &args))) {
						imaqkit::adaptorWarn("DepthAdapter:aquire", "Unable to aquire event data.");
						continue;
					}

					IDepthFrameReference *frameRef;
					args->get_FrameReference(&frameRef);

					IDepthFrame *frame;
					frameRef->AcquireFrame(&frame);

					frame->CopyFrameDataToArray(frameSize, data);

					frame->Release();
					frameRef->Release();
					args->Release();

					if (adaptor->isSendFrame()) {
						imaqkit::frametypes::FRAMETYPE frameType = adaptor->getFrameType();
						int imWidth = adaptor->getMaxWidth();
						int imHeight = adaptor->getMaxHeight();

						imaqkit::IAdaptorFrame *frame =
							adaptor->getEngine()->makeFrame(frameType, imWidth, imHeight);

						frame->setImage(data, imWidth, imHeight, 0, 0);

						frame->setTime(imaqkit::getCurrentTime());

						adaptor->getEngine()->receiveFrame(frame);
					}

					adaptor->incrementFrameCount();
					break;

				default:
					break;
				}
			}
			adaptor->notifyCaptureFinished();
			break;

		default:
			break;
		}
	}
	return 0;
}

bool DepthAdapter::closeDevice() {
	
	if (!isOpen()) {
		return true;
	}

	if (m_aquireThread) {
		PostThreadMessage(m_aquireThreadID, WM_QUIT, 0, 0);

		WaitForSingleObject(m_aquireThread, 10000);

		CloseHandle(m_aquireThread);
		m_aquireThread = NULL;
	}

	if (m_reader) {
		m_reader->Release();
		m_reader = nullptr;
	}
	
	if (m_source) {
		m_source->Release();
		m_source = nullptr;
	}
	

	BOOLEAN open;
	if (FAILED(m_sensor->get_IsOpen(&open))) {
		imaqkit::adaptorError(this, "DepthAdapter:closeDevice", "Unable to check device status.");
	}

	if (open && FAILED(m_sensor->Close())) {
		imaqkit::adaptorError(this, "DepthAdapter:closeDevice", "Unable to close kinect device.");
	}
	
	return true;
}

imaqkit::frametypes::FRAMETYPE DepthAdapter::getFrameType() const { 
	return imaqkit::frametypes::MONO12;
}
int DepthAdapter::getMaxHeight() const { return 424; }
int DepthAdapter::getMaxWidth() const { return 512; }
int DepthAdapter::getNumberOfBands() const { return 1; }

bool DepthAdapter::startCapture() {

	if (!isOpen()) {
		return false;
	}

	if (isAcquiring()) {
		return true;
	}

	m_captureFinished = false;

	if (FAILED(m_reader->SubscribeFrameArrived(&m_frameEvent)))	{
		imaqkit::adaptorError(this, "DepthAdapter:startCapture", "Unable to subscribe to frame arrived event.");
	}

	m_aquireFrame = true;

	PostThreadMessage(m_aquireThreadID, WM_USER, 0, 0);

	return true;
}
bool DepthAdapter::stopCapture() {
	if (!isOpen()) {
		return true;
	}

	if (!isAcquiring()) {
		return true;
	}

	m_reader->UnsubscribeFrameArrived(m_frameEvent);
	m_aquireFrame = false;

	int count = 0;
	while (!m_captureFinished && count++ < 100) {
		Sleep(1);
	}

	return true;
}