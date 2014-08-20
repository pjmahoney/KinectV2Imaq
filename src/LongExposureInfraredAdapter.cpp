#include "../include/LongExposureInfraredAdapter.h"

LongExposureInfraredAdapter::LongExposureInfraredAdapter(imaqkit::IEngine* engine,
	const KinectDeviceInfo *deviceInfo,
	const char* formatName) 
	:imaqkit::IAdaptor(engine),
	m_frameEvent() {

		m_sensor = deviceInfo->getDevice();
	}

LongExposureInfraredAdapter::~LongExposureInfraredAdapter() {}

const char* LongExposureInfraredAdapter::getDriverDescription() const {
	return "KinectV2LongExposureInfrared_Driver";
}
const char* LongExposureInfraredAdapter::getDriverVersion() const {
	return "0.0.1";
}

WAITABLE_HANDLE LongExposureInfraredAdapter::getFrameEvent() {
	return m_frameEvent;
}

IKinectSensor *LongExposureInfraredAdapter::getSensor() const {
	return m_sensor;
}

ILongExposureInfraredFrameSource *LongExposureInfraredAdapter::getSource() const {
	return m_source;
}

ILongExposureInfraredFrameReader *LongExposureInfraredAdapter::getReader() const {
	return m_reader;
}

unsigned int LongExposureInfraredAdapter::getFrameSize() const {
	return m_frameSize;
}

bool LongExposureInfraredAdapter::aquireFrame() const {
	return m_aquireFrame;
}

void LongExposureInfraredAdapter::notifyCaptureFinished() {
	m_captureFinished = true;
}

bool LongExposureInfraredAdapter::openDevice() {

	if (isOpen()) {
		return true;
	}

	BOOLEAN open;

	if (FAILED(m_sensor->get_IsOpen(&open))) {
		imaqkit::adaptorError(this, "LongExposureInfraredAdapter:openDevice", "Unable to determine kinect device status.");
	}
	
	if (!open && FAILED(m_sensor->Open())) {
		imaqkit::adaptorError(this, "LongExposureInfraredAdapter:openDevice", "Unable to open kinect device.");
		closeDevice();
		return false;
	}

	if (FAILED(m_sensor->get_LongExposureInfraredFrameSource(&m_source))) {
		imaqkit::adaptorError(this, "LongExposureInfraredAdapter:openDevice", "Unable to get LongExposureInfrared frame source from kinect device.");
		closeDevice();
		return false;
	}

	if (FAILED(m_source->OpenReader(&m_reader))) {
		imaqkit::adaptorError(this, "LongExposureInfraredAdapter:openDevice", "Unable to get frame reader from LongExposureInfrared source.");
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

DWORD WINAPI LongExposureInfraredAdapter::aquireThread(void* param) {
	LongExposureInfraredAdapter *adaptor = reinterpret_cast<LongExposureInfraredAdapter*>(param);

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
					ILongExposureInfraredFrameArrivedEventArgs *args;
					if (FAILED(adaptor->getReader()->GetFrameArrivedEventData(frameEvent, &args))) {
						imaqkit::adaptorWarn("LongExposureInfraredAdapter:aquire", "Unable to aquire event data.");
						continue;
					}

					ILongExposureInfraredFrameReference *frameRef;
					args->get_FrameReference(&frameRef);

					ILongExposureInfraredFrame *frame;
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

bool LongExposureInfraredAdapter::closeDevice() {
	
	if (!isOpen()) {
		return true;
	}

	if (m_aquireThread) {
		PostThreadMessage(m_aquireThreadID, WM_QUIT, 0, 0);

		WaitForSingleObject(m_aquireThread, 10000);

		CloseHandle(m_aquireThread);
		m_aquireThread = NULL;
	}

	BOOLEAN open;
	if (FAILED(m_sensor->get_IsOpen(&open))) {
		imaqkit::adaptorError(this, "LongExposureInfraredAdapter:closeDevice", "Unable to check device status.");
	}

	if (open && FAILED(m_sensor->Close())) {
		imaqkit::adaptorError(this, "LongExposureInfraredAdapter:closeDevice", "Unable to close kinect device.");
	}
	
	return true;
}

imaqkit::frametypes::FRAMETYPE LongExposureInfraredAdapter::getFrameType() const { 
	return imaqkit::frametypes::MONO16;
}
int LongExposureInfraredAdapter::getMaxHeight() const { return 424; }
int LongExposureInfraredAdapter::getMaxWidth() const { return 512; }
int LongExposureInfraredAdapter::getNumberOfBands() const { return 1; }

bool LongExposureInfraredAdapter::startCapture() {

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
bool LongExposureInfraredAdapter::stopCapture() {
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