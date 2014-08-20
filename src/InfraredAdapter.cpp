#include "../include/InfraredAdapter.h"

InfraredAdapter::InfraredAdapter(imaqkit::IEngine* engine,
	const KinectDeviceInfo *deviceInfo,
	const char* formatName) 
	:imaqkit::IAdaptor(engine),
	m_frameEvent() {

		m_sensor = deviceInfo->getDevice();
	}

InfraredAdapter::~InfraredAdapter() {}

const char* InfraredAdapter::getDriverDescription() const {
	return "KinectV2Infrared_Driver";
}
const char* InfraredAdapter::getDriverVersion() const {
	return "0.0.1";
}

WAITABLE_HANDLE InfraredAdapter::getFrameEvent() {
	return m_frameEvent;
}

IKinectSensor *InfraredAdapter::getSensor() const {
	return m_sensor;
}

IInfraredFrameSource *InfraredAdapter::getSource() const {
	return m_source;
}

IInfraredFrameReader *InfraredAdapter::getReader() const {
	return m_reader;
}

unsigned int InfraredAdapter::getFrameSize() const {
	return m_frameSize;
}

bool InfraredAdapter::aquireFrame() const {
	return m_aquireFrame;
}

void InfraredAdapter::notifyCaptureFinished() {
	m_captureFinished = true;
}


bool InfraredAdapter::openDevice() {

	if (isOpen()) {
		return true;
	}

	BOOLEAN open;

	if (FAILED(m_sensor->get_IsOpen(&open))) {
		imaqkit::adaptorError(this, "InfraredAdapter:openDevice", "Unable to determine kinect device status.");
	}
	
	if (!open && FAILED(m_sensor->Open())) {
		imaqkit::adaptorError(this, "InfraredAdapter:openDevice", "Unable to open kinect device.");
		closeDevice();
		return false;
	}

	if (FAILED(m_sensor->get_InfraredFrameSource(&m_source))) {
		imaqkit::adaptorError(this, "InfraredAdapter:openDevice", "Unable to get Infrared frame source from kinect device.");
		closeDevice();
		return false;
	}

	if (FAILED(m_source->OpenReader(&m_reader))) {
		imaqkit::adaptorError(this, "InfraredAdapter:openDevice", "Unable to get frame reader from Infrared source.");
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

DWORD WINAPI InfraredAdapter::aquireThread(void* param) {
	InfraredAdapter *adaptor = reinterpret_cast<InfraredAdapter*>(param);

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
					IInfraredFrameArrivedEventArgs *args;
					if (FAILED(adaptor->getReader()->GetFrameArrivedEventData(frameEvent, &args))) {
						imaqkit::adaptorWarn("InfraredAdapter:aquire", "Unable to aquire event data.");
						continue;
					}

					IInfraredFrameReference *frameRef;
					args->get_FrameReference(&frameRef);

					IInfraredFrame *frame;
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

bool InfraredAdapter::closeDevice() {
	
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
		imaqkit::adaptorError(this, "InfraredAdapter:closeDevice", "Unable to check device status.");
	}

	if (open && FAILED(m_sensor->Close())) {
		imaqkit::adaptorError(this, "InfraredAdapter:closeDevice", "Unable to close kinect device.");
	}
	
	return true;
}

imaqkit::frametypes::FRAMETYPE InfraredAdapter::getFrameType() const { 
	return imaqkit::frametypes::MONO16;
}
int InfraredAdapter::getMaxHeight() const { return 424; }
int InfraredAdapter::getMaxWidth() const { return 512; }
int InfraredAdapter::getNumberOfBands() const { return 1; }

bool InfraredAdapter::startCapture() {

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
bool InfraredAdapter::stopCapture() {
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