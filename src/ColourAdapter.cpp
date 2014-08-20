#include "../include/ColourAdapter.h"

ColourAdapter::ColourAdapter(imaqkit::IEngine* engine,
	const KinectDeviceInfo *deviceInfo,
	const char* formatName) 
		:imaqkit::IAdaptor(engine),
		 m_frameEvent() {

	m_sensor = deviceInfo->getDevice();
	if (strcmp(formatName, "RGB32_1920x1080") == 0) {
		m_format = ColorImageFormat::ColorImageFormat_Rgba;
	}
	else if (strcmp(formatName, "YUV_UYVY_1920x1080") == 0) {
		m_format = ColorImageFormat::ColorImageFormat_Yuv;
	}
	else if (strcmp(formatName, "BGR32_1920x1080") == 0) {
		m_format = ColorImageFormat::ColorImageFormat_Bgra;
	}
	else if (strcmp(formatName, "BAYER_GRBG_1920x1080") == 0) {
		m_format = ColorImageFormat::ColorImageFormat_Bayer;
	}
	else if (strcmp(formatName, "YUV_YUY2_1920x1080") == 0) {
		m_format = ColorImageFormat::ColorImageFormat_Yuy2;
	}
}

ColourAdapter::~ColourAdapter() {}

const char* ColourAdapter::getDriverDescription() const {
	return "KinectV2Colour_Driver";
}
const char* ColourAdapter::getDriverVersion() const {
	return "0.0.1";
}

WAITABLE_HANDLE ColourAdapter::getFrameEvent() {
	return m_frameEvent;
}

IKinectSensor *ColourAdapter::getSensor() const {
	return m_sensor;
}

IColorFrameSource *ColourAdapter::getSource() const {
	return m_source;
}

IColorFrameReader *ColourAdapter::getReader() const {
	return m_reader;
}

ColorImageFormat ColourAdapter::getFormat() const {
	return m_format;
}

unsigned int ColourAdapter::getFrameSize() const {
	return m_frameSize;
}

bool ColourAdapter::aquireFrame() const {
	return m_aquireFrame;
}

void ColourAdapter::notifyCaptureFinished() {
	m_captureFinished = true;
}

bool ColourAdapter::openDevice() {

	if (isOpen()) {
		return true;
	}

	BOOLEAN open;

	if (FAILED(m_sensor->get_IsOpen(&open))) {
		imaqkit::adaptorError(this, "ColourAdapter:openDevice", "Unable to determine kinect device status.");
	}
	
	if (!open && FAILED(m_sensor->Open())) {
		imaqkit::adaptorError(this, "ColourAdapter:openDevice", "Unable to open kinect device.");
		closeDevice();
		return false;
	}

	if (FAILED(m_sensor->get_ColorFrameSource(&m_source))) {
		imaqkit::adaptorError(this, "ColourAdapter:openDevice", "Unable to get color frame source from kinect device.");
		closeDevice();
		return false;
	}

	IFrameDescription *desc;
	m_source->CreateFrameDescription(m_format, &desc);

	unsigned int bpp, ppf;
	desc->get_BytesPerPixel(&bpp);
	desc->get_LengthInPixels(&ppf);

	m_frameSize = bpp * ppf;

	if (FAILED(m_source->OpenReader(&m_reader))) {
		imaqkit::adaptorError(this, "ColourAdapter:openDevice", "Unable to get frame reader from color source.");
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

DWORD WINAPI ColourAdapter::aquireThread(void* param) {
	ColourAdapter *adaptor = reinterpret_cast<ColourAdapter*>(param);

	MSG msg;

	ColorImageFormat format = adaptor->getFormat();

	unsigned int frameSize = adaptor->getFrameSize();
	BYTE *data = new BYTE[frameSize];

	

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
					IColorFrameArrivedEventArgs *args;
					if (FAILED(adaptor->getReader()->GetFrameArrivedEventData(frameEvent, &args))) {
						imaqkit::adaptorWarn("ColourAdapter:aquire", "Unable to aquire event data.");
						continue;
					}

					IColorFrameReference *frameRef;
					args->get_FrameReference(&frameRef);

					IColorFrame *frame;
					frameRef->AcquireFrame(&frame);

					frame->CopyConvertedFrameDataToArray(frameSize, data, format);

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

bool ColourAdapter::closeDevice() {
	
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
		imaqkit::adaptorError(this, "ColourAdapter:closeDevice", "Unable to check device status.");
	}

	if (open && FAILED(m_sensor->Close())) {
		imaqkit::adaptorError(this, "ColourAdapter:closeDevice", "Unable to close kinect device.");
	}
	
	return true;
}

imaqkit::frametypes::FRAMETYPE ColourAdapter::getFrameType() const { 
	switch (m_format)
	{
	case ColorImageFormat::ColorImageFormat_Bayer:
		return imaqkit::frametypes::BAYER8_GRBG;
	case ColorImageFormat::ColorImageFormat_Bgra:
		return imaqkit::frametypes::BGR32_PACKED;
	case ColorImageFormat::ColorImageFormat_Rgba:
		return imaqkit::frametypes::RGB32_PACKED;
	case ColorImageFormat::ColorImageFormat_Yuv:
		return imaqkit::frametypes::YUV_UYVY;
	case ColorImageFormat::ColorImageFormat_Yuy2:
		return imaqkit::frametypes::YUV_YUY2;
	default:
		return imaqkit::frametypes::RGB32_PACKED;
		break;
	}
	
}
int ColourAdapter::getMaxHeight() const { return 1080; }
int ColourAdapter::getMaxWidth() const { return 1920; }
int ColourAdapter::getNumberOfBands() const { return 3; }

bool ColourAdapter::startCapture() {

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
bool ColourAdapter::stopCapture() {
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