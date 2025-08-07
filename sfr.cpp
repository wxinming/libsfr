#include "sfr.h"
#include "mitre_sfr.h"

int sfr::Area::_size = 0;

#if defined(_WIN64)|| defined(__x86_64__)
typedef unsigned long long PtrSize;
#else
typedef unsigned long PtrSize;
#endif

sfr::Data::Data()
{
	frequency = 0.125;
	center = 100;
	circum = 90;
	interval = 10;
	fovp = 50;
}

sfr::Data::~Data()
{

}

sfr::Enable::Enable()
{
	drawLocateCenter = true;
	drawResultPass = true;
	//drawFps = true;
	drawFovp = true;
}

sfr::Enable::~Enable()
{

}

sfr::Area::Area() 
{
	x = 0;
	y = 0;
	width = 0;
	height = 0;
	denoisePixel = 10;
	threshold = 90;
	locateType = BLACK_SECTOR_WITH_WHITE_BACKGROUND;
	roi = {};
	++_size;
	contrast = 1.0;
	brightness = 1;
}

sfr::Area::~Area()
{
	--_size;
}

sfr::Paint::Paint()
{
	textScale = 1.5;
	textColor = CV_RGB(0, 255, 255);
	textThickness = 2;
	areaRectColor = CV_RGB(255, 255, 0);
	areaRectThickness = 2;
	roiRectColor = CV_RGB(255, 0, 0);
	roiRectThickness = 1;
	locateLineColor = CV_RGB(255, 0, 0);
	locateLineThickness = 2;
	locateLineLength = 10;
	centerLineColor = CV_RGB(255, 250, 205);
	centerLineThickness = 1;
	fovLineColor = CV_RGB(255, 250, 205);
	fovLineThickness = 1;
}

sfr::Paint::~Paint()
{

}

double& sfr::Frequency::operator*()
{
	if (!_ptr0)
		_ptr0 = reinterpret_cast<double*>(this);
	else
		if (reinterpret_cast<PtrSize>(_ptr0) - reinterpret_cast<PtrSize>(this) >= sizeof(double) * MAX_AREA_SIZE)
			_ptr0 = reinterpret_cast<double*>(this);
	return *(_ptr1 = _ptr0++);
}

double sfr::Frequency::total() const
{
	return center + leftTop + rightTop + leftBottom + rightBottom;
}

void sfr::Area::startGrab(const std::function<void(int index, const cv::Mat& mat)>& func)
{
	std::lock_guard<std::mutex> lock(_mutex);
	_grab = func;
}

void sfr::Area::stopGrab()
{
	std::lock_guard<std::mutex> lock(_mutex);
	_grab = nullptr;
}

sfr::Algorithm::Algorithm()
{

}

sfr::Algorithm::Algorithm(sfr::Area* area, sfr::Data* data, sfr::Enable* enable, sfr::Paint* paint)
{
	initialize(area, data, enable, paint);
}

sfr::Algorithm::~Algorithm()
{

}

void sfr::Algorithm::initialize(sfr::Area* area, sfr::Data* data, sfr::Enable* enable, sfr::Paint* paint)
{
	m_data = data;
	m_area = area;
	m_enable = enable;
	m_paint = paint;
	for (int i = 0; i < sfr::Area::_size; ++i) {
		m_area[i]._rect = cv::Rect(m_area[i].x, m_area[i].y, m_area[i].width, m_area[i].height);
	}
}

bool sfr::Algorithm::isCalculate(int index)
{
	double tick = cv::getTickCount() / cv::getTickFrequency() * 1000;
	if (tick >= m_area[index]._tick + m_data->interval)
	{
		m_area[index]._tick = tick;
		return true;
	}
	return false;
}

static bool findSectorCrossLine(std::vector<cv::Point2i>& coord, std::vector<cv::Point2i>& vec)
{
	if (coord.empty())
	{
		return false;
	}

	const int interval = 5;
	//前顶点
	std::sort(coord.begin(), coord.end(), [](const cv::Point& a, const cv::Point& b) {return a.y < b.y; });
	cv::Point topP = coord.at(0);
	std::vector<int> topX;
	for (auto& x : coord)
	{
		if (abs(x.y - topP.y) <= interval)
		{
			topX.push_back(x.x);
		}
	}

	if (topX.empty())
	{
		return false;
	}

	std::sort(topX.begin(), topX.end(), [](int a, int b) {return a > b; });
	topP.x = topX.at(0);
	vec.push_back(topP);

	//后顶点
	std::sort(coord.begin(), coord.end(), [](const cv::Point& a, const cv::Point& b) {return a.y > b.y; });
	cv::Point bottomP = coord.at(0);
	std::vector<int> bottomX;
	for (auto& x : coord)
	{
		if (abs(x.y - bottomP.y) <= interval)
		{
			bottomX.push_back(x.x);
		}
	}

	if (bottomX.empty())
	{
		return false;
	}

	std::sort(bottomX.begin(), bottomX.end(), [](int a, int b) {return a < b; });
	bottomP.x = bottomX.at(0);
	vec.push_back(bottomP);

	//左侧点
	std::sort(coord.begin(), coord.end(), [](const cv::Point& a, const cv::Point& b) {return (a.x < b.x); });
	cv::Point leftP = coord.at(0);
	std::vector<int> leftY;
	for (auto& x : coord)
	{
		if (abs(x.x - leftP.x) <= interval)
		{
			leftY.push_back(x.y);
		}
	}

	if (leftY.empty())
	{
		return false;
	}

	std::sort(leftY.begin(), leftY.end(), [](int a, int b) {return a > b; });
	leftP.y = leftY.at(0);
	vec.push_back(leftP);

	//右侧点
	std::sort(coord.begin(), coord.end(), [](const cv::Point& a, const cv::Point& b) {return (a.x > b.x); });
	cv::Point rightP = coord.at(0);
	std::vector<int> rightY;
	for (auto& x : coord)
	{
		if (abs(x.x - rightP.x) <= interval)
		{
			rightY.push_back(x.y);
		}
	}

	if (rightY.empty())
	{
		return false;
	}

	std::sort(rightY.begin(), rightY.end(), [](int a, int b) {return a < b; });
	rightP.y = rightY.at(0);
	vec.push_back(rightP);
	return true;
}

static bool findTrapezoidCrossLine(cv::Mat& img, std::vector<cv::Point2i>& coord, 
	std::vector<cv::Point2i>& vec, int locateType)
{
	if (coord.empty())
	{
		return false;
	}

	auto locateValue = locateType == sfr::BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND ? 0x00 : 0xff;
	//前顶点
	std::sort(coord.begin(), coord.end(), [](const cv::Point& a, const cv::Point& b) {return a.y < b.y; });
	cv::Point topP = coord.at(0);

	//判断方向
	int needToOffsetCount = 100, forwardOffsetCount = 0, reverseOffsetCount = 0;
	for (int i = 0; i < needToOffsetCount; ++i) {
		//黑色图形白底
		auto offsetX = topP.x - i, offsetY = topP.y + i;
		if (offsetX >= 0 && offsetY < img.rows - 1) {
			auto pixmap = img.at<uchar>(offsetY, offsetX);
			if (pixmap == locateValue) {
				++forwardOffsetCount;
			}
			else {
				++reverseOffsetCount;
			}
		}
	}

	//判断是否为正向
	auto isForward = forwardOffsetCount > reverseOffsetCount;
	//printf("is forward %d, forward count %d, reverse count %d\n", isForward, forwardOffsetCount, reverseOffsetCount);

	int offsetCount = 0;
	for (int i = 0; i < needToOffsetCount; ++i) {
		//黑色图形白底
		auto offsetX = 0, offsetY = 0;
		if (isForward) {
			offsetX = topP.x + i;
			offsetY = topP.y + i;
		}
		else {
			offsetX = topP.x - i;
			offsetY = topP.y - i;
		}

		if (offsetX >= 0 && offsetX < img.cols - 1 &&
			offsetY >= 0 && offsetY < img.rows - 1) {
			if (img.at<uchar>(offsetY, offsetX) == locateValue) {
				++offsetCount;
			}
		}
	}
	//printf("\n top offsetCount %d\n", offsetCount);
	//修正x坐标到最精确坐标
	if (isForward) {
		topP.x += offsetCount;
	}
	else {
		topP.x -= offsetCount;
	}
	vec.push_back(topP);

	//后顶点
	std::sort(coord.begin(), coord.end(), [](const cv::Point& a, const cv::Point& b) {return a.y > b.y; });
	cv::Point bottomP = coord.at(0);
	offsetCount = 0;
	for (int i = 0; i < needToOffsetCount; ++i) {
		//黑色图形白底
		auto offsetX = 0, offsetY = 0;
		if (isForward) {
			offsetX = bottomP.x - i;
			offsetY = bottomP.y + i;
		}
		else {
			offsetX = bottomP.x + i;
			offsetY = bottomP.y + i;
		}

		if (offsetX >= 0 && offsetX < img.cols - 1 &&
			offsetY >= 0 && offsetY < img.rows - 1) {
			if (img.at<uchar>(offsetY, offsetX) == locateValue) {
				++offsetCount;
			}
		}
	}
	//printf("\n bottom offsetCount %d\n", offsetCount);
	if (isForward) {
		bottomP.x -= offsetCount;
	}
	else {
		bottomP.x += offsetCount;
	}
	vec.push_back(bottomP);
	return true;
}

bool sfr::Algorithm::getCrossLineCenter(int index, const cv::Mat& source)
{
	m_area[index]._time = cv::getTickCount() / cv::getTickFrequency() * 1000;
	cv::Mat src = source(m_area[index]._rect);

	if (m_area[index].locateType == sfr::SEARCH_AREA_CENTER_FIXED_POSTION) {
		m_area[index]._point0 = cv::Point(src.cols / 2, src.rows / 2);
		return true;
	}

	//图形是黑色并且为白底则为true,图形是白色并且为黑底则为false
	auto locateType = m_area[index].locateType;
	cv::cvtColor(src, src, CV_BGR2GRAY);

	auto thresholdType = 0, denoiseType = 0;
	if (locateType == sfr::BLACK_SECTOR_WITH_WHITE_BACKGROUND ||
		locateType == sfr::BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND) {
		thresholdType = cv::THRESH_BINARY;
		denoiseType = 0;//消除黑色噪点
	}
	else if (locateType == sfr::WHITE_SECTOR_WITH_BLACK_BACKGROUND ||
		locateType == sfr::WHITE_TRAPEZOID_WITH_BLACK_BACKGROUND) {
		thresholdType = cv::THRESH_BINARY_INV;
		denoiseType = 0xff;//消除白色噪点
	}
	else {
		return false;
	}

	src.convertTo(src, -1, m_area[index].contrast, m_area[index].brightness);
	cv::GaussianBlur(src, src, cv::Size(5, 5), 2);
	cv::threshold(src, src, m_area[index].threshold, 255, thresholdType);

	int morph_size = 2;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
		cv::Point(morph_size, morph_size));
	cv::morphologyEx(src, src, cv::MORPH_OPEN, element);
	cv::morphologyEx(src, src, cv::MORPH_CLOSE, element);

	// 定义锐化卷积核
	cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
		0, -1, 0,
		-1, 5, -1,
		0, -1, 0);

	// 使用卷积核进行锐化
	cv::filter2D(src, src, src.depth(), kernel);

	//cv::morphologyEx(src, src, cv::MORPH_GRADIENT, element);
	//cv::medianBlur(src, src, 7);
	cv::Mat thr = src.clone();
#ifdef _DEBUG
	std::string name;
	switch (index)
	{
	case 0:
		name = "center";
		break;
	case 1:
		name = "left_top";
		break;
	case 2:
		name = "right_top";
		break;
	case 3:
		name = "left_bottom";
		break;
	case 4:
		name = "right_bottom";
		break;
	default:
		break;
	}
	cv::imshow(name, src);
	cv::waitKey(0);
#endif // _DEBUG

	int sum = (int)cv::sum(src)[0];
	if (sum == 0 || sum == src.total() * 255) {
		//printf("too light or too dark\n");
		return false;//图像光线太暗或太亮
	}

	std::vector<cv::Point> pixelVec;
	for (int y = 0; y < src.rows; ++y) {
		for (int x = 0; x < src.cols; ++x) {
			if (src.at<uchar>(y, x) == denoiseType) {
				pixelVec.push_back(cv::Point(x, y));
			}
		}
	}

	/* 边缘N个像素内消除噪点 */
	const int edge = m_area[index].denoisePixel;
	for (auto iter = pixelVec.begin(); iter != pixelVec.end(); ++iter) {
		for (int pixmap = 0; pixmap < 5; ++pixmap) {
			bool changeValue = false;
			if ((src.at<uchar>(iter->y, iter->x) == denoiseType) &&
				(((iter->x + pixmap < src.cols) && (src.at<uchar>(iter->y, iter->x + pixmap) != denoiseType)) &&
					((iter->x - pixmap >= 0) && (src.at<uchar>(iter->y, iter->x - pixmap) != denoiseType)))) {
				changeValue = true;
			}

			if ((src.at<uchar>(iter->y, iter->x) == 0) &&
				((iter->y + pixmap < src.rows) && (src.at<uchar>(iter->y + pixmap, iter->x) != 0)) &&
				((iter->y - pixmap >= 0) && (src.at<uchar>(iter->y - pixmap, iter->x) != 0))) {
				changeValue = true;
			}

			if (iter->x < edge || iter->x + edge > src.cols) {
				changeValue = true;
			}

			if (iter->y < edge || iter->y + edge > src.rows) {
				changeValue = true;
			}

			if (changeValue) {
				if (locateType == sfr::BLACK_SECTOR_WITH_WHITE_BACKGROUND ||
					locateType == sfr::BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND) {
					src.at<uchar>(iter->y, iter->x) = 255;
				}
				else if (locateType == sfr::WHITE_SECTOR_WITH_BLACK_BACKGROUND ||
					locateType == sfr::WHITE_TRAPEZOID_WITH_BLACK_BACKGROUND) {
					src.at<uchar>(iter->y, iter->x) = 0;
				}
			}
		}
	}

	for (int i = 0; i < src.rows; ++i) {
		for (int j = 0; j < src.cols; ++j) {
			if (locateType == sfr::BLACK_SECTOR_WITH_WHITE_BACKGROUND ||
				locateType == sfr::BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND) {
				if (src.at<uchar>(i, j) != 0) {
					src.at<uchar>(i, j) = 0xff;
				}
			}
			else if (locateType == sfr::WHITE_SECTOR_WITH_BLACK_BACKGROUND ||
				locateType == sfr::WHITE_TRAPEZOID_WITH_BLACK_BACKGROUND) {
				if (src.at<uchar>(i, j) != 0xff) {
					src.at<uchar>(i, j) = 0;
				}
			}
		}
	}

	m_area[index]._mutex.lock();
	if (m_area[index]._grab) {
		m_area[index]._grab(index, src);
	}
	m_area[index]._mutex.unlock();

	if (locateType == sfr::BLACK_SECTOR_WITH_WHITE_BACKGROUND ||
		locateType == sfr::BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND) {
		cv::Canny(src, src, 100, 250);
	}

	std::vector<cv::Point2i> vec, coord;
	for (int y = 0; y < src.rows; ++y) {
		for (int x = 0; x < src.cols; ++x) {
			if (src.at<uchar>(y, x) == 0xff) {
				coord.push_back(cv::Point(x, y));
			}
		}
	}

	bool result = true;
	if (locateType == sfr::BLACK_SECTOR_WITH_WHITE_BACKGROUND ||
		locateType == sfr::WHITE_SECTOR_WITH_BLACK_BACKGROUND) {
		if (!findSectorCrossLine(coord, vec)) {
			return false;
		}
		result = getCrossPoint(vec[0], vec[1], vec[2], vec[3], m_area[index]._point0);
	}
	else if (locateType == sfr::BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND ||
		locateType == sfr::WHITE_TRAPEZOID_WITH_BLACK_BACKGROUND) {
		if (!findTrapezoidCrossLine(thr, coord, vec, locateType)) {
			return false;
		}

		//auto tmp = source(m_area[index]._rect);
		//cv::circle(tmp, vec[0], 5, CV_RGB(255, 0, 0), 3);
		//cv::circle(tmp, vec[1], 5, CV_RGB(255, 0, 0), 3);

		cv::Point2i p2i;
		p2i.x = (vec[0].x + vec[1].x) / 2;
		p2i.y = (vec[0].y + vec[1].y) / 2;
		m_area[index]._point0 = p2i;
	}

	return result;
}

bool sfr::Algorithm::calculateRoi(int index, float threshold)
{
	auto& area = m_area[index];

	cv::Point2f&& p = area._point1 - area._point0;
	if (std::abs<float>(p.x) > threshold || std::abs<float>(p.y) > threshold)
		area._point1 = area._point0;
	else
		area._point0 = area._point1;

	cv::Rect rect(int(area._point1.x + area.roi.xOffset),
		int(area._point1.y + area.roi.yOffset),
		area.roi.width, area.roi.height);
	area._roi = rect;

	area._roiOk = (area._roi.x > 0 && area._roi.y > 0) &&
		((int)area._point1.x < area.width) &&
		((int)area._point1.y < area.height);
	return area._roiOk;
}

bool sfr::Algorithm::calculateSfr(int index, const cv::Mat& source)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	cv::Mat mat = source(m_area[index]._rect)(m_area[index]._roi);
	m_area[index]._result = calculatesfr(mat, m_area[index]._value, &m_area[index]._curve);
	return m_area[index]._result;
}

void sfr::Algorithm::putText(int index, cv::Mat& source)
{
	if (m_paint) {
		putTextCustom(index, source);
	}
	else {
		putTextDefault(index, source);
	}
}

bool sfr::Algorithm::isPass() const
{
	int sum = 0;
	for (int i = 0; i < std::min<int>(sfr::Area::_size, sfr::MAX_AREA_SIZE); ++i)
	{
		if (m_area[i]._value >= (i ? m_data->circum : m_data->center))
			sum++;
	}
	return sum == sfr::Area::_size;
}

cv::Rect sfr::Algorithm::roi(int index) const
{
	return m_area[index]._roi;
}

double sfr::Algorithm::value(int index)
{
	std::lock_guard<std::mutex> guard(m_mutex);
	return m_area[index]._value;
}

bool sfr::Algorithm::result(int index)
{
	std::lock_guard<std::mutex> guard(m_mutex);
	return m_area[index]._result;
}

std::map<double, double> sfr::Algorithm::curve(int index)
{
	std::lock_guard<std::mutex> guard(m_mutex);
	return m_area[index]._curve;
}

void sfr::Algorithm::locateCenter(cv::Mat& img, const cv::Scalar& color, int thickness) const
{
	auto&& full = img.size();
	auto&& half = img.size() / 2;
	cv::Point line1S(0, half.height), line1E(full.width, half.height);
	cv::Point line2S(half.width, 0), line2E(half.width, full.height);

	drawDottedLine(img, line1S, line1E, color, thickness);
	drawDottedLine(img, line2S, line2E, color, thickness);
}

//int sfr::Algorithm::calculateFps()
//{
//	static int fps = 0;
//	static double last = cv::getTickCount() / cv::getTickFrequency() * 1000;
//	static int frame = 0;
//	++frame;
//	double curr = cv::getTickCount() / cv::getTickFrequency() * 1000;
//	if (curr - last > 1000.0f)
//	{
//		fps = frame;
//		frame = 0;
//		last = curr;
//	}
//	return fps;
//}

/*
* @brief 画FPS
* @param[in] img 图像
* @param[in] fontScale 字体大小
* @param[in] thickness 字体粗细
* @param[in] color 字体颜色
* @param[in] point 坐标点
* @return void
*/
//void drawFps(cv::Mat& img, double fontScale = 2, int thickness = 2,
//	const cv::Scalar& color = CV_RGB(0, 255, 0),
//	cv::Point point = cv::Point(-1, -1));
//void sfr::Algorithm::drawFps(cv::Mat& img, double fontScale, int thickness, const cv::Scalar& color, cv::Point point)
//{
//	int fps = calculateFps();
//	char data[32] = { 0 };
//	sprintf_s(data, "FPS:%02d/s", fps);
//	int baseLine = 0;
//	cv::Size&& size = cv::getTextSize(data, 1, fontScale, thickness, &baseLine);
//	cv::Point p;
//	if (point.x != -1 && point.y != -1)
//	{
//		p = point;
//	}
//	else
//	{
//		p.x = img.cols - size.width;
//		p.y = size.height + baseLine;
//	}
//	cv::putText(img, data, p, 1, fontScale, color, thickness);
//}

void sfr::Algorithm::drawFov(cv::Mat& img, const cv::Scalar& color, int thickness)
{
	double a = (double)img.cols / 2.0f;
	double b = (double)img.rows / 2.0f;
	double c = std::sqrt(std::pow(a, 2) + std::pow(b, 2));
	c *= (m_data->fovp / 100.0f);
	for (int i = 0; i < 60; i++)
	{
		cv::ellipse(img, cv::Point((int)a, (int)b), cv::Size((int)c, (int)c), 90, i * 6, i * 6 + 5, color);
	}
}

bool sfr::Algorithm::calculatesfr(const cv::Mat& area, double& value,
	std::map<double, double>* curve)
{
	value = 0;
	std::map<double, double> map;
	cv::Mat mat = area.clone();
	cv::cvtColor(mat, mat, CV_BGR2GRAY);
	mat.convertTo(mat, CV_64FC1, 1.0 / 255.0);

	double* freq = nullptr, * sfr = nullptr;
	int size = 0, cols = mat.cols, rows = mat.rows;
	int cycles = 0, peak = 0;
	double slope = 0, offset = 0.0, r2 = 0.0;

	int version = 0, iterate = 1;
	if (sfr_proc(&freq, &sfr, &size, (double*)mat.data, cols, &rows,
		&slope, &cycles, &peak, &offset, &r2, version, iterate))
	{
		return false;
	}
	//printf("slope %.3lf angle %.3lf, offset %.3lf, r2 %.3lf\n", slope, std::atan(slope) * 180 / CV_PI, offset, r2);
	for (int i = 0; i < size; ++i)
	{
		map.insert(std::make_pair(freq[i], sfr[i]));
	}

	if (curve != nullptr)
	{
		*curve = map;
	}

	if (freq)
	{
		free(freq);
	}

	if (sfr)
	{
		free(sfr);
	}

	bool find = false;
	for (const auto& x : map)
	{
		if (abs(x.first - m_data->frequency) <= 0.000001)
		{
			value = x.second * 100;
			find = true;
			break;
		}
	}
	return find;
}

bool sfr::Algorithm::getCrossPoint(const cv::Point2i& line1S, const cv::Point2i& line1E, const cv::Point2i& line2S, const cv::Point2i& line2E, cv::Point2f& value) const
{
	cv::Point2f& pt = value;
	// line1's cpmponent
	double X1 = line1E.x - line1S.x;//b1
	double Y1 = line1E.y - line1S.y;//a1
	// line2's cpmponent
	double X2 = line2E.x - line2S.x;//b2
	double Y2 = line2E.y - line2S.y;//a2
	// distance of 1,2
	double X21 = line2S.x - line1S.x;
	double Y21 = line2S.y - line1S.y;
	// determinant
	double D = Y1 * X2 - Y2 * X1;// a1b2-a2b1
	// 
	if (fabs(D - 0) <= 0.000001)
		return false;
	// cross point
	pt.x = float((X1 * X2 * Y21 + Y1 * X2 * line1S.x - Y2 * X1 * line2S.x) / D);
	// on screen y is down increased ! 
	pt.y = -float((Y1 * Y2 * X21 + X1 * Y2 * line1S.y - X2 * Y1 * line2S.y) / D);
	// segments intersect.
	if ((abs(pt.x - line1S.x - X1 / 2) <= abs(X1 / 2)) &&
		(abs(pt.y - line1S.y - Y1 / 2) <= abs(Y1 / 2)) &&
		(abs(pt.x - line2S.x - X2 / 2) <= abs(X2 / 2)) &&
		(abs(pt.y - line2S.y - Y2 / 2) <= abs(Y2 / 2)))
	{
		return !((abs(pt.x - 0) <= 0.000001) && (abs(pt.y - 0) <= 0.000001));
	}
	return false;
}

void sfr::Algorithm::drawPointLine(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, const cv::Scalar& color, int thickness) const
{
	float n = 15; //虚点间隔
	float w = p2.x - p1.x, h = p2.y - p1.y;
	float l = sqrtf(w * w + h * h);
	int m = int(l / n);
	n = l / m; // 矫正虚点间隔，使虚点数为整数

	cv::circle(img, p1, 1, color, thickness); // 画起点
	cv::circle(img, p2, 1, color, thickness); // 画终点
	// 画中间点
	if (p1.y == p2.y) // 水平线：y = m
	{
		float x1 = cv::min<float>(p1.x, p2.x);
		float x2 = cv::max<float>(p1.x, p2.x);
		for (float x = x1 + n; x < x2; x = x + n)
			cv::circle(img, cv::Point2f(x, p1.y), 1, color, thickness);
	}
	else if (p1.x == p2.x) // 垂直线, x = m
	{
		float y1 = cv::min<float>(p1.y, p2.y);
		float y2 = cv::max<float>(p1.y, p2.y);
		for (float y = y1 + n; y < y2; y = y + n)
			cv::circle(img, cv::Point2f(p1.x, y), 1, color, thickness);
	}
	else // 倾斜线，与x轴、y轴都不垂直或平行
	{
		// 直线方程的两点式：(y-y1)/(y2-y1)=(x-x1)/(x2-x1) -> y = (y2-y1)*(x-x1)/(x2-x1)+y1
		float m = n * abs(w) / l;
		float k = h / w;
		float x1 = cv::min<float>(p1.x, p2.x);
		float x2 = cv::max<float>(p1.x, p2.x);
		for (float x = x1 + m; x < x2; x = x + m)
			cv::circle(img, cv::Point2f(x, k * (x - p1.x) + p1.y), 1, color, thickness);
	}
	return;
}

void sfr::Algorithm::drawDottedLine(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, const cv::Scalar& color, int thickness) const
{
	float n = 15; //线长度
	float w = p2.x - p1.x, h = p2.y - p1.y;
	float l = sqrtf(w * w + h * h);
	// 矫正线长度，使线个数为奇数
	int m = int(l / n);
	m = m % 2 ? m : m + 1;
	n = l / m;

	cv::circle(img, p1, 1, color, thickness); // 画起点
	cv::circle(img, p2, 1, color, thickness); // 画终点
	// 画中间点
	if (p1.y == p2.y) //水平线：y = m
	{
		float x1 = cv::min<float>(p1.x, p2.x);
		float x2 = cv::max<float>(p1.x, p2.x);
		for (float x = x1, n1 = 2 * n; x < x2; x = x + n1)
			line(img, cv::Point2f(x, p1.y), cv::Point2f(x + n, p1.y), color, thickness);
	}
	else if (p1.x == p2.x) //垂直线, x = m
	{
		float y1 = cv::min<float>(p1.y, p2.y);
		float y2 = cv::max<float>(p1.y, p2.y);
		for (float y = y1, n1 = 2 * n; y < y2; y = y + n1)
			cv::line(img, cv::Point2f(p1.x, y), cv::Point2f(p1.x, y + n), color, thickness);
	}
	else // 倾斜线，与x轴、y轴都不垂直或平行
	{
		// 直线方程的两点式：(y-y1)/(y2-y1)=(x-x1)/(x2-x1) -> y = (y2-y1)*(x-x1)/(x2-x1)+y1
		float n1 = n * abs(w) / l;
		float k = h / w;
		float x1 = cv::min<float>(p1.x, p2.x);
		float x2 = cv::max<float>(p1.x, p2.x);
		for (float x = x1, n2 = 2 * n1; x < x2; x = x + n2)
		{
			cv::Point p3 = cv::Point2f(x, k * (x - p1.x) + p1.y);
			cv::Point p4 = cv::Point2f(x + n1, k * (x + n1 - p1.x) + p1.y);
			cv::line(img, p3, p4, color, thickness);
		}
	}
	return;
}

void sfr::Algorithm::putTextDefault(int index, cv::Mat& source)
{
	auto& area = m_area[index];
	cv::Mat roi = source(area._rect);

	//画ROI矩形框
	cv::rectangle(roi, cv::Rect(0, 0, roi.cols, roi.rows), CV_RGB(255, 255, 0), 2);

	//画编号
	cv::String&& number = "#" + std::to_string(index);
	int baseLine = 0;
	cv::Size&& size = cv::getTextSize(number, 1, 1.5, 2, &baseLine);
	cv::putText(roi, number, cv::Point(0, size.height + baseLine), 1, 1.5, CV_RGB(0, 255, 255), 2);

	if (area._roiOk)
	{
		area._roiOk = false;

		//画坐标
		char coordinate[32] = { 0 };
		sprintf_s(coordinate, "(%d,%d)", (int)area._point1.x + area.x, (int)area._point1.y + area.y);
		cv::putText(roi, coordinate, cv::Point(size.width, size.height + baseLine), 1, 1.5, CV_RGB(0, 255, 255), 2);

		//画中心点+
		cv::Point2f xLineS(area._point1.x + 10, area._point1.y);
		cv::Point2f xLineE(area._point1.x - 10, area._point1.y);
		cv::line(roi, xLineS, xLineE, CV_RGB(255, 0, 0), 2);

		cv::Point2f yLineS(area._point1.x, area._point1.y + 10);
		cv::Point2f yLineE(area._point1.x, area._point1.y - 10);
		cv::line(roi, yLineS, yLineE, CV_RGB(255, 0, 0), 2);

		//画SFR矩形框
		cv::rectangle(roi, area._roi, CV_RGB(255, 0, 0), 1);

		//画数值
		cv::Point p(area._roi.x + area._roi.width + 2, area._roi.y + area._roi.height / 2);

		cv::Scalar&& color = (area._value >= (index ? m_data->circum : m_data->center)) ?
			CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0);

		char value[32] = { 0 };
		sprintf_s(value, "%.2lf", area._value);
		cv::putText(roi, cv::String(area._value ? value : "N/A"), p, 1, 1.5, color, 2);

		//画耗时时间
		{
			m_area[index]._time = cv::getTickCount() / cv::getTickFrequency() * 1000 - m_area[index]._time;
			int baseLine = 0;
			char time[32] = { 0 };
			sprintf_s(time, "%.2lf/ms", m_area[index]._time);
			cv::String text(time);
			cv::Size&& size = cv::getTextSize(text, 1, 1.5, 2, &baseLine);
			cv::Point point(roi.cols - size.width, baseLine + size.height);
			cv::putText(roi, text, point, 1, 1.5, CV_RGB(0, 255, 255), 2);
		}
	}
	else
	{
		//画中心点+到左下脚
		cv::Point xLineS(area._rect.x + 10, area._rect.y + area._rect.height);
		cv::Point xLineE(area._rect.x - 10, area._rect.y + area._rect.height);
		cv::line(source, xLineS, xLineE, CV_RGB(255, 0, 0), 2);

		cv::Point yLineS(area._rect.x, area._rect.y + area._rect.height + 10);
		cv::Point yLineE(area._rect.x, area._rect.y + area._rect.height - 10);
		cv::line(source, yLineS, yLineE, CV_RGB(255, 0, 0), 2);
	}

	//始终等于最后一个
	if (index == sfr::Area::_size - 1)
	{
		//定位中心
		if (m_enable->drawLocateCenter)
		{
			locateCenter(source);
		}

		//画结果PASS
		if (m_enable->drawResultPass)
		{
			cv::String text;
			cv::Scalar color;
			if (isPass()) {
				text = "PASS";
				color = CV_RGB(0, 255, 0);
			}
			else {
				text = "NOT PASS";
				color = CV_RGB(255, 0, 0);
			}
			int baseLine = 0;
			cv::Size&& size = cv::getTextSize(text, 1, 3, 3, &baseLine);
			cv::putText(source, text, cv::Point(0, size.height + baseLine), 1, 3, color, 3);
		}

		//画FPS
		//if (m_enable->drawFps)
		//{
		//	drawFps(source);
		//}

		//画视场角
		if (m_enable->drawFovp)
		{
			drawFov(source);
		}
	}
}

void sfr::Algorithm::putTextCustom(int index, cv::Mat& source)
{
	auto& area = m_area[index];
	cv::Mat roi = source(area._rect);

	//画区域矩形框
	cv::rectangle(roi, cv::Rect(0, 0, roi.cols, roi.rows), m_paint->areaRectColor, m_paint->areaRectThickness);

	//画编号
	cv::String&& number = "#" + std::to_string(index);
	int baseLine = 0;
	cv::Size&& size = cv::getTextSize(number, 1, m_paint->textScale, m_paint->textThickness, &baseLine);
	cv::putText(roi, number, cv::Point(0, size.height + baseLine), 1,
		m_paint->textScale, m_paint->textColor, m_paint->textThickness);

	if (area._roiOk) {
		area._roiOk = false;

		//画坐标
		char coordinate[32] = { 0 };
		sprintf_s(coordinate, "(%d,%d)", (int)area._point1.x + area.x, (int)area._point1.y + area.y);
		cv::putText(roi, coordinate, cv::Point(size.width, size.height + baseLine), 1,
			m_paint->textScale, m_paint->textColor, m_paint->textThickness);

		//画中心点+
		cv::Point2f xLineS(area._point1.x + m_paint->locateLineLength, area._point1.y);
		cv::Point2f xLineE(area._point1.x - m_paint->locateLineLength, area._point1.y);
		cv::line(roi, xLineS, xLineE, m_paint->locateLineColor, m_paint->locateLineThickness);

		cv::Point2f yLineS(area._point1.x, area._point1.y + m_paint->locateLineLength);
		cv::Point2f yLineE(area._point1.x, area._point1.y - m_paint->locateLineLength);
		cv::line(roi, yLineS, yLineE, m_paint->locateLineColor, m_paint->locateLineThickness);

		//画ROI矩形框
		cv::rectangle(roi, area._roi, m_paint->roiRectColor, m_paint->roiRectThickness);

		//画数值
		cv::Point p(area._roi.x + area._roi.width + 2, area._roi.y + area._roi.height / 2);

		cv::Scalar&& color = (area._value >= (index ? m_data->circum : m_data->center)) ?
			CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0);

		char value[32] = { 0 };
		sprintf_s(value, "%.2lf", area._value);
		cv::putText(roi, cv::String(area._value ? value : "N/A"), p, 1, m_paint->textScale, color, m_paint->textThickness);

		//画耗时时间
		{
			m_area[index]._time = cv::getTickCount() / cv::getTickFrequency() * 1000 - m_area[index]._time;
			int baseLine = 0;
			char time[32] = { 0 };
			sprintf_s(time, "%.0lf/ms", m_area[index]._time);
			cv::String text(time);
			cv::Size&& size = cv::getTextSize(text, 1, m_paint->textScale, m_paint->textThickness, &baseLine);
			cv::Point point(roi.cols - size.width, baseLine + size.height);
			cv::putText(roi, text, point, 1, m_paint->textScale, m_paint->textColor, m_paint->textThickness);
		}
	}
	else {
		//画中心点+到左下脚
		cv::Point xLineS(area._rect.x + m_paint->locateLineLength, area._rect.y + area._rect.height);
		cv::Point xLineE(area._rect.x - m_paint->locateLineLength, area._rect.y + area._rect.height);
		cv::line(source, xLineS, xLineE, m_paint->locateLineColor, m_paint->locateLineThickness);

		cv::Point yLineS(area._rect.x, area._rect.y + area._rect.height + m_paint->locateLineLength);
		cv::Point yLineE(area._rect.x, area._rect.y + area._rect.height - m_paint->locateLineLength);
		cv::line(source, yLineS, yLineE, m_paint->locateLineColor, m_paint->locateLineThickness);
	}

	//始终等于最后一个
	if (index == sfr::Area::_size - 1) {
		//定位中心
		if (m_enable->drawLocateCenter) {
			locateCenter(source, m_paint->centerLineColor, m_paint->centerLineThickness);
		}

		//画结果PASS
		if (m_enable->drawResultPass) {
			cv::String text;
			cv::Scalar color;
			if (isPass()) {
				text = "PASS";
				color = CV_RGB(0, 255, 0);
			}
			else {
				text = "NOT PASS";
				color = CV_RGB(255, 0, 0);
			}
			int baseLine = 0;
			double fontScale = m_paint->textScale + 3;
			int thickness = m_paint->textThickness + 3;
			cv::Size&& size = cv::getTextSize(text, 1, fontScale, thickness, &baseLine);
			cv::putText(source, text, cv::Point(0, size.height + baseLine), 1, fontScale, color, thickness);
		}

		//画FPS
		//if (m_enable->drawFps) {
		//	drawFps(source, m_paint->textScale + 2, m_paint->textThickness + 2);
		//}

		//画视场角
		if (m_enable->drawFovp) {
			drawFov(source, m_paint->fovLineColor, m_paint->fovLineThickness);
		}
	}
}

