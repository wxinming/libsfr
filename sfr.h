#pragma once

#include <map>
#include <mutex>
#include <functional>

#include <OpenCv/OpenCv.h>

#if defined(LIBSFR_NOT_EXPORTS)
#define SFR_DLL_EXPORT
#else
#if defined(LIBSFR_EXPORTS)
#define SFR_DLL_EXPORT __declspec(dllexport)
#else
#define SFR_DLL_EXPORT __declspec(dllimport)
#endif // LIBSFR_EXPORTS
#pragma warning(push)
#pragma warning(disable:4251)
#define SFR_DISABLE_WARNING
#endif // LIBSFR_NOT_EXPORTS

/*
* @brief 空间响应频率算法类库
* SFR[Spatial Frequency Response]
* 计算步骤
* 0.获取垂直斜边的ROI
* 1.进行数据的归一化
* 2.计算图像每一行的像素矩心
* 3.对每行的矩心使用最小二乘法进行线性拟合,获得一条关于矩心的直线
* 4.重新定位ROI,获得ESF
* 5.对获得的ESF进行四倍超采样
* 6.通过差分运算获得LSF
* 7.对LSF应用汉明窗
* 8.进行DFT运算
*/

namespace sfr {

	//测试数据
	struct SFR_DLL_EXPORT Data {
		//构造
		Data();

		//析构
		~Data();

		//SFR频率
		double frequency;

		//SFR中心值
		double center;

		//SFR周边值
		double circum;

		//计算SFR间隔(ms)
		double interval;

		//视场角百分比
		double fovp;
	};

	//启用
	struct SFR_DLL_EXPORT Enable {
		//构造
		Enable();

		//析构
		~Enable();

		//绘制定位中心
		bool drawLocateCenter;

		//绘制结果PASS
		bool drawResultPass;

		//绘制视场角
		bool drawFovp;
	};

	//频率
	struct SFR_DLL_EXPORT Frequency {
		//中心
		double center;

		//左上
		double leftTop;

		//右上
		double rightTop;

		//左下
		double leftBottom;

		//右下
		double rightBottom;

		//运动位置
		long motionPosition;

		double& operator*();

		double* _ptr0 = nullptr;

		double* _ptr1 = nullptr;

		//总数
		double total() const;
	};

	//定位类型,目前仅支持两种定位算法
	enum LocateType  {
		//黑色扇形&白色背景
		BLACK_SECTOR_WITH_WHITE_BACKGROUND,

		//白色扇形&黑色背景
		WHITE_SECTOR_WITH_BLACK_BACKGROUND,

		//黑色梯形&白色背景
		BLACK_TRAPEZOID_WITH_WHITE_BACKGROUND,

		//白色梯形&黑色背景
		WHITE_TRAPEZOID_WITH_BLACK_BACKGROUND,

		//搜索区域中心固定位置
		SEARCH_AREA_CENTER_FIXED_POSTION,
	};

	//最大区域 中心,左上,右上,左下,右下
	static const int MAX_AREA_SIZE = 5;

	//区域
	struct SFR_DLL_EXPORT Area {
		//构造
		Area();

		//析构
		~Area();

		//此区域的x坐标
		int x;

		//此区域的y坐标
		int y;

		//此区域的宽度
		int width;

		//此区域的高度
		int height;

		//感兴趣的区域
		struct {
			//roi的宽度
			int width;

			//roi的高度
			int height;

			//x坐标偏移
			int xOffset;

			//y坐标偏移
			int yOffset;
		} roi;//SFR的ROI

		//定位类型
		int locateType;

		//边缘降噪多少个像素点
		int denoisePixel;

		//此区域的二值化阈值
		double threshold;

		//对比度
		double contrast;

		//亮度
		int brightness;

		double _value = 0;

		bool _result = false;

		double _tick = 0;

		double _time = 0;

		cv::Point2f _point0;

		cv::Point2f _point1;

		cv::Rect _rect;

		cv::Rect _roi;

		bool _roiOk = false;

		std::map<double, double> _curve;

		std::mutex _mutex;

		std::function<void(int index, const cv::Mat& mat)> _grab = nullptr;

		static int _size;

		/*
		* @brief 开始抓取此区域
		* @param[in] index 区域索引
		* @param[in] mat cv::Mat
		* @return void
		*/
		void startGrab(const std::function<void(int index, const cv::Mat& mat)>& func);

		/*
		* @brief 停止抓取此区域
		* @return void
		*/
		void stopGrab();
	};

	//位置
	enum Position {
		//中心区域
		CENTER,

		//左上区域
		LEFT_TOP,

		//右上区域
		RIGHT_TOP,

		//左下区域
		LEFT_BOTTOM,

		//右下区域
		RIGHT_BOTTOM
	};

	//绘图
	struct SFR_DLL_EXPORT Paint {
		//构造
		Paint();

		//析构
		~Paint();

		//文本缩放
		double textScale;

		//文本颜色,不涉及结果颜色
		cv::Scalar textColor;

		//文本粗细
		int textThickness;

		//区域矩形框颜色
		cv::Scalar areaRectColor;

		//区域矩形框粗细
		int areaRectThickness;

		//ROI矩形框颜色
		cv::Scalar roiRectColor;

		//ROI矩形框粗细
		int roiRectThickness;

		//定位线条颜色
		cv::Scalar locateLineColor;

		//定位线条的粗细
		int locateLineThickness;

		//定位线条的长度
		int locateLineLength;

		//中心线条颜色
		cv::Scalar centerLineColor;

		//中心线条的粗细
		int centerLineThickness;

		//视场角线条颜色
		cv::Scalar fovLineColor;

		//视场角线条粗细
		int fovLineThickness;
	};

	class SFR_DLL_EXPORT Algorithm {
	public:
		/*
		* @brief 构造
		*/
		Algorithm();

		/*
		* @brief 构造
		* @param[in] area 区域
		* @param[in] data 数据
		* @param[in] enable 启用
		* @param[in] paint 绘图
		*/
		Algorithm(sfr::Area* area, sfr::Data* data, sfr::Enable* enable, sfr::Paint* paint = nullptr);

		/*
		* @brief 析构
		*/
		~Algorithm();

		/*
		* @brief 初始化
		* @param[in] area 区域
		* @param[in] data 数据
		* @param[in] enable 启用
		* @param[in] paint 绘图
		* @return void
		*/
		void initialize(sfr::Area* area, sfr::Data* data, sfr::Enable* enable, sfr::Paint* paint = nullptr);

		/*
		* @brief 是否计算[指定区域]
		* @param[in] index 区域索引
		* @return bool
		*/
		bool isCalculate(int index);

		/*
		* @brief 获取交叉线中心
		* @param[in] index 区域索引
		* @param[in] source 图像源(整个图像)
		* @return bool
		*/
		bool getCrossLineCenter(int index, const cv::Mat& source);

		/*
		* @brief 计算SFR的ROI
		* @param[in] index 区域索引
		* @param[in] thresold 误差阈值(允许与上次获取的坐标相差+-threshold,则返回上次的坐标点)
		* @return bool
		*/
		bool calculateRoi(int index, float threshold = 2.0f);

		/*
		* @brief 计算SFR
		* @param[in] index 区域索引
		* @param[in] source 图像源(整个图像)
		* @return bool
		*/
		bool calculateSfr(int index, const cv::Mat& source);

		/*
		* @brief 将数据输出在图像上
		* @param[in] index 区域索引
		* @param[in|out] source 图像源(整个图像)
		* @return void
		*/
		void putText(int index, cv::Mat& source);

		/*
		* @brief 区域是否通过
		* @return bool
		*/
		bool isPass() const;

		/*
		* @brief SFR的ROI[非线程安全]
		* @param[in] index 区域索引
		* @return cv::Rect
		*/
		cv::Rect roi(int index) const;

		/*
		* @brief SFR的值[线程安全]
		* @param[in] index 区域索引
		* @return double
		*/
		double value(int index);

		/*
		* @brief SFR的结果[线程安全]
		* @param[in] index 区域索引
		* @return bool
		*/
		bool result(int index);

		/*
		* @brief MTF曲线[线程安全]
		* @param[in] index 区域索引
		* @return MTF曲线MAP
		*/
		std::map<double, double> curve(int index);

	protected:

		/*
		* @brief 定位中心
		* @param[in|out] img 图像
		* @param[in] color 虚线颜色
		* @param[in] thickness 线条粗细
		* @return void
		*/
		void locateCenter(cv::Mat& img, const cv::Scalar& color = CV_RGB(255, 250, 205), int thickness = 1) const;

		/*
		* @brief 画视场角
		* @param[in] img 图像
		* @param[in] color 虚线颜色
		* @param[in] thickness 线条粗细
		* @return void
		*/
		void drawFov(cv::Mat& img, const cv::Scalar& color = CV_RGB(255, 250, 205), int thickness = 1);

		/*
		* @brief 计算SFR
		* @param[in] area 计算的区域
		* @param[out] value SFR的值
		* @param[out] curve MTF曲线
		* @return bool
		*/
		bool calculatesfr(const cv::Mat& area, double& value,
			std::map<double, double>* curve = nullptr);

		/*
		* @brief 获取交叉点
		* @param[in] line1S 线条1起点
		* @param[in] line1E 线条1终点
		* @param[in] line2S 线条2起点
		* @param[in] line2E 线条2终点
		* @param[in] value 交叉坐标
		* @return bool
		*/
		bool getCrossPoint(const cv::Point2i& line1S, const cv::Point2i& line1E, const cv::Point2i& line2S, const cv::Point2i& line2E, cv::Point2f& value) const;

		/*
		* @brief 画点线
		* @param[in|out] img 图像
		* @param[in] p1 起点
		* @param[in] p2 终点
		* @param[in] color 颜色
		* @param[in] thickness 线条粗细
		* @return void
		*/
		void drawPointLine(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, const cv::Scalar& color, int thickness) const;

		/*
		* @brief 画虚线
		* @param[in|out] mat 图像
		* @param[in] p1 起点
		* @param[in] p2 终点
		* @param[in] color 颜色
		* @param[in] thickness 线条粗细
		* @return void
		*/
		void drawDottedLine(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, const cv::Scalar& color, int thickness) const;

		/*
		* @brief 默认将数据输出在图像上
		* @param[in] index 区域索引
		* @param[in|out] source 图像源(整个图像)
		* @return void
		*/
		void putTextDefault(int index, cv::Mat& source);

		/*
		* @brief 自定义将数据输出在图像上
		* @param[in] index 区域索引
		* @param[in|out] source 图像源(整个图像)
		* @return void
		*/
		void putTextCustom(int index, cv::Mat& source);

	private:
		std::mutex m_mutex;
		sfr::Area* m_area = nullptr;
		sfr::Data* m_data = nullptr;
		sfr::Enable* m_enable = nullptr;
		sfr::Paint* m_paint = nullptr;
	};
}

#ifdef SFR_DISABLE_WARNING
#pragma warning(pop)
#endif

