// KinectGrabber is pcl::Grabber to retrieve the point cloud data from Kinect v1 using Kinect for Windows SDK v1.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.
/*
//類別名稱：KinectGrabber(繼承pcl::Grabber)
//作者：杉浦 司 (Tsukasa Sugiura)
//日期：2016/08/15
//目的：Kinect Sdk以及PCL接口
//使用函式庫：PCL, Kinect sdk 1.8
*/
#ifndef KINECT_GRABBER
#define KINECT_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <NuiApi.h>
#undef max
#undef min
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZI;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZRGBA;
	template <typename T> class pcl::PointCloud;

	class KinectGrabber : public pcl::Grabber
	{
		public:
			KinectGrabber( const int index = 0 );
			virtual ~KinectGrabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

			typedef void ( signal_Kinect_PointXYZ )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
			typedef void ( signal_Kinect_PointXYZI )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& );
			typedef void ( signal_Kinect_PointXYZRGB )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );
			typedef void ( signal_Kinect_PointXYZRGBA )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>& );

		protected:
			boost::signals2::signal<signal_Kinect_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect_PointXYZI>* signal_PointXYZI;
			boost::signals2::signal<signal_Kinect_PointXYZRGB>* signal_PointXYZRGB;
			boost::signals2::signal<signal_Kinect_PointXYZRGBA>* signal_PointXYZRGBA;

			pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( NUI_LOCKED_RECT* depthLockedRect );
			pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI( NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB( NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect );
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA( NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect );

			boost::thread thread;
			mutable boost::mutex mutex;

			void threadFunction();

			bool quit;
			bool running;

			HRESULT result;
			INuiSensor* sensor;
			INuiCoordinateMapper* mapper;
			HANDLE colorHandle;
			HANDLE depthHandle;

			int width;
			int height;
	};
}

#endif KINECT_GRABBER

