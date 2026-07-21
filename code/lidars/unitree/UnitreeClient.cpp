#include "lidars/unitree/UnitreeClient.h"

#include <chrono>
#include <cmath>
#include <iostream>

namespace mandeye
{

UnitreeClient::~UnitreeClient()
{
	stopListener();
}

nlohmann::json UnitreeClient::produceStatus()
{
	nlohmann::json data;
	nlohmann::json data_status;

	{
		std::lock_guard<std::mutex> lock(m_statusMutex);
		data_status["init_success"] = m_initSuccess;
		data_status["firmware_version"] = m_firmwareVersion;
		data_status["hardware_version"] = m_hardwareVersion;
		data_status["sdk_version"] = m_sdkVersion;
		data_status["dirty_percentage"] = m_dirtyPercentage;
		data_status["timestamp_sec"] = m_timestamp;
		data_status["time_diff"] = m_time_diff;
		data_status["lidar_ip"] = m_lidarIp;
		data_status["local_ip"] = m_localIp;
	}
	data_status["is_done"] = isDone.load();
	data_status["received_point_messages"] = m_recivedPointMessages.load();
	data_status["received_imu_messages"] = m_recivedIMUMessages.load();

	data["is_synced"] = isSynced();
	data["UnitreeClient"]["status"] = data_status;
	data["counters"]["imu"] = m_recivedIMUMessages.load();
	data["counters"]["lidar"] = m_recivedPointMessages.load();

	return data;
}

bool UnitreeClient::isReadyToScan()
{
	std::lock_guard<std::mutex> lock(m_statusMutex);
	// SDK must have finished init (rotation started, work mode set).
	if(!m_initSuccess)
	{
		return false;
	}
	// Timestamps must be in sync (this also implies point data is flowing,
	// since m_time_diff is only updated when point clouds arrive).
	if(m_time_diff >= 1.0)
	{
		return false;
	}
	// Give the rotation a moment to stabilize after init so the first
	// recorded frames aren't distorted by spin-up.
	const auto elapsed = std::chrono::steady_clock::now() - m_initTime;
	if(elapsed < std::chrono::duration<double>(kWarmupSeconds))
	{
		return false;
	}
	return true;
}

bool UnitreeClient::startListener(const std::string& interfaceIp)
{
	std::cout << "UnitreeClient: startListener called with interfaceIp: " << interfaceIp << std::endl;
	if(!interfaceIp.empty())
	{
		// Use the configured listen interface as the local (host) IP for the UDP socket.
		m_localIp = interfaceIp;
	}
	m_watchThread = std::thread(&UnitreeClient::DataThreadFunction, this);
	return true;
}

void UnitreeClient::stopListener()
{
	isDone.store(true);
	if(m_watchThread.joinable())
	{
		m_watchThread.join();
	}
}

void UnitreeClient::startLog()
{
	std::cout << "UnitreeClient: startLog called" << std::endl;
	std::lock_guard<std::mutex> lock1(m_bufferImuMutex);
	std::lock_guard<std::mutex> lock2(m_bufferPointMutex);
	m_bufferLidarPtr = std::make_shared<LidarPointsBuffer>();
	m_bufferIMUPtr = std::make_shared<LidarIMUBuffer>();
}

void UnitreeClient::stopLog()
{
	std::cout << "UnitreeClient: stopLog called" << std::endl;
	std::lock_guard<std::mutex> lock1(m_bufferImuMutex);
	std::lock_guard<std::mutex> lock2(m_bufferPointMutex);
	m_bufferLidarPtr.reset();
	m_bufferIMUPtr.reset();
}

std::pair<LidarPointsBufferPtr, LidarIMUBufferPtr> UnitreeClient::retrieveData()
{
	std::lock_guard<std::mutex> lock1(m_bufferImuMutex);
	std::lock_guard<std::mutex> lock2(m_bufferPointMutex);

	LidarPointsBufferPtr returnPointerLidar{std::make_shared<LidarPointsBuffer>()};
	LidarIMUBufferPtr returnPointerImu{std::make_shared<LidarIMUBuffer>()};
	std::swap(m_bufferIMUPtr, returnPointerImu);
	std::swap(m_bufferLidarPtr, returnPointerLidar);
	return std::pair<LidarPointsBufferPtr, LidarIMUBufferPtr>(returnPointerLidar, returnPointerImu);
}

void UnitreeClient::DataThreadFunction()
{
	using namespace unilidar_sdk2;

	std::cout << "UnitreeClient: DataThreadFunction started" << std::endl;

	m_lidar = createUnitreeLidarReader();
	// initializeUDP returns 0 on success, non-zero on failure.
	if(m_lidar->initializeUDP(m_lidarPort, m_lidarIp, m_localPort, m_localIp))
	{
		std::cerr << "UnitreeClient: initializeUDP failed (lidar " << m_lidarIp << ":" << m_lidarPort << ", local " << m_localIp << ":"
				  << m_localPort << ")" << std::endl;
		return;
	}

	m_lidar->startLidarRotation();
	std::this_thread::sleep_for(std::chrono::seconds(1));
	m_lidar->setLidarWorkMode(0); // 0: 3D point cloud mode
	std::this_thread::sleep_for(std::chrono::seconds(1));

	{
		std::string sdkVersion;
		if(m_lidar->getVersionOfSDK(sdkVersion))
		{
			std::lock_guard<std::mutex> lock(m_statusMutex);
			m_sdkVersion = sdkVersion;
		}
		std::lock_guard<std::mutex> lock(m_statusMutex);
		m_initSuccess = true;
		m_initTime = std::chrono::steady_clock::now();
	}

	LidarImuData imu;
	PointCloudUnitree cloud;
	while(!isDone.load())
	{
		const int result = m_lidar->runParse();
		switch(result)
		{
		case LIDAR_POINT_DATA_PACKET_TYPE:
			if(m_lidar->getPointCloud(cloud))
			{
				m_recivedPointMessages.fetch_add(1);

				const double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
				{
					std::lock_guard<std::mutex> lock(m_statusMutex);
					m_timestamp = cloud.stamp;
					m_time_diff = std::abs(now - cloud.stamp);
				}

				std::lock_guard<std::mutex> lock(m_bufferPointMutex);
				if(m_bufferLidarPtr)
				{
					for(const auto& point : cloud.points)
					{
						LidarPoint data;
						data.x = point.x;
						data.y = point.y;
						data.z = point.z;
						data.intensity = point.intensity;
						data.tag = 0;
						// point.time is relative (seconds) to cloud.stamp -> absolute nanoseconds
						data.timestamp = static_cast<uint64_t>((cloud.stamp + point.time) * 1e9);
						data.line_id = static_cast<uint8_t>(point.ring);
						data.laser_id = 0;
						m_bufferLidarPtr->push_back(data);
					}
				}
			}
			break;

		case LIDAR_IMU_DATA_PACKET_TYPE:
			if(m_lidar->getImuData(imu))
			{
				m_recivedIMUMessages.fetch_add(1);

				const auto now = std::chrono::system_clock::now().time_since_epoch();
				const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

				std::lock_guard<std::mutex> lock(m_bufferImuMutex);
				if(m_bufferIMUPtr)
				{
					LidarIMU data;
					// angular_velocity is reported in rad/s, linear_acceleration in m/s^2.
					data.gyro_x = imu.angular_velocity[0];
					data.gyro_y = imu.angular_velocity[1];
					data.gyro_z = imu.angular_velocity[2];
					data.acc_x = imu.linear_acceleration[0];
					data.acc_y = imu.linear_acceleration[1];
					data.acc_z = imu.linear_acceleration[2];
					data.timestamp = static_cast<uint64_t>(imu.info.stamp.sec) * 1000000000ull + imu.info.stamp.nsec;
					data.laser_id = 0;
					data.epoch_time = millis;
					m_bufferIMUPtr->push_back(data);
				}
			}
			break;

		case LIDAR_VERSION_PACKET_TYPE:
		{
			std::string firmware;
			std::string hardware;
			std::lock_guard<std::mutex> lock(m_statusMutex);
			if(m_lidar->getVersionOfLidarFirmware(firmware))
			{
				m_firmwareVersion = firmware;
			}
			if(m_lidar->getVersionOfLidarHardware(hardware))
			{
				m_hardwareVersion = hardware;
			}
			float dirty;
			if(m_lidar->getDirtyPercentage(dirty))
			{
				m_dirtyPercentage = dirty;
			}
			break;
		}

		case 0:
			// no valid message available yet, avoid busy spinning
			std::this_thread::sleep_for(std::chrono::microseconds(200));
			break;

		default:
			break;
		}
	}

	m_lidar->stopLidarRotation();
	m_lidar->closeUDP();
	std::cout << "UnitreeClient: DataThreadFunction ended" << std::endl;
}

} // namespace mandeye
