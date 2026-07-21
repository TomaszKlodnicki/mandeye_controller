#pragma once

#include "lidars/BaseLidarClient.h"
#include <atomic>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

// Unitree unilidar_sdk2 (L2)
#include "unitree_lidar_sdk.h"

namespace mandeye
{

//! Client for the Unitree L2 lidar, using the unilidar_sdk2.
//! Unlike the callback-based SDKs, this SDK is poll based: a worker thread
//! repeatedly calls runParse() and dispatches the returned packet type.
class UnitreeClient : public BaseLidarClient
{
public:
	UnitreeClient() = default;
	~UnitreeClient() override;

	nlohmann::json produceStatus() override;

	//! Starts the worker thread. interfaceIp is used as the local (host) IP for the UDP socket.
	bool startListener(const std::string& interfaceIp) override;

	//! Stops the worker thread and closes the connection.
	void stopListener() override;

	//! Start log to memory data from Lidar and IMU
	void startLog() override;

	//! Stops log to memory data from Lidar and IMU
	void stopLog() override;

	std::pair<LidarPointsBufferPtr, LidarIMUBufferPtr> retrieveData() override;

	// TimestampProvider overrides ...
	double getTimestamp() override
	{
		return m_timestamp;
	}
	double getSessionDuration() override
	{
		return 0.0;
	}
	double getSessionStart() override
	{
		return 0.0;
	}
	void initializeDuration() override { }

	bool isSynced() override
	{
		return m_time_diff < 1.0; // lidar reports time close to the computer's timestamp
	}

	//! Ready to scan once the SDK is initialized, time is synced, and the
	//! rotation has had a moment to stabilize (avoids distorted first frames).
	bool isReadyToScan() override;

private:
	void DataThreadFunction();

	std::string m_lidarIp{"192.168.123.110"};
	std::string m_localIp{"192.168.123.120"};
	unsigned short m_lidarPort{6101};
	unsigned short m_localPort{6201};

	unilidar_sdk2::UnitreeLidarReader* m_lidar{nullptr};

	std::mutex m_bufferImuMutex;
	std::mutex m_bufferPointMutex;
	LidarPointsBufferPtr m_bufferLidarPtr;
	LidarIMUBufferPtr m_bufferIMUPtr;

	std::thread m_watchThread;
	std::atomic_bool isDone{false};
	std::atomic_int m_recivedPointMessages{0};
	std::atomic_int m_recivedIMUMessages{0};

	//! Seconds to wait after init before the lidar is considered ready to scan,
	//! letting the rotation stabilize so the first recorded frames aren't distorted.
	static constexpr double kWarmupSeconds{3.0};

	std::mutex m_statusMutex;
	bool m_initSuccess{false};
	std::chrono::steady_clock::time_point m_initTime{}; // set once init completes (guarded by m_statusMutex)
	std::string m_firmwareVersion;
	std::string m_hardwareVersion;
	std::string m_sdkVersion;
	float m_dirtyPercentage{-1.0f};
	double m_timestamp{0.0};
	double m_time_diff{std::numeric_limits<double>::max()};
};

} // namespace mandeye

extern "C" void* create_unitree_client()
{
	return new mandeye::UnitreeClient();
}

extern "C" void destroy_unitree_client(void* ptr)
{
	delete static_cast<mandeye::UnitreeClient*>(ptr);
}
