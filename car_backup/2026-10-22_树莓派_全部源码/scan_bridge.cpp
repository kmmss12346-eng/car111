// Scan-only bridge using the bundled LDROBOT MIT-licensed SDK.
#include "ldlidar_driver.h"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <sstream>
#include <thread>

static volatile std::sig_atomic_t running = 1;
static void stop_handler(int) { running = 0; }
static uint64_t stamp_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: scan_bridge PORT LD14P_2300HZ|LD14P_4000HZ\n";
    return 2;
  }
  std::string model(argv[2]);
  ldlidar::LDType type;
  if (model == "LD14P_2300HZ") type = ldlidar::LDType::LD_14P_2300HZ;
  else if (model == "LD14P_4000HZ") type = ldlidar::LDType::LD_14P_4000HZ;
  else { std::cerr << "Unsupported model\n"; return 2; }
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);
  ldlidar::LDLidarDriver driver;
  driver.RegisterGetTimestampFunctional(stamp_ns);
  // 初次诊断保留通过协议校验的测距点，不启用厂商噪声过滤。
  driver.EnableFilterAlgorithnmProcess(false);
  if (!driver.Start(type, argv[1], 230400)) {
    std::cerr << "OPEN_FAILED: check port, permissions and other serial programs\n";
    return 3;
  }
  if (!driver.WaitLidarCommConnect(5000)) {
    std::cerr << "NO_VALID_PACKETS: check power, TX/RX/GND, model and baud (230400)\n";
    driver.Stop(); return 4;
  }
  std::cerr << "Connected: " << model << ", 230400 baud; scan-only, no chassis commands.\n";
  unsigned seq = 0;
  int exit_code = 0;
  ldlidar::Points2D points;
  while (running && ldlidar::LDLidarDriver::IsOk()) {
    auto status = driver.GetLaserScanData(points, 2500);
    if (status == ldlidar::LidarStatus::NORMAL) {
      double hz = 0; driver.GetLidarScanFreq(hz);
      std::ostringstream line;
      line << "SCAN {\"seq\":" << ++seq << ",\"hz\":" << std::fixed
           << std::setprecision(4) << hz << ",\"points\":[";
      bool first = true;
      for (const auto& p : points) {
        if (!first) line << ',';
        first = false;
        line << '[' << p.angle << ',' << p.distance << ',' << unsigned(p.intensity)
             << ',' << p.stamp << ']';
      }
      line << "]}";
      std::cout << line.str() << std::endl;
    } else if (status == ldlidar::LidarStatus::DATA_TIME_OUT) {
      std::cerr << "SCAN_TIMEOUT: valid full scans stopped\n";
      exit_code = 5; break;
    } else if (status == ldlidar::LidarStatus::ERROR) {
      std::cerr << "LIDAR_ERROR: " << unsigned(driver.GetLidarErrorCode()) << '\n';
      exit_code = 6; break;
    } else if (status == ldlidar::LidarStatus::STOP) {
      std::cerr << "LIDAR_STOPPED\n"; exit_code = 7; break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  driver.Stop();
  return exit_code;
}
