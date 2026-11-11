/**
 * @file service_process.h
 *
 * @author John Wason, PhD
 *
 * @copyright Copyright 2026 Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * @par
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_ROBOTRACONTEUR_TEST_SERVICE_PROCESS_H
#define TESSERACT_ROBOTRACONTEUR_TEST_SERVICE_PROCESS_H

#include <string>
#include <vector>

namespace tesseract_robotraconteur
{
namespace test
{

// Runs the service executable as a child process. The child inherits stdout, stderr and the environment.
class ServiceProcess
{
  public:
    ServiceProcess() = default;
    ~ServiceProcess();
    ServiceProcess(const ServiceProcess&) = delete;
    ServiceProcess& operator=(const ServiceProcess&) = delete;

    void Start(const std::string& executable, const std::vector<std::string>& args);

    bool IsRunning();

    // Requests a graceful shutdown the way drekar-launch does, which drekar_launch_process_cpp::CWaitForExit in the
    // service handles: SIGINT on POSIX, or WM_CLOSE to the drekar message window on Windows. Then waits for the
    // process to exit.
    // The process is killed if it does not exit within timeout_ms. Returns the exit code, or -1 if the
    // process was killed or did not exit normally.
    int Stop(int timeout_ms);

    static int CurrentProcessId();

    static void SetEnv(const std::string& name, const std::string& value);

  private:
#ifdef _WIN32
    void* process_handle = nullptr;
    unsigned long process_id = 0;
#else
    int pid = -1;
    int exit_status = -1;
    bool exited = false;
#endif
};

} // namespace test
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_TEST_SERVICE_PROCESS_H
