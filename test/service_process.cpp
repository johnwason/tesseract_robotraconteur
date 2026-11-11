/**
 * @file service_process.cpp
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

#include "service_process.h"

#include <chrono>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <csignal>
#include <cstdlib>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace tesseract_robotraconteur
{
namespace test
{

#ifdef _WIN32

static std::string QuoteArg(const std::string& arg)
{
    if (!arg.empty() && arg.find_first_of(" \t\"") == std::string::npos)
    {
        return arg;
    }
    std::string ret = "\"";
    size_t backslashes = 0;
    for (char c : arg)
    {
        if (c == '\\')
        {
            ++backslashes;
            continue;
        }
        if (c == '"')
        {
            ret.append(backslashes * 2 + 1, '\\');
        }
        else
        {
            ret.append(backslashes, '\\');
        }
        backslashes = 0;
        ret.push_back(c);
    }
    ret.append(backslashes * 2, '\\');
    ret.push_back('"');
    return ret;
}

void ServiceProcess::Start(const std::string& executable, const std::vector<std::string>& args)
{
    std::string command_line = QuoteArg(executable);
    for (const auto& arg : args)
    {
        command_line += " " + QuoteArg(arg);
    }

    STARTUPINFOA startup_info{};
    startup_info.cb = sizeof(startup_info);
    PROCESS_INFORMATION process_info{};
    std::vector<char> command_line_buf(command_line.begin(), command_line.end());
    command_line_buf.push_back('\0');
    if (!CreateProcessA(executable.c_str(), command_line_buf.data(), nullptr, nullptr, TRUE, CREATE_NEW_PROCESS_GROUP,
                        nullptr, nullptr, &startup_info, &process_info))
    {
        throw std::runtime_error("Could not start " + executable);
    }
    CloseHandle(process_info.hThread);
    process_handle = process_info.hProcess;
    process_id = process_info.dwProcessId;
}

bool ServiceProcess::IsRunning() { return process_handle && WaitForSingleObject(process_handle, 0) == WAIT_TIMEOUT; }

int ServiceProcess::Stop(int timeout_ms)
{
    if (!process_handle)
    {
        return -1;
    }
    int ret = -1;
    GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, process_id);
    if (WaitForSingleObject(process_handle, timeout_ms) == WAIT_OBJECT_0)
    {
        DWORD exit_code = 0;
        if (GetExitCodeProcess(process_handle, &exit_code))
        {
            ret = static_cast<int>(exit_code);
        }
    }
    else
    {
        TerminateProcess(process_handle, 1);
        WaitForSingleObject(process_handle, INFINITE);
    }
    CloseHandle(process_handle);
    process_handle = nullptr;
    return ret;
}

ServiceProcess::~ServiceProcess()
{
    if (process_handle)
    {
        TerminateProcess(process_handle, 1);
        WaitForSingleObject(process_handle, INFINITE);
        CloseHandle(process_handle);
    }
}

int ServiceProcess::CurrentProcessId() { return static_cast<int>(GetCurrentProcessId()); }

void ServiceProcess::SetEnv(const std::string& name, const std::string& value)
{
    _putenv_s(name.c_str(), value.c_str());
}

#else

void ServiceProcess::Start(const std::string& executable, const std::vector<std::string>& args)
{
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(executable.c_str()));
    for (const auto& arg : args)
    {
        argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);

    pid = fork();
    if (pid < 0)
    {
        throw std::runtime_error("Could not fork to start " + executable);
    }
    if (pid == 0)
    {
        execv(executable.c_str(), argv.data());
        _exit(127);
    }
}

bool ServiceProcess::IsRunning()
{
    if (pid <= 0 || exited)
    {
        return false;
    }
    int status = 0;
    pid_t res = waitpid(pid, &status, WNOHANG);
    if (res == pid)
    {
        exited = true;
        exit_status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        return false;
    }
    return res == 0;
}

int ServiceProcess::Stop(int timeout_ms)
{
    if (pid <= 0)
    {
        return -1;
    }
    if (IsRunning())
    {
        kill(pid, SIGINT);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (IsRunning() && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (IsRunning())
        {
            kill(pid, SIGKILL);
            waitpid(pid, nullptr, 0);
            exited = true;
            exit_status = -1;
        }
    }
    pid = -1;
    return exit_status;
}

ServiceProcess::~ServiceProcess()
{
    if (IsRunning())
    {
        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
    }
}

int ServiceProcess::CurrentProcessId() { return static_cast<int>(getpid()); }

void ServiceProcess::SetEnv(const std::string& name, const std::string& value)
{
    setenv(name.c_str(), value.c_str(), 1);
}

#endif

} // namespace test
} // namespace tesseract_robotraconteur
