// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "app/detach.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <io.h>
#else
#include <fcntl.h>
#include <spawn.h>
#include <csignal>
#include <climits>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
extern char** environ;
#endif

namespace sg::app {

namespace {

// Set in the background copy's environment, so it knows not to start another.
constexpr const char* kMarker = "SG_DETACHED";

}  // namespace

#if defined(_WIN32)

namespace {

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
    w.resize(static_cast<std::size_t>(n - 1));
    return w;
}

void output_to(const std::string& log_path) {
    if (!log_path.empty() && _wfreopen(widen(log_path).c_str(), L"w", stdout)) _dup2(_fileno(stdout), _fileno(stderr));
}

}  // namespace

Detach detach_from_terminal(int, char**, const std::string& log_path) {
    if (GetEnvironmentVariableA(kMarker, nullptr, 0)) {
        SetEnvironmentVariableA(kMarker, nullptr);
        return Detach::Child;
    }
    if (!GetConsoleWindow()) {  // no console to free
        output_to(log_path);
        return Detach::Stay;
    }
    wchar_t exe[MAX_PATH];
    if (!GetModuleFileNameW(nullptr, exe, MAX_PATH)) return Detach::Stay;
    SECURITY_ATTRIBUTES inherit{sizeof inherit, nullptr, TRUE};
    HANDLE log = log_path.empty() ? INVALID_HANDLE_VALUE
                                  : CreateFileW(widen(log_path).c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                                &inherit, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    STARTUPINFOW si{};
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nullptr;
    si.hStdOutput = si.hStdError = log == INVALID_HANDLE_VALUE ? nullptr : log;
    std::wstring command = GetCommandLineW();  // CreateProcessW may write to it
    PROCESS_INFORMATION pi{};
    SetEnvironmentVariableA(kMarker, "1");
    const BOOL ok = CreateProcessW(exe, command.data(), nullptr, nullptr, TRUE, DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP,
                                   nullptr, nullptr, &si, &pi);
    SetEnvironmentVariableA(kMarker, nullptr);
    if (log != INVALID_HANDLE_VALUE) CloseHandle(log);
    if (!ok) return Detach::Stay;
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return Detach::Parent;
}

#else

namespace {

std::string self_path(const char* argv0) {
#if defined(__linux__)
    char buf[PATH_MAX];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof buf - 1);
    if (n > 0) return std::string(buf, static_cast<std::size_t>(n));
#elif defined(__APPLE__)
    char buf[PATH_MAX];
    uint32_t size = sizeof buf;
    if (_NSGetExecutablePath(buf, &size) == 0) return buf;
#endif
    return argv0;
}

void output_to(const std::string& log_path) {
    if (log_path.empty()) return;
    const int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return;
    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);
    if (fd > STDERR_FILENO) close(fd);
}

}  // namespace

Detach detach_from_terminal(int argc, char** argv, const std::string& log_path) {
    if (std::getenv(kMarker)) {
        unsetenv(kMarker);
        setsid();  // no controlling terminal from here on
        return Detach::Child;
    }
    if (!isatty(STDIN_FILENO) && !isatty(STDOUT_FILENO) && !isatty(STDERR_FILENO)) {
        output_to(log_path);
        return Detach::Stay;
    }
    // A fresh copy rather than fork(): on macOS the system frameworks are
    // not safe to use in a forked child that has not called exec.
    std::vector<std::string> env_text;
    for (char** e = environ; *e; ++e) env_text.emplace_back(*e);
    env_text.push_back(std::string(kMarker) + "=1");
    std::vector<char*> env;
    for (auto& e : env_text) env.push_back(e.data());
    env.push_back(nullptr);
    std::vector<char*> args(argv, argv + argc);
    args.push_back(nullptr);

    posix_spawn_file_actions_t files;
    posix_spawn_file_actions_init(&files);
    posix_spawn_file_actions_addopen(&files, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    if (!log_path.empty()) {
        posix_spawn_file_actions_addopen(&files, STDOUT_FILENO, log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        posix_spawn_file_actions_adddup2(&files, STDOUT_FILENO, STDERR_FILENO);
    }
    // Until the copy has its own session, Ctrl-C or a closing terminal must
    // not reach it: it inherits these as ignored (it sets its own handlers).
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGHUP, SIG_IGN);
    std::signal(SIGQUIT, SIG_IGN);
    pid_t pid = 0;
    const std::string exe = self_path(argv[0]);
    const int rc = posix_spawn(&pid, exe.c_str(), &files, nullptr, args.data(), env.data());
    posix_spawn_file_actions_destroy(&files);
    if (rc != 0) {
        std::signal(SIGINT, SIG_DFL);
        std::signal(SIGHUP, SIG_DFL);
        std::signal(SIGQUIT, SIG_DFL);
        return Detach::Stay;
    }
    return Detach::Parent;
}

#endif

}  // namespace sg::app
