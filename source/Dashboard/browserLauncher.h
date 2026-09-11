/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef BROWSERLAUNCHER_H
#define BROWSERLAUNCHER_H

#pragma once

#include <spawn.h>
#include <unistd.h>
#include <string>
#include <string_view>
#include <vector>
#include <system_error>

extern char **environ;

/// Opens the dashboard in an app-style browser window on startup. See
/// docs/Dashboard.md for the hardcoded-browser-name gotcha on VMware/Mesa.
class BrowserLauncher {

public:

    struct Result {
        bool success{false};
        pid_t pid{-1};
        std::error_code error{};
    };

    /// @param url Opened via `chromium --app=<url>`.
    /// @return `success=false` with `error` set if the spawn itself failed
    ///         (e.g. chromium not installed), does not confirm the
    ///         browser actually rendered anything.
    static Result launch(const std::string_view url) {
        std::vector<std::string> arguments =
        {
            "chromium",
            "--app=" + std::string(url),
            "--disable-logging",
            "--log-level=3"
        };

        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1);

        for (auto& arg : arguments) {
            argv.push_back(arg.data());
        }

        argv.push_back(nullptr);

        // Custom environment
        std::vector<std::string> environment = {"GTK_MODULES=",};

        // Copy current environment
        for (char** env = environ; *env != nullptr; ++env) {
            environment.emplace_back(*env);
        }

        std::vector<char*> envp;
        envp.reserve(environment.size() + 1);

        for (auto& e : environment) {
            envp.push_back(e.data());
        }

        envp.push_back(nullptr);


        pid_t pid = -1;

        const int rc = ::posix_spawnp(
            &pid,
            arguments[0].c_str(),
            nullptr,
            nullptr,
            argv.data(),
            envp.data());


        if (rc != 0) {
            return {.success = false, .pid = -1, .error = std::error_code(rc, std::generic_category())};
        }

        return {.success = true, .pid = pid, .error = {}};
    }

private:
    BrowserLauncher() = delete;
};

#endif //BROWSERLAUNCHER_H
