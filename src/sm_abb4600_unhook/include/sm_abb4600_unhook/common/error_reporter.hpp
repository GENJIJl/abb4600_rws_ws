#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace sm_abb4600_unhook
{

inline std::string lastErrorFilePath()
{
  return "/tmp/abb4600_unhook_last_error.txt";
}

inline std::string makeErrorReport(
  const std::string & state,
  const std::string & reason,
  const std::string & fix)
{
  std::ostringstream oss;

  oss << "\n========== ABB4600 UNHOOK ERROR REPORT ==========\n";
  oss << "State:\n";
  oss << "  " << state << "\n\n";
  oss << "Reason:\n";
  oss << "  " << reason << "\n\n";
  oss << "Suggested Fix:\n";
  oss << fix << "\n";
  oss << "=================================================\n";

  return oss.str();
}

inline void writeLastErrorReport(
  const rclcpp::Logger & logger,
  const std::string & state,
  const std::string & reason,
  const std::string & fix)
{
  const auto report = makeErrorReport(state, reason, fix);

  std::ofstream file(lastErrorFilePath(), std::ios::out | std::ios::trunc);
  if (file.is_open())
  {
    file << report;
    file.close();
  }

  RCLCPP_ERROR(logger, "%s", report.c_str());
}

inline std::string readLastErrorReport()
{
  std::ifstream file(lastErrorFilePath());

  if (!file.is_open())
  {
    return makeErrorReport(
      "unknown",
      "No detailed error report was written before entering StError.",
      "Check the state machine terminal log and RTA transition history.");
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace sm_abb4600_unhook
