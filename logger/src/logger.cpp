/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2019
 *
 */

#include "logger/logger.hpp"

#include <cstdlib>
#include <memory>
#include <string>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#define LOGGER_FILENAME "TerabeeLog.log"
#define LOGGER_MAX_FILE_SIZE 1024*1024*64  // using 64MB log files
#define LOGGER_MAX_NUMBER_OF_FILES 20  // let's say that 20 files (64MB each) is enough

namespace terabee
{
namespace utility
{
namespace logger
{

namespace
{
std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> g_file_sink;
}

LoggerParameterInitializer::LoggerParameterInitializer() {
  // If LOGGER_ENABLE_LOGGING is not set, no file is created
  g_file_sink = nullptr;
  if (!std::getenv(LOGGER_ENABLE_LOGGING)) {
    spdlog::set_level(spdlog::level::off);
    return;
  }
  // Set the level of logging
  if (std::getenv(LOGGER_ENABLE_DEBUG)) {
    spdlog::set_level(spdlog::level::debug);
  } else {
    spdlog::set_level(spdlog::level::info);
  }
  // If LOGGER_PRINT_STDOUT not set, create the logging file
  if (!std::getenv(LOGGER_PRINT_STDOUT)) {
    if (!std::getenv("LOGGER_FILENAME")) {
      g_file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          LOGGER_FILENAME, LOGGER_MAX_FILE_SIZE, LOGGER_MAX_NUMBER_OF_FILES);
    } else {
      g_file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          std::getenv("LOGGER_FILENAME"), LOGGER_MAX_FILE_SIZE,
          LOGGER_MAX_NUMBER_OF_FILES);
    }
  }
  spdlog::set_pattern(
      "%^[%Y-%m-%d %H:%M:%S.%e] <PID:%P> <Thread:%t> [%l] [%n] : %v%$");
}

Logger::Logger(const std::string& name) {
  logger_ = spdlog::get(name);
  if (logger_) return;
  if (std::getenv(LOGGER_PRINT_STDOUT)) {
    logger_ = spdlog::stdout_color_mt(name);
  } else if (std::getenv(LOGGER_ENABLE_LOGGING)) {
    logger_ = std::make_shared<spdlog::logger>(name, g_file_sink);
    spdlog::initialize_logger(logger_);
  } else {
    logger_ = std::make_shared<spdlog::logger>(name);
    spdlog::initialize_logger(logger_);
  }
}

const LoggerParameterInitializer Logger::initializer_ = LoggerParameterInitializer();

}  // namespace logger
}  // namespace utility
}  // namespace terabee
