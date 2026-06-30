/// @file file_utils.h
#pragma once

#include <fmt/format.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <future>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
namespace fs = std::filesystem;

using json = nlohmann::json;
namespace common {
namespace utils {

/**
 * @brief Create a temp directory which is guaranteed to exist.
 *
 * @return fs::path
 */
fs::path temp_dir();

/**
 * @brief Read the contents from a file into a string instance.
 *
 * @param file_path path to target file
 * @return const std::string
 */
std::string FileToString(const fs::path& file_path);

/**
 * @brief Save string contents to a file.
 *
 * @param file_path Path to file
 * @param contents Contents to save
 * @param overwrite_existing If true, overwrite existing file
 * @param binary If true, write in binary mode
 * @return true
 * @return false
 */
bool SaveToFile(const fs::path& file_path, const std::string& contents,
                bool overwrite_existing = true, bool binary = false);

/**
 * @brief Save JSON object to a file
 *
 * @param file_path Path to file
 * @param j JSON object
 * @param overwrite_existing If true, overwrite existing file
 * @param indent Number of spaces to indent
 * @return true
 * @return false
 */
bool SaveJsonToFile(const fs::path& file_path, const json& j, int indent = 2,
                    bool overwrite_existing = true);

/**
 * @brief Load a JSON object from a file.
 *
 * @param file_path Path to file
 * @param j Target JSON object
 * @return true if successful, false otherwise
 */
bool LoadJsonFromFile(const fs::path& file_path, json& j);

/**
 * @brief Check if a string is present in a file.
 *
 * @param file_path Path to file
 * @param str String to search for
 * @return true if the string is found, false otherwise
 */
bool StringInFile(const fs::path& file_path, const std::string& str);

}  // namespace utils
}  // namespace common
