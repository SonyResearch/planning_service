#include <ctime>
#include <string>
namespace common {
namespace utils {

/** Date-only format (YYYYMMDD). */
extern const std::string DATE_FMT;
/** Date and time format ([YYYYMMDD]T[hhmmss]). */
extern const std::string DATETIME_FMT;

/**
 * @brief Return the current date and time as a string.
 *
 * @param fmt Time format string
 * @return const std::string
 */
const std::string datetime_str(const std::string_view fmt);
}  // namespace utils
}  // namespace common
