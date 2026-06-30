#include <CivetServer.h>
#include <civetweb.h>

#include "planning_service/common/file_utils.h"
#include "planning_service/draco/planner/draco_planner.h"

// This code is a simple web server using CivetWeb that serves an HTML page
#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

namespace service {
namespace utils {
namespace web {

using json = nlohmann::json;

struct Entry {
  long unsigned int context_id;
  int port;
  std::string panda_east;
  std::string panda_east_tool;
  std::string panda_west;
  std::string panda_west_tool;
};

// Generate HTML table
std::string GenerateHtmlTable(const std::vector<Entry>& meshcat_ports) {
  std::ostringstream oss;
  oss << R"(
<html>
  <head>
    <title>Meshcat Port Table</title>
    <link href="https://fonts.googleapis.com/css2?family=Roboto&display=swap" rel="stylesheet">
    <style>
      body {
        font-family: 'Roboto', sans-serif;
        background-color: #f7d8ffff;
        margin: 40px;
      }
      h1 {
        color: #333;
      }
      table {
        border-collapse: collapse;
        width: 90%;
        background-color: #fff;
        box-shadow: 0 2px 6px rgba(0,0,0,0.1);
      }
      th, td {
        text-align: left;
        padding: 12px 16px;
        border-bottom: 1px solid #ddd;
        vertical-align: top;
      }
      th {
        background-color: #f0f0f0;
        color: #333;
      }
      tr:hover {
        background-color: #d5ffc0ff;
      }
      a {
        color: #0066cc;
        text-decoration: none;
      }
      a:hover {
        text-decoration: underline;
      }
    </style>
  </head>
  <body>
    <div style="margin-bottom: 20px; font-size: 18px;">
    <strong>Note</strong>: For the Meshcat session of the live robot, open
    <a href="#" onclick="window.open('http://' + window.location.hostname + ':7000', '_blank'); return false;">port 7000</a>.
     Remember, you need to run the corresponding script first. Please refer to the
    proper documentation for more details.
    </div>
    <h1>Meshcat Port Table</h1>
    <table>
      <tr>
        <th>Context ID</th>
        <th>Meshcat Port</th>
        <th>PANDA_EAST</th>
        <th>PANDA_EAST_TOOL</th>
        <th>PANDA_WEST</th>
        <th>PANDA_WEST_TOOL</th>
      </tr>
)";

  for (const auto& entry : meshcat_ports) {
    oss << "<tr><td>" << entry.context_id << "</td>"
        << "<td><a href=\"#\" onclick=\"window.open('http://' + "
           "window.location.hostname + ':"
        << entry.port << "', '_blank'); return false;\">" << entry.port
        << "</a></td>" << "<td>" << entry.panda_east << "</td>" << "<td>"
        << entry.panda_east_tool << "</td>" << "<td>" << entry.panda_west
        << "</td>" << "<td>" << entry.panda_west_tool << "</td></tr>\n";
  }

  oss << R"(
    </table>
  </body>
</html>
)";
  return oss.str();
}

// Handler class
class HtmlHandler : public CivetHandler {
 public:
  explicit HtmlHandler(const std::vector<Entry>& meshcat_ports)
      : meshcat_ports_(meshcat_ports) {}

  bool handleGet(CivetServer*, struct mg_connection* conn) override {
    std::string html = GenerateHtmlTable(meshcat_ports_);
    mg_printf(conn,
              "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
              "Content-Length: %lu\r\n\r\n%s",
              html.length(), html.c_str());
    return true;
  }

 private:
  const std::vector<Entry>& meshcat_ports_;
};

void MakeMeshcatPortsHtml(
    const std::map<uint64_t, std::unique_ptr<draco::planner::DracoPlanner>>&
        draco_map,
    std::atomic<bool>& stop_requested, std::mutex& html_mutex,
    std::condition_variable& html_cv, int port_number) {
  std::string port_str = std::to_string(port_number);
  const char* port_number_char = port_str.c_str();  // must keep port_str alive
  const char* options[] = {"document_root",
                           ".",  // Required but unused
                           "listening_ports",
                           port_number_char,
                           "enable_keep_alive",
                           "no",
                           0};
  std::unique_ptr<CivetServer> server;
  try {
    server = std::make_unique<CivetServer>(options);
  } catch (const std::exception& e) {
    logging::log()->error("Failed to start Meshcat ports HTML server: {}",
                          e.what());
    return;
  }
  std::vector<Entry> entries;
  // Now convert the draco_map to entries
  for (const auto& [hash, planner] : draco_map) {
    std::string panda_east_setting = "Metadata not properly loaded!";
    std::string panda_east_tool = "NA";
    std::string panda_west_setting = "Metadata not properly loaded!";
    std::string panda_west_tool = "NA";
    auto metadata_file = planner->context_dir() / "metadata.json";
    if (fs::is_regular_file(metadata_file)) {
      try {
        json metadata;
        common::utils::LoadJsonFromFile(metadata_file, metadata);
        panda_east_setting = fmt::format(
            "{}, {}, {}", metadata["panda_east"]["finger_type"],
            metadata["panda_east"]["gripper_type"],
            metadata["panda_east"]["calibration_dh"]["calibration_id"]);
        panda_east_tool = fmt::format("<strong>{}</strong>",
                                      metadata["panda_east"]["tool_type"]);
        panda_west_setting = fmt::format(
            "{}, {}, {}", metadata["panda_west"]["finger_type"],
            metadata["panda_west"]["gripper_type"],
            metadata["panda_west"]["calibration_dh"]["calibration_id"]);
        panda_west_tool = fmt::format("<strong>{}</strong>",
                                      metadata["panda_west"]["tool_type"]);
      } catch (const std::exception& e) {
        logging::log()->error("Failed to load metadata from {}: {}",
                              metadata_file, e.what());
      }
    } else {
      logging::log()->warn(
          "MakeMeshcatPortsHtml: Metadata file not found at {}", metadata_file);
    }
    if (!planner->has_draco_visualizer()) {
      logging::log()->warn(
          "MakeMeshcatPortsHtml: DracoPlanner {} does not have a visualizer.",
          hash);
      continue;  // Skip if no visualizer is available
    }
    logging::log()->debug(
        "MakeMeshcatPortsHtml: Adding entry for DracoPlanner with hash {}",
        hash);
    int port = planner->mutable_draco_visualizer().Port();
    entries.push_back({hash, port, panda_east_setting, panda_east_tool,
                       panda_west_setting, panda_west_tool});
  }

  // Sort entries based on meshcat port in ascending order
  std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
    return a.port < b.port;
  });

  service::utils::web::HtmlHandler handler(entries);
  server->addHandler("/", handler);

  logging::log()->info(
      "MakeMeshcatPortsHtml: Meshcat ports HTML server started on "
      "http://localhost:{}",
      port_number);

  // Keep the server alive until stop_requested is set
  std::unique_lock<std::mutex> lock(html_mutex);
  html_cv.wait(lock, [&stop_requested]() {
    return stop_requested.load();
  });
  server->removeHandler("/");
  server.reset();  // Clean up the server
}

}  // namespace web
}  // namespace utils
}  // namespace service
