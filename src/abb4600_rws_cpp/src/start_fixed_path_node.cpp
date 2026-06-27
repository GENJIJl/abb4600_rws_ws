#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <curl/curl.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace
{
size_t writeCallback(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  auto * body = static_cast<std::string *>(userdata);
  body->append(ptr, size * nmemb);
  return size * nmemb;
}

struct HttpResponse
{
  CURLcode curl_code{CURLE_OK};
  long http_code{0};
  std::string body;
};
}  // namespace

class StartFixedPathNode : public rclcpp::Node
{
public:
  StartFixedPathNode()
  : Node("start_fixed_path_node")
  {
    base_url_ = this->declare_parameter<std::string>("base_url", "http://120.55.45.142:15555");
    username_ = this->declare_parameter<std::string>("username", "Default User");
    password_ = this->declare_parameter<std::string>("password", "robotics");
    request_timeout_s_ = this->declare_parameter<int>("request_timeout_s", 10);

    const auto curl_init_code = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (curl_init_code != CURLE_OK) {
      RCLCPP_ERROR(
        this->get_logger(), "libcurl 全局初始化失败：%s", curl_easy_strerror(curl_init_code));
    } else {
      curl_initialized_ = true;
    }

    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/abb4600/start_fixed_path",
      rclcpp::QoS(10),
      std::bind(&StartFixedPathNode::triggerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "StartFixedPathNode 已就绪。base_url='%s', username='%s'",
      base_url_.c_str(),
      username_.c_str());
  }

  ~StartFixedPathNode() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_.joinable()) {
        worker_.join();
      }
    }

    if (curl_initialized_) {
      curl_global_cleanup();
    }
  }

private:
  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    if (running_.exchange(true)) {
      RCLCPP_WARN(this->get_logger(), "RWS 启动请求正在执行，忽略本次重复触发。");
      return;
    }

    if (!curl_initialized_) {
      RCLCPP_ERROR(this->get_logger(), "libcurl 尚未初始化，无法调用 ABB RWS。");
      running_.store(false);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "正在通过 ABB RWS 启动固定 RAPID 路径...");

    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }

    worker_ = std::thread([this]() {
      const bool ok = startFixedPath();
      if (ok) {
        RCLCPP_INFO(this->get_logger(), "固定 RAPID 路径启动成功。");
      }

      running_.store(false);
    });
  }

  bool startFixedPath()
  {
    const auto execution = get("/rw/rapid/execution");
    if (execution.curl_code != CURLE_OK) {
      logCurlError("check RAPID execution state", execution);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "RAPID 执行状态 HTTP_CODE=%ld", execution.http_code);

    const auto reset = post("/rw/rapid/execution?action=resetpp", "");
    if (!expectHttpCode("resetpp", reset, 204)) {
      return false;
    }

    const auto start = post(
      "/rw/rapid/execution?action=start",
      "regain=continue&execmode=continue&cycle=once&condition=none&stopatbp=disabled&"
      "alltaskbytsp=false");
    if (!expectHttpCode("start RAPID", start, 204)) {
      return false;
    }

    return true;
  }

  HttpResponse get(const std::string & path)
  {
    return request("GET", path, "");
  }

  HttpResponse post(const std::string & path, const std::string & form_data)
  {
    return request("POST", path, form_data);
  }

  HttpResponse request(
    const std::string & method, const std::string & path, const std::string & form_data)
  {
    HttpResponse response;

    CURL * curl = curl_easy_init();
    if (curl == nullptr) {
      response.curl_code = CURLE_FAILED_INIT;
      return response;
    }

    struct curl_slist * headers = nullptr;
    const std::string url = base_url_ + path;
    const std::string userpwd = username_ + ":" + password_;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST);
    curl_easy_setopt(curl, CURLOPT_USERPWD, userpwd.c_str());
    curl_easy_setopt(curl, CURLOPT_NOPROXY, "*");
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, request_timeout_s_);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);

    if (method == "POST") {
      headers = curl_slist_append(headers, "Content-Type: application/x-www-form-urlencoded;v=2.0");
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl, CURLOPT_POST, 1L);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, form_data.c_str());
    }

    response.curl_code = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.http_code);

    if (headers != nullptr) {
      curl_slist_free_all(headers);
    }
    curl_easy_cleanup(curl);

    return response;
  }

  bool expectHttpCode(
    const std::string & action, const HttpResponse & response, const long expected_code)
  {
    if (response.curl_code != CURLE_OK) {
      logCurlError(action, response);
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "%s HTTP_CODE=%ld", action.c_str(), response.http_code);
    if (response.http_code == expected_code) {
      return true;
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "%s 失败。期望 HTTP %ld，实际 HTTP %ld。响应内容：%s",
      action.c_str(),
      expected_code,
      response.http_code,
      response.body.c_str());
    return false;
  }

  void logCurlError(const std::string & action, const HttpResponse & response)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s 失败。CURL 错误：%s。HTTP_CODE=%ld。响应内容：%s",
      action.c_str(),
      curl_easy_strerror(response.curl_code),
      response.http_code,
      response.body.c_str());
  }

  std::string base_url_;
  std::string username_;
  std::string password_;
  int request_timeout_s_{10};
  bool curl_initialized_{false};
  std::atomic_bool running_{false};
  std::mutex worker_mutex_;
  std::thread worker_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StartFixedPathNode>());
  rclcpp::shutdown();
  return 0;
}
