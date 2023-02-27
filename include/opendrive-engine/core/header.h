#ifndef OPENDRIVE_ENGINE_CORE_HEADER_H_
#define OPENDRIVE_ENGINE_CORE_HEADER_H_

#include <memory>
#include <string>

namespace opendrive {
namespace engine {
namespace core {

class Header {
 public:
  typedef std::shared_ptr<Header> Ptr;
  typedef std::shared_ptr<Header const> ConstPtr;
  Header() = default;
  void set_rev_major(const std::string& s) { rev_major_ = s; }
  void set_rev_minor(const std::string& s) { rev_minor_ = s; }
  void set_name(const std::string& s) { name_ = s; }
  void set_version(const std::string& s) { version_ = s; }
  void set_date(const std::string& s) { date_ = s; }
  void set_north(double d) { north_ = d; }
  void set_south(double d) { south_ = d; }
  void set_west(double d) { west_ = d; }
  void set_east(double d) { east_ = d; }
  void set_vendor(const std::string& s) { vendor_ = s; }
  const std::string& rev_major() { return rev_major_; }
  const std::string& rev_minor() { return rev_minor_; }
  const std::string& name() { return name_; }
  const std::string& version() { return version_; }
  const std::string& date() { return date_; }
  const std::string& vendor() { return vendor_; }
  double north() { return north_; }
  double south() { return south_; }
  double west() { return west_; }
  double east() { return east_; }

 private:
  std::string rev_major_;
  std::string rev_minor_;
  std::string name_;
  std::string version_;
  std::string date_;
  std::string vendor_;
  double north_ = 0;
  double south_ = 0;
  double west_ = 0;
  double east_ = 0;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_HEADER_H_
