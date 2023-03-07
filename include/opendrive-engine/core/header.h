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
  Header() : north_(0), south_(0), west_(0), east_(0) {}
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
  std::string& mutable_rev_major() { return rev_major_; }
  std::string& mutable_rev_minor() { return rev_minor_; }
  std::string& mutable_version() { return version_; }
  std::string& mutable_name() { return name_; }
  std::string& mutable_date() { return date_; }
  std::string& mutable_vendor() { return vendor_; }
  double& mutable_north() { return north_; }
  double& mutable_south() { return south_; }
  double& mutable_west() { return west_; }
  double& mutable_east() { return east_; }
  const std::string& rev_major() const { return rev_major_; }
  const std::string& rev_minor() const { return rev_minor_; }
  const std::string& name() const { return name_; }
  const std::string& version() const { return version_; }
  const std::string& date() const { return date_; }
  const std::string& vendor() const { return vendor_; }
  double north() const { return north_; }
  double south() const { return south_; }
  double west() const { return west_; }
  double east() const { return east_; }

 private:
  std::string rev_major_;
  std::string rev_minor_;
  std::string name_;
  std::string version_;
  std::string date_;
  std::string vendor_;
  double north_;
  double south_;
  double west_;
  double east_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_HEADER_H_
