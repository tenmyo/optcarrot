
#include <memory>

namespace optcarrot {
class Config;
class Video;

/// CPU implementation
class CPU {
public:
  explicit CPU(std::shared_ptr<Config> conf);
  ~CPU();
  // disallow copy
  CPU(const CPU &) = delete;
  CPU &operator=(const CPU &) = delete;
  // allow move
  CPU(CPU &&) noexcept = default;
  CPU &operator=(CPU &&) noexcept = default;

  void reset();

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
