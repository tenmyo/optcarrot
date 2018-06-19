#include "optcarrot.h"
#include "optcarrot/Config.h"
#include "optcarrot/Driver.h"
#include "optcarrot/NES.h"
#include <memory>
using namespace optcarrot;

int main(int argc, const char *argv[]) {
  std::shared_ptr<Config> conf{std::make_shared<Config>(argc, argv)};
  NES nes{conf};
  SDL2Video v{conf};
  nes.run();
  return 0;
}
