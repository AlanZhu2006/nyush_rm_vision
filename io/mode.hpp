#ifndef IO__MODE_HPP
#define IO__MODE_HPP

#include <string>
#include <vector>

namespace io
{
enum Mode
{
  idle = 0,
  auto_aim = 1,
  small_buff = 2,
  big_buff = 3,
  outpost = 4
};

const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

}  // namespace io

#endif  // IO__MODE_HPP
