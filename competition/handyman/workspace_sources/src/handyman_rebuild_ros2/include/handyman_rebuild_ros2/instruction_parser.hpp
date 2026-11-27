#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "handyman_rebuild_ros2/module_interfaces.hpp"

namespace handyman_rebuild_ros2
{

class NameAliases
{
public:
  bool loadFromFile(const std::string & path, std::string & error);

  const std::unordered_map<std::string, std::vector<std::string>> & rooms() const noexcept;
  const std::unordered_map<std::string, std::vector<std::string>> & objects() const noexcept;
  const std::unordered_map<std::string, std::vector<std::string>> & destinations() const noexcept;

private:
  std::unordered_map<std::string, std::vector<std::string>> rooms_;
  std::unordered_map<std::string, std::vector<std::string>> objects_;
  std::unordered_map<std::string, std::vector<std::string>> destinations_;
};

class RuleBasedInstructionParser final : public InstructionParser
{
public:
  explicit RuleBasedInstructionParser(NameAliases aliases);
  OperationResult parse(const std::string & instruction, HandymanTask & task) override;

private:
  struct Match
  {
    std::string canonical;
    std::size_t position{0};
    std::size_t length{0};
  };

  static std::string normalize(const std::string & value);
  static std::vector<Match> findMatches(
    const std::string & normalized,
    const std::unordered_map<std::string, std::vector<std::string>> & aliases);

  NameAliases aliases_;
};

}  // namespace handyman_rebuild_ros2
