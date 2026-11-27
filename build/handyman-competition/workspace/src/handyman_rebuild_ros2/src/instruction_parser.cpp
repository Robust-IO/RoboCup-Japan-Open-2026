#include "handyman_rebuild_ros2/instruction_parser.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace handyman_rebuild_ros2
{
namespace
{

bool loadAliasGroup(
  const YAML::Node & root, const std::string & group,
  std::unordered_map<std::string, std::vector<std::string>> & output,
  std::string & error)
{
  const YAML::Node node = root[group];
  if (!node || !node.IsMap()) {
    error = "Missing alias group: " + group;
    return false;
  }
  output.clear();
  for (const auto & item : node) {
    const std::string canonical = item.first.as<std::string>();
    if (!item.second.IsSequence() || item.second.size() == 0) {
      error = "Alias list is empty for: " + canonical;
      return false;
    }
    output[canonical] = item.second.as<std::vector<std::string>>();
    output[canonical].push_back(canonical);
  }
  return true;
}

}  // namespace

bool NameAliases::loadFromFile(const std::string & path, std::string & error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    return loadAliasGroup(root, "rooms", rooms_, error) &&
           loadAliasGroup(root, "objects", objects_, error) &&
           loadAliasGroup(root, "destinations", destinations_, error);
  } catch (const YAML::Exception & exception) {
    error = exception.what();
    return false;
  }
}

const std::unordered_map<std::string, std::vector<std::string>> &
NameAliases::rooms() const noexcept { return rooms_; }
const std::unordered_map<std::string, std::vector<std::string>> &
NameAliases::objects() const noexcept { return objects_; }
const std::unordered_map<std::string, std::vector<std::string>> &
NameAliases::destinations() const noexcept { return destinations_; }

RuleBasedInstructionParser::RuleBasedInstructionParser(NameAliases aliases)
: aliases_(std::move(aliases)) {}

std::string RuleBasedInstructionParser::normalize(const std::string & value)
{
  std::string output;
  output.reserve(value.size());
  bool last_was_space = true;
  for (const unsigned char character : value) {
    if (std::isalnum(character)) {
      output.push_back(static_cast<char>(std::tolower(character)));
      last_was_space = false;
    } else if (!last_was_space) {
      output.push_back(' ');
      last_was_space = true;
    }
  }
  if (!output.empty() && output.back() == ' ') {
    output.pop_back();
  }
  return output;
}

std::vector<RuleBasedInstructionParser::Match> RuleBasedInstructionParser::findMatches(
  const std::string & normalized,
  const std::unordered_map<std::string, std::vector<std::string>> & aliases)
{
  std::vector<Match> matches;
  const std::string padded = " " + normalized + " ";
  for (const auto & canonical_aliases : aliases) {
    for (const auto & raw_alias : canonical_aliases.second) {
      const std::string alias = normalize(raw_alias);
      if (alias.empty()) {
        continue;
      }
      const std::string needle = " " + alias + " ";
      std::size_t position = padded.find(needle);
      while (position != std::string::npos) {
        matches.push_back({canonical_aliases.first, position, alias.size()});
        position = padded.find(needle, position + 1);
      }
    }
  }
  std::sort(matches.begin(), matches.end(), [](const Match & left, const Match & right) {
    if (left.position != right.position) {
      return left.position < right.position;
    }
    return left.length > right.length;
  });
  std::vector<Match> unique;
  for (const auto & match : matches) {
    const bool duplicate = std::any_of(unique.begin(), unique.end(), [&match](const Match & existing) {
      return existing.canonical == match.canonical && existing.position == match.position;
    });
    if (!duplicate) {
      unique.push_back(match);
    }
  }
  return unique;
}

OperationResult RuleBasedInstructionParser::parse(
  const std::string & instruction, HandymanTask & task)
{
  const std::string normalized = normalize(instruction);
  if (normalized.empty()) {
    return {false, false, "Instruction is empty"};
  }
  const auto room_matches = findMatches(normalized, aliases_.rooms());
  const auto object_matches = findMatches(normalized, aliases_.objects());
  const auto destination_matches = findMatches(normalized, aliases_.destinations());
  if (room_matches.empty()) {
    return {false, false, "Instruction does not contain a known pickup room"};
  }
  if (object_matches.empty()) {
    return {false, false, "Instruction does not contain a known target object"};
  }
  if (destination_matches.empty()) {
    return {false, false, "Instruction does not contain a known destination"};
  }

  task.raw_instruction = instruction;
  task.pickup_room = room_matches.front().canonical;
  task.target_object = object_matches.front().canonical;
  task.destination = destination_matches.back().canonical;
  task.destination_is_avatar = task.destination == "avatar";
  if (task.destination_is_avatar) {
    task.destination_room.clear();
  } else if (room_matches.size() > 1) {
    task.destination_room = room_matches.back().canonical;
  } else {
    task.destination_room.clear();
  }
  return {true, false, ""};
}

}  // namespace handyman_rebuild_ros2
