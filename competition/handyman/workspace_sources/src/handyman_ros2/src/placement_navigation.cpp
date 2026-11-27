#include "handyman_ros2/placement_navigation.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace handyman_ros2 {
namespace {

std::string normalizeKey(const std::string &value)
{
  std::string normalized;
  normalized.reserve(value.size());
  bool last_was_separator = false;

  for (unsigned char ch : value) {
    if (std::isalnum(ch)) {
      normalized += static_cast<char>(std::tolower(ch));
      last_was_separator = false;
    } else if (ch == ' ' || ch == '-' || ch == '_') {
      if (!last_was_separator) {
        normalized += '_';
        last_was_separator = true;
      }
    } else {
      if (!last_was_separator) {
        normalized += '_';
        last_was_separator = true;
      }
    }
  }

  if (!normalized.empty() && normalized.back() == '_') {
    normalized.pop_back();
  }

  return normalized;
}

std::string normalizeForParsing(const std::string &value)
{
  std::string normalized;
  normalized.reserve(value.size());
  bool last_was_space = true;

  for (unsigned char ch : value) {
    if (std::isalnum(ch)) {
      normalized += static_cast<char>(std::tolower(ch));
      last_was_space = false;
    } else {
      if (!last_was_space) {
        normalized += ' ';
        last_was_space = true;
      }
    }
  }

  if (!normalized.empty() && normalized.back() == ' ') {
    normalized.pop_back();
  }

  return normalized;
}

bool matchWholePhrase(
  const std::string &text,
  const std::string &phrase,
  std::size_t position)
{
  if (position > 0 && text[position - 1] != ' ') {
    return false;
  }

  const std::size_t end = position + phrase.size();
  if (end < text.size() && text[end] != ' ') {
    return false;
  }

  return true;
}

struct RoomAliasDefinition {
  const char *alias_text;
  const char *canonical_key;
  const char *canonical_label;
};

static constexpr RoomAliasDefinition kRoomAliasDefinitions[] = {
  {"living room", "living", "living"},
  {"living", "living", "living"},
  {"bedroom", "bedroom", "bedroom"},
  {"lobby", "lobby", "lobby"},
  {"kitchen", "kitchen", "kitchen"},
  {"trash box for bottle can", "trash_box_for_bottle_can", "trash_box_for_bottle_can"},
  {"trash_box_for_bottle_can", "trash_box_for_bottle_can", "trash_box_for_bottle_can"}
};

const std::unordered_map<std::string, std::string> &roomAliasKeyLookup()
{
  static const std::unordered_map<std::string, std::string> lookup = [] {
    std::unordered_map<std::string, std::string> map;
    map.reserve(std::size(kRoomAliasDefinitions));
    for (const auto &entry : kRoomAliasDefinitions) {
      const std::string key = normalizeKey(entry.alias_text);
      if (!key.empty()) {
        map[key] = entry.canonical_key;
      }
    }
    return map;
  }();
  return lookup;
}

const std::vector<std::pair<std::string, std::string>> &roomAliasParseTable()
{
  static const std::vector<std::pair<std::string, std::string>> table = [] {
    std::vector<std::pair<std::string, std::string>> entries;
    entries.reserve(std::size(kRoomAliasDefinitions));
    for (const auto &entry : kRoomAliasDefinitions) {
      const std::string phrase = normalizeForParsing(entry.alias_text);
      if (!phrase.empty()) {
        entries.emplace_back(phrase, entry.canonical_key);
      }
    }
    return entries;
  }();
  return table;
}

const std::unordered_map<std::string, std::string> &canonicalRoomLabels()
{
  static const std::unordered_map<std::string, std::string> labels = [] {
    std::unordered_map<std::string, std::string> map;
    map.reserve(std::size(kRoomAliasDefinitions));
    for (const auto &entry : kRoomAliasDefinitions) {
      map[entry.canonical_key] = entry.canonical_label;
    }
    return map;
  }();
  return labels;
}

std::string canonicalRoomKey(const std::string &value)
{
  const std::string normalized = normalizeKey(value);
  if (normalized.empty()) {
    return {};
  }

  const auto &lookup = roomAliasKeyLookup();
  const auto it = lookup.find(normalized);
  if (it != lookup.end()) {
    return it->second;
  }

  return normalized;
}

std::string canonicalRoomLabel(const std::string &key)
{
  const auto &labels = canonicalRoomLabels();
  const auto it = labels.find(key);
  if (it != labels.end()) {
    return it->second;
  }

  return key;
}

struct CandidateKey {
  std::string environment;
  std::string destination;
  std::string room;

  bool matches(const CandidateKey &other) const
  {
    return environment == other.environment &&
      destination == other.destination &&
      room == other.room;
  }
};

struct CandidateEntry {
  CandidateKey key;
  std::vector<PlacementCandidate> candidates;
};

const std::vector<CandidateEntry> &placementCandidateTable()
{
  static const std::vector<CandidateEntry> table = [] {
    return std::vector<CandidateEntry>{
      {
        {"layout2019hm01", "square_low_table", "living"},
        {
          {0.774, 2.79, 0.8},
          {1.10, 2.35, 1.20},
          {0.30, 2.35, 0.20}
        }
      },
      {
        {"layout2019hm01", "trash_box_for_bottle_can", "living"},
        {
          {-0.8, -2.0, 3.14},
          {-0.2, -2.0, 3.14},
          {-0.8, -1.4, 3.14}
        }
      }
    };
  }();
  return table;
}

}  // namespace

std::string resolveDestinationRoom(
  const std::string &instruction,
  const std::vector<std::string> &rooms,
  const std::string &pickup_room)
{
  std::unordered_map<std::string, std::string> allowed_rooms;
  allowed_rooms.reserve(rooms.size() + 1);

  auto add_room = [&](const std::string &room) {
    const std::string key = canonicalRoomKey(room);
    if (key.empty()) {
      return;
    }

    if (allowed_rooms.find(key) == allowed_rooms.end()) {
      allowed_rooms[key] = canonicalRoomLabel(key);
    }
  };

  for (const auto &room : rooms) {
    add_room(room);
  }
  add_room(pickup_room);

  if (allowed_rooms.empty()) {
    return pickup_room;
  }

  const std::string normalized_instruction = normalizeForParsing(instruction);
  bool found = false;
  std::size_t best_position = 0;
  std::string resolved_room;

  for (const auto &entry : roomAliasParseTable()) {
    if (entry.first.empty()) {
      continue;
    }

    const auto allowed_it = allowed_rooms.find(entry.second);
    if (allowed_it == allowed_rooms.end()) {
      continue;
    }

    std::size_t position = normalized_instruction.find(entry.first);
    while (position != std::string::npos) {
      if (matchWholePhrase(normalized_instruction, entry.first, position)) {
        if (!found || position >= best_position) {
          found = true;
          best_position = position;
          resolved_room = allowed_it->second;
        }
      }

      position = normalized_instruction.find(entry.first, position + 1);
    }
  }

  if (found) {
    return resolved_room;
  }

  if (!rooms.empty()) {
    const std::string last_key = canonicalRoomKey(rooms.back());
    const auto last_it = allowed_rooms.find(last_key);
    if (last_it != allowed_rooms.end()) {
      return last_it->second;
    }
  }

  const std::string pickup_key = canonicalRoomKey(pickup_room);
  const auto pickup_it = allowed_rooms.find(pickup_key);
  if (pickup_it != allowed_rooms.end()) {
    return pickup_it->second;
  }

  return pickup_room;
}

std::vector<PlacementCandidate> placementCandidates(
  const std::string &environment,
  const std::string &destination,
  const std::string &destination_room)
{
  CandidateKey requested_key{
    normalizeKey(environment),
    normalizeKey(destination),
    canonicalRoomKey(destination_room)
  };

  for (const auto &entry : placementCandidateTable()) {
    if (entry.key.matches(requested_key)) {
      return entry.candidates;
    }
  }

  return {};
}

std::size_t nextPlacementCandidateIndex(
  std::size_t current_index,
  std::size_t candidate_count)
{
  if (candidate_count == 0) {
    return 0;
  }

  return (current_index + 1) % candidate_count;
}

}  // namespace handyman_ros2
