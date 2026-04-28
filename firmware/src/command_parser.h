#pragma once

// Serial command framing and parsing. Hardware independent so it can be unit
// tested on the development machine. See docs/protocol.md.

#include <stddef.h>
#include <stdint.h>

#include "config.h"

enum class CommandType : uint8_t {
  Aim,     // A<pan> <tilt>
  Fire,    // F
  Status,  // S
};

struct Command {
  CommandType type;
  float panDeg;
  float tiltDeg;
};

enum class ParseResult : uint8_t {
  Ok,
  Empty,    // blank line, ignored without a reply
  Unknown,  // unknown command letter
  Syntax,   // missing, extra or malformed arguments
  Value,    // number with too many integer digits
};

// Parses one line, newline already removed. `out` is only written on Ok.
ParseResult parseCommand(const char* line, Command& out);

// Protocol error code for a failed parse ("UNKNOWN", "SYNTAX", ...).
const char* parseResultCode(ParseResult result);

// Collects incoming bytes into complete lines without dynamic allocation.
// Accepts "\n" and "\r\n" line endings. A line longer than kMaxLineLength is
// discarded up to its newline and reported once as an overflow.
class LineAssembler {
 public:
  enum class Event : uint8_t { None, Line, Overflow };

  LineAssembler();

  // Feeds one byte. Returns Line when line() holds a complete line.
  Event push(char byte);

  // The last complete line; valid until the next call to push().
  const char* line() const { return buffer_; }

 private:
  void reset();

  char buffer_[config::kMaxLineLength + 1];
  size_t length_;
  bool discarding_;
};
