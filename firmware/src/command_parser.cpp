#include "command_parser.h"

namespace {

// Integer digits accepted in a number; more is reported as a VALUE error.
constexpr int kMaxIntegerDigits = 6;

enum class NumberResult : uint8_t { Ok, Syntax, TooLarge };

bool isDigit(char c) { return c >= '0' && c <= '9'; }

const char* skipSpaces(const char* cursor) {
  while (*cursor == ' ' || *cursor == '\t') {
    ++cursor;
  }
  return cursor;
}

// Reads "[+-]digits[.digits]" (at least one digit) and advances the cursor.
NumberResult readDecimal(const char*& cursor, float& out) {
  const char* p = cursor;
  bool negative = false;
  if (*p == '+' || *p == '-') {
    negative = (*p == '-');
    ++p;
  }

  double value = 0.0;
  int integerDigits = 0;
  while (isDigit(*p)) {
    if (++integerDigits > kMaxIntegerDigits) {
      return NumberResult::TooLarge;
    }
    value = value * 10.0 + (*p - '0');
    ++p;
  }

  int fractionDigits = 0;
  if (*p == '.') {
    ++p;
    double scale = 0.1;
    while (isDigit(*p)) {
      value += (*p - '0') * scale;
      scale *= 0.1;
      ++fractionDigits;
      ++p;
    }
  }

  if (integerDigits + fractionDigits == 0) {
    return NumberResult::Syntax;
  }
  out = static_cast<float>(negative ? -value : value);
  cursor = p;
  return NumberResult::Ok;
}

ParseResult toParseResult(NumberResult result) {
  return result == NumberResult::TooLarge ? ParseResult::Value : ParseResult::Syntax;
}

// Parses "<pan> <tilt>" after the A letter.
ParseResult parseAimArguments(const char* cursor, Command& out) {
  float pan = 0.0f;
  float tilt = 0.0f;

  cursor = skipSpaces(cursor);
  NumberResult result = readDecimal(cursor, pan);
  if (result != NumberResult::Ok) {
    return toParseResult(result);
  }

  const char* afterPan = cursor;
  cursor = skipSpaces(cursor);
  if (cursor == afterPan) {
    return ParseResult::Syntax;  // the two numbers must be separated
  }

  result = readDecimal(cursor, tilt);
  if (result != NumberResult::Ok) {
    return toParseResult(result);
  }

  if (*skipSpaces(cursor) != '\0') {
    return ParseResult::Syntax;
  }
  out.type = CommandType::Aim;
  out.panDeg = pan;
  out.tiltDeg = tilt;
  return ParseResult::Ok;
}

// Parses the end of a command that takes no argument (F, S).
ParseResult parseBareCommand(const char* cursor, CommandType type, Command& out) {
  if (*skipSpaces(cursor) != '\0') {
    return ParseResult::Syntax;
  }
  out.type = type;
  out.panDeg = 0.0f;
  out.tiltDeg = 0.0f;
  return ParseResult::Ok;
}

}  // namespace

ParseResult parseCommand(const char* line, Command& out) {
  const char* cursor = skipSpaces(line);
  if (*cursor == '\0') {
    return ParseResult::Empty;
  }

  const char letter = *cursor++;
  switch (letter) {
    case 'A':
      return parseAimArguments(cursor, out);
    case 'F':
      return parseBareCommand(cursor, CommandType::Fire, out);
    case 'S':
      return parseBareCommand(cursor, CommandType::Status, out);
    default:
      return ParseResult::Unknown;
  }
}

const char* parseResultCode(ParseResult result) {
  switch (result) {
    case ParseResult::Ok:
      return "OK";
    case ParseResult::Empty:
      return "EMPTY";
    case ParseResult::Unknown:
      return "UNKNOWN";
    case ParseResult::Syntax:
      return "SYNTAX";
    case ParseResult::Value:
      return "VALUE";
  }
  return "SYNTAX";
}

LineAssembler::LineAssembler() { reset(); }

void LineAssembler::reset() {
  length_ = 0;
  discarding_ = false;
  buffer_[0] = '\0';
}

LineAssembler::Event LineAssembler::push(char byte) {
  if (byte == '\r') {
    return Event::None;  // tolerate CRLF line endings
  }

  if (byte == '\n') {
    const bool overflowed = discarding_;
    buffer_[length_] = '\0';
    length_ = 0;
    discarding_ = false;
    return overflowed ? Event::Overflow : Event::Line;
  }

  if (discarding_) {
    return Event::None;
  }
  if (length_ >= config::kMaxLineLength) {
    discarding_ = true;
    return Event::None;
  }
  buffer_[length_++] = byte;
  return Event::None;
}
