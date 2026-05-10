#include <string.h>
#include <unity.h>

#include "command_parser.h"

void setUp() {}
void tearDown() {}

namespace {

ParseResult parse(const char* line, Command& out) { return parseCommand(line, out); }

// Feeds a whole string to the assembler and returns the last event.
LineAssembler::Event pushAll(LineAssembler& assembler, const char* text) {
  LineAssembler::Event last = LineAssembler::Event::None;
  for (const char* c = text; *c != '\0'; ++c) {
    last = assembler.push(*c);
  }
  return last;
}

}  // namespace

void test_aim_command_with_integers() {
  Command cmd;
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse("A90 75", cmd));
  TEST_ASSERT_EQUAL(CommandType::Aim, cmd.type);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f, cmd.panDeg);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 75.0f, cmd.tiltDeg);
}

void test_aim_command_with_decimals_signs_and_spaces() {
  Command cmd;
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse("  A -12.5   +80.25  ", cmd));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -12.5f, cmd.panDeg);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 80.25f, cmd.tiltDeg);

  TEST_ASSERT_EQUAL(ParseResult::Ok, parse("A.5 90.", cmd));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, cmd.panDeg);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f, cmd.tiltDeg);
}

void test_aim_command_rejects_malformed_arguments() {
  Command cmd;
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A90", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A90 ", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A90,75", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A90 75 10", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A90 75x", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A- 75", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A1e2 75", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("Anan 75", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A. 75", cmd));
}

void test_aim_command_rejects_huge_numbers() {
  Command cmd;
  TEST_ASSERT_EQUAL(ParseResult::Value, parse("A1234567 90", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Value, parse("A90 9999999", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse("A123456 90", cmd));
}

void test_failed_parse_leaves_command_untouched() {
  Command cmd;
  cmd.type = CommandType::Status;
  cmd.panDeg = 1.0f;
  cmd.tiltDeg = 2.0f;
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("A45 x", cmd));
  TEST_ASSERT_EQUAL(CommandType::Status, cmd.type);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cmd.panDeg);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, cmd.tiltDeg);
}

void test_fire_and_status_commands() {
  Command cmd;
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse("F", cmd));
  TEST_ASSERT_EQUAL(CommandType::Fire, cmd.type);
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse(" S  ", cmd));
  TEST_ASSERT_EQUAL(CommandType::Status, cmd.type);
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("F1", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Syntax, parse("S now", cmd));
}

void test_empty_and_unknown_lines() {
  Command cmd;
  TEST_ASSERT_EQUAL(ParseResult::Empty, parse("", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Empty, parse("   ", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Unknown, parse("X", cmd));
  TEST_ASSERT_EQUAL(ParseResult::Unknown, parse("f", cmd));
}

void test_parse_result_codes() {
  TEST_ASSERT_EQUAL_STRING("UNKNOWN", parseResultCode(ParseResult::Unknown));
  TEST_ASSERT_EQUAL_STRING("SYNTAX", parseResultCode(ParseResult::Syntax));
  TEST_ASSERT_EQUAL_STRING("VALUE", parseResultCode(ParseResult::Value));
}

void test_line_assembler_handles_lf_and_crlf() {
  LineAssembler assembler;
  TEST_ASSERT_EQUAL(LineAssembler::Event::Line, pushAll(assembler, "A90 75\n"));
  TEST_ASSERT_EQUAL_STRING("A90 75", assembler.line());
  TEST_ASSERT_EQUAL(LineAssembler::Event::Line, pushAll(assembler, "S\r\n"));
  TEST_ASSERT_EQUAL_STRING("S", assembler.line());
}

void test_line_assembler_waits_for_newline() {
  LineAssembler assembler;
  TEST_ASSERT_EQUAL(LineAssembler::Event::None, pushAll(assembler, "A10"));
  TEST_ASSERT_EQUAL(LineAssembler::Event::Line, pushAll(assembler, " 20\n"));
  TEST_ASSERT_EQUAL_STRING("A10 20", assembler.line());
}

void test_line_assembler_accepts_maximum_length() {
  char line[config::kMaxLineLength + 2];
  memset(line, 'S', config::kMaxLineLength);
  line[config::kMaxLineLength] = '\n';
  line[config::kMaxLineLength + 1] = '\0';

  LineAssembler assembler;
  TEST_ASSERT_EQUAL(LineAssembler::Event::Line, pushAll(assembler, line));
  TEST_ASSERT_EQUAL(config::kMaxLineLength, strlen(assembler.line()));
}

void test_line_assembler_discards_overlong_line_then_recovers() {
  char line[config::kMaxLineLength + 3];
  memset(line, 'A', config::kMaxLineLength + 1);
  line[config::kMaxLineLength + 1] = '\n';
  line[config::kMaxLineLength + 2] = '\0';

  LineAssembler assembler;
  TEST_ASSERT_EQUAL(LineAssembler::Event::Overflow, pushAll(assembler, line));
  TEST_ASSERT_EQUAL(LineAssembler::Event::Line, pushAll(assembler, "F\n"));
  TEST_ASSERT_EQUAL_STRING("F", assembler.line());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_aim_command_with_integers);
  RUN_TEST(test_aim_command_with_decimals_signs_and_spaces);
  RUN_TEST(test_aim_command_rejects_malformed_arguments);
  RUN_TEST(test_aim_command_rejects_huge_numbers);
  RUN_TEST(test_failed_parse_leaves_command_untouched);
  RUN_TEST(test_fire_and_status_commands);
  RUN_TEST(test_empty_and_unknown_lines);
  RUN_TEST(test_parse_result_codes);
  RUN_TEST(test_line_assembler_handles_lf_and_crlf);
  RUN_TEST(test_line_assembler_waits_for_newline);
  RUN_TEST(test_line_assembler_accepts_maximum_length);
  RUN_TEST(test_line_assembler_discards_overlong_line_then_recovers);
  return UNITY_END();
}
