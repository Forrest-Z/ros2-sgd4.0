// Copyright 2021 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "gtest/gtest.h"

#include "include/nmea_parser.hpp"

static std::string BASE_PATH("");
static const double error = 1/1E6;

TEST(NmeaParserTest, ImportXmlOk)
{
  const std::string xml_file = BASE_PATH + "/nmea_ok.xml";
  sgd_hardware::Nmea_Parser parser;
  parser.import_xml(xml_file);

  EXPECT_FALSE(parser.has_error()) << parser.get_last_error().to_string();
}

TEST(NmeaParserTest, ImportXmlBadFormat)
{
  const std::string xml_file = BASE_PATH + "/nmea_bad.xml";
  sgd_hardware::Nmea_Parser parser;
  parser.import_xml(xml_file);

  EXPECT_TRUE(parser.has_error());
}

TEST(NmeaParserTest, ParseNmeaMsgOk)
{
  const std::string xml_file = BASE_PATH + "/nmea_ok.xml";
  sgd_hardware::Nmea_Parser parser;
  parser.import_xml(xml_file);

  ASSERT_FALSE(parser.has_error()) << parser.get_last_error().to_string();

  std::string nmea_msg_ok = "$GPGGA,105353.000,5340.9773,N,00943.7816,E,1,08,0.9,15.2,M,45.1,M,,0000*64";
  parser.parse_msg(nmea_msg_ok);

  ASSERT_FALSE(parser.has_error()) << parser.get_last_error().to_string();

  EXPECT_NEAR(parser.time(), 39233.0, error);
  EXPECT_NEAR(parser.latitude(), 53.6829550, error);
  EXPECT_NEAR(parser.longitude(), 9.7296933, error);
  double hdop = std::get<double>(parser.get_data("hdop").first);
  EXPECT_NEAR(hdop, 0.9, error);
}

int main(int argc, char **argv)
{
  auto exe = std::string(argv[0]);
  BASE_PATH = exe.substr(0,exe.find_last_of("/")) + "/test/data";

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}