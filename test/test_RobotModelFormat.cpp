#include <boost/test/unit_test.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "TestHelpers.hpp"

using namespace kdl_parser;
using namespace std;
using namespace KDL;

static const string SDF_STRING = "\
    <sdf version=\"1.6\">\
    <model name=\"test\" \\>\
    </sdf>";

static const string URDF_STRING = "\
    <robot name='urdf_robot'>\
      <link name='link' />\
    </robot>";

static const string RANDOM_STRING = "string with some content";

BOOST_AUTO_TEST_CASE(it_autodetects_a_sdf_xml_string_as_SDF)
{
    BOOST_REQUIRE(make_pair(SDF_STRING, ROBOT_MODEL_SDF) == getRobotModelString(SDF_STRING, ROBOT_MODEL_AUTO));
}

BOOST_AUTO_TEST_CASE(it_returns_a_sdf_xml_string_as_SDF_if_given_the_format_explicitely)
{
    BOOST_REQUIRE(make_pair(SDF_STRING, ROBOT_MODEL_SDF) == getRobotModelString(SDF_STRING, ROBOT_MODEL_SDF));
}

BOOST_AUTO_TEST_CASE(it_throws_if_given_something_that_looks_like_a_URDF_string_and_format_is_SDF)
{
    BOOST_REQUIRE_THROW(getRobotModelString(URDF_STRING, ROBOT_MODEL_SDF), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(it_autodetects_a_URDF_xml_string_as_URDF)
{
    BOOST_REQUIRE(make_pair(URDF_STRING, ROBOT_MODEL_URDF) == getRobotModelString(URDF_STRING, ROBOT_MODEL_AUTO));
}

BOOST_AUTO_TEST_CASE(it_returns_a_URDF_xml_string_as_URDF_if_given_the_format_explicitely)
{
    BOOST_REQUIRE(make_pair(URDF_STRING, ROBOT_MODEL_URDF) == getRobotModelString(URDF_STRING, ROBOT_MODEL_URDF));
}

BOOST_AUTO_TEST_CASE(it_throws_if_given_something_that_looks_like_a_SDF_string_and_format_is_URDF)
{
    BOOST_REQUIRE_THROW(getRobotModelString(SDF_STRING, ROBOT_MODEL_URDF), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(it_throws_if_the_file_does_not_exist)
{
    string path     = TEST_DATA_PATH("doesNotExist.urdf");
    BOOST_REQUIRE_THROW(getRobotModelString(path, ROBOT_MODEL_AUTO), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(it_automatically_detects_a_URDF_file_from_the_urdf_extension)
{
    string path     = TEST_DATA_PATH("getRobotModelString.urdf");
    string contents = readFile(path);
    BOOST_REQUIRE(make_pair(contents, ROBOT_MODEL_URDF) == getRobotModelString(path, ROBOT_MODEL_AUTO));
}

BOOST_AUTO_TEST_CASE(it_automatically_detects_a_SDF_file_from_the_sdf_extension)
{
    string path     = TEST_DATA_PATH("getRobotModelString.sdf");
    string contents = readFile(path);
    BOOST_REQUIRE(make_pair(contents, ROBOT_MODEL_SDF) == getRobotModelString(path, ROBOT_MODEL_AUTO));
}

BOOST_AUTO_TEST_CASE(it_automatically_detects_a_SDF_file_from_the_world_extension)
{
    string path     = TEST_DATA_PATH("getRobotModelString.world");
    string contents = readFile(path);
    BOOST_REQUIRE(make_pair(contents, ROBOT_MODEL_SDF) == getRobotModelString(path, ROBOT_MODEL_AUTO));
}

BOOST_AUTO_TEST_CASE(it_throws_if_the_extension_is_not_known_and_the_format_is_auto)
{
    string path     = TEST_DATA_PATH("getRobotModelString.anything");
    BOOST_REQUIRE_THROW(getRobotModelString(path, ROBOT_MODEL_AUTO), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(it_ignores_the_extension_if_explicitely_passed_URDF_as_model_format)
{
    string path     = TEST_DATA_PATH("getRobotModelString.anything");
    string contents = readFile(path);
    BOOST_REQUIRE(make_pair(contents, ROBOT_MODEL_URDF) == getRobotModelString(path, ROBOT_MODEL_URDF));
}

BOOST_AUTO_TEST_CASE(it_ignores_the_extension_if_explicitely_passed_SDF_as_model_format)
{
    string path     = TEST_DATA_PATH("getRobotModelString.anything");
    string contents = readFile(path);
    BOOST_REQUIRE(make_pair(contents, ROBOT_MODEL_SDF) == getRobotModelString(path, ROBOT_MODEL_SDF));
}

