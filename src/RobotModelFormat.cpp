#include <kdl_parser/RobotModelFormat.hpp>
#include <stdexcept>
#include <fstream>
#include <base-logging/Logging.hpp>

using namespace kdl_parser;
using namespace std;

const char* kdl_parser::formatNameFromID(int type)
{
    switch(type)
    {
        case ROBOT_MODEL_AUTO: return "autodetected";
        case ROBOT_MODEL_URDF: return "URDF";
        case ROBOT_MODEL_SDF: return "SDF";
        default:
            throw std::invalid_argument("invalid model type ID given");
    }
}

ROBOT_MODEL_FORMAT kdl_parser::guessFormatFromFilename(const std::string& file)
{
    if (file.substr(file.size() - 4) == ".sdf")
        return ROBOT_MODEL_SDF;
    if (file.substr(file.size() - 5) == ".urdf")
        return ROBOT_MODEL_URDF;
    if (file.substr(file.size() - 6) == ".world")
        return ROBOT_MODEL_SDF;
    throw std::invalid_argument("cannot guess robot model format for " + file);
}

