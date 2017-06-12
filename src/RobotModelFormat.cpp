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

pair<string, ROBOT_MODEL_FORMAT> kdl_parser::getRobotModelString(
        string const& model, ROBOT_MODEL_FORMAT format)
{
    char const* sdf_header  = "<sdf";
    char const* urdf_header = "<robot";
    if(model.find(sdf_header) != string::npos)
    {
        if (format == ROBOT_MODEL_URDF)
            throw std::invalid_argument("string contains the '<sdf' marker but the type was ROBOT_MODEL_SDF");
        return make_pair(model, ROBOT_MODEL_SDF);
    }
    else if(model.find(urdf_header) != string::npos)
    {
        if (format == ROBOT_MODEL_SDF)
            throw std::invalid_argument("string contains the '<urdf' marker but the type was ROBOT_MODEL_URDF");
        return make_pair(model, ROBOT_MODEL_URDF);
    }

    if (format == ROBOT_MODEL_AUTO)
        format = guessFormatFromFilename(model);

    ifstream file(model.c_str());
    if(!file)
        throw std::invalid_argument("Robot model file " + model + " does not exist");
    return std::make_pair(
            string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>()),
            format);
}

