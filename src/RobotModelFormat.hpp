#ifndef KDL_PARSER_ROBOT_MODEL_FORMAT_HPP
#define KDL_PARSER_ROBOT_MODEL_FORMAT_HPP

#include <string>
#include <utility>

namespace kdl_parser{

//define format of robot model
enum ROBOT_MODEL_FORMAT
{
    //! try to guess the format
    ROBOT_MODEL_AUTO,
    //! URDF model
    ROBOT_MODEL_URDF,
    //! SDF model
    ROBOT_MODEL_SDF
};

/** Returns the model format based on a filename's extension
 */
ROBOT_MODEL_FORMAT guessFormatFromFilename(const std::string& file);

/** Returns the name of one of the ROBOT_MODEL_* format IDs
 */
const char* formatNameFromID(int type);

#endif
