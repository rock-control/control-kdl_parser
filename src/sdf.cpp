#include <base/Logging.hpp>
#include <kdl/tree.hpp>
#include "kdl_parser.hpp"
#include <sdf/sdf.hh>

using namespace std;
using ignition::math::Vector3d;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;

/*
 * SDF to KDL
 * bellow there are the source codes to convert SDFs information to KDL
 */
typedef map<string, sdf::ElementPtr> JointsMap;
typedef map<string, sdf::ElementPtr> LinksMap;
typedef map<string, vector<string> > LinkNamesMap;


/**
 * convert <axis> element to KDL::Vector
 */
static KDL::Vector toKdl(Vector3d axis)
{
    return KDL::Vector(axis.X(), axis.Y(), axis.Z());
}

static KDL::Rotation toKdl(Quaterniond rot)
{
    return KDL::Rotation::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W());
}
/**
 * convert <pose> element to KDL::Frame
 */
static KDL::Frame toKdl(Pose3d pose)
{
    KDL::Vector position = toKdl(pose.Pos());
    KDL::Rotation rotation = toKdl(pose.Rot());
    return KDL::Frame(rotation, position);
}

/**
 * convert <joint> element to KDL::Joint
 */
static KDL::Joint toKdl(string name, string type, KDL::Frame pose, KDL::Vector axis)
{
    if (type == "revolute"){
        return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::RotAxis);
    }
    else if (type == "prismatic"){
        return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::TransAxis);
    }

    return KDL::Joint(KDL::Joint::None);
}

/**
 * convert <inertial> element to KDL::RigidBodyInertia
 */
static KDL::RigidBodyInertia sdfInertiaToKdl(sdf::ElementPtr sdf)
{
    KDL::Frame pose = toKdl(sdf->GetElement("pose")->Get<Pose3d>());
    double mass;

    if (sdf->HasElement("mass")){
        mass = sdf->GetElement("mass")->Get<double>();
        if (sdf->HasElement("inertia")){

            sdf::ElementPtr sdf_inertia = sdf->GetElement("inertia");

            const char *tags[] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz" };

            double ixx, ixy, ixz, iyy, iyz, izz;
            double *ptr[] = {&ixx, &ixy, &ixz, &iyy, &iyz, &izz};

            for (int i = 0; i < 6; i++){
                (*ptr[i]) = sdf_inertia->GetElement(tags[i])->Get<double>();
            }

            return pose.M * KDL::RigidBodyInertia(mass, pose.p, KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz));
        }
    }

    return KDL::RigidBodyInertia();
}


/*
 * extract joint data
 */
static void sdfExtractJointData(sdf::ElementPtr sdf_joint,
                         string& joint_name,
                         string& joint_type,
                         KDL::Frame& joint_pose,
                         KDL::Vector& joint_axis,
                         bool& use_parent_model_frame)
{
    if (sdf_joint->HasAttribute("name")){
        joint_name = sdf_joint->Get<string>("name");
    }

    if (sdf_joint->HasAttribute("type")){
        joint_type = sdf_joint->Get<string>("type");
    }

    if (sdf_joint->HasElement("pose")){
        joint_pose = toKdl(sdf_joint->GetElement("pose")->Get<Pose3d>());
    }

    if (sdf_joint->HasElement("axis")){
        sdf::ElementPtr sdf_axis = sdf_joint->GetElement("axis");

        joint_axis = toKdl(sdf_joint->GetElement("axis")->GetElement("xyz")->Get<Vector3d>());

        if (sdf_axis->HasElement("use_parent_model_frame")){
            use_parent_model_frame = sdf_axis->GetElement("use_parent_model_frame")->Get<bool>();
        }

    }
}

/**
 * Fill KDL::Tree with SDF information
 */
static void fillKdlTreeFromSDF(LinkNamesMap linkNames,
                        JointsMap joints,
                        LinksMap links,
                        string parent_name,
                        string model_name,
                        KDL::Tree& tree)
{


    LinkNamesMap::iterator parentItr = linkNames.find(parent_name);

    if (parentItr != linkNames.end()){
        for (vector<string>::iterator childItr = parentItr->second.begin();
             childItr != parentItr->second.end(); childItr++) {

            LinksMap::iterator child_link_itr = links.find(*childItr);

            if (child_link_itr == links.end()){
                throw logic_error("cannot find link " + *childItr);
            }

            sdf::ElementPtr sdf_child_link = child_link_itr->second;

            KDL::Frame child2model;
            if (sdf_child_link->HasElement("pose")) {
                child2model = toKdl(sdf_child_link->GetElement("pose")->Get<ignition::math::Pose3d>());
            }

            KDL::RigidBodyInertia I;
            if (sdf_child_link->HasElement("inertial")){
                I = sdfInertiaToKdl(sdf_child_link->GetElement("inertial"));
            }

            string joint_type;
            string joint_name;
            KDL::Vector joint_axis;
            KDL::Frame joint2child;
            bool use_parent_model_frame = false;

            JointsMap::iterator joint_itr = joints.find(*childItr);

            if (joint_itr != joints.end()){
                sdfExtractJointData(joint_itr->second, joint_name, joint_type, joint2child, joint_axis, use_parent_model_frame);
            }

            LinksMap::iterator parent_link_itr = links.find(parent_name);

            KDL::Frame parent2model;
            if (parent_link_itr != links.end()){
                sdf::ElementPtr sdf_parent_link = parent_link_itr->second;

                if (sdf_parent_link->HasElement("pose")){
                    parent2model = toKdl(sdf_parent_link->GetElement("pose")->Get<ignition::math::Pose3d>());
                }
            }

            KDL::Frame child2parent = parent2model.Inverse() * child2model;
            KDL::Frame joint2parent = child2parent * joint2child;

            if (use_parent_model_frame){
                KDL::Frame joint2model = joint2child * child2model;
                joint_axis = joint2model.M.Inverse() * joint_axis;
            }

            KDL::Joint joint = toKdl(model_name + "::" + joint_name, joint_type, joint2parent, joint_axis);


            KDL::Segment segment(*childItr, joint,
                                            child2parent,
                                            I);

            tree.addSegment(segment, parent_name);

            fillKdlTreeFromSDF(linkNames, joints, links, *childItr, model_name, tree);
        }
    }
    else{
        throw logic_error("could not find link " + parent_name);
    }
}

/**
 * create a data structure to map <joint> elements
 * use link child as key to map refence the joints
 * if joint is a child then it has a joint
 */
static JointsMap sdfLoadJoints(string name_prefix, sdf::ElementPtr sdf_model)
{
    JointsMap joints;

    if (sdf_model->HasElement("joint")) {
        sdf::ElementPtr jointElem = sdf_model->GetElement("joint");

        while (jointElem) {
            string childLinkName = jointElem->GetElement("child")->Get<string>();
            joints.insert(make_pair(name_prefix + childLinkName, jointElem));
            jointElem = jointElem->GetNextElement("joint");
        }
    }

    return joints;
}

/**
 * create a data structure to map <link> elements
 * use link name as key to map the links
 */
static LinksMap sdfLoadLinks(string name_prefix, sdf::ElementPtr sdf_model)
{
    map<string, sdf::ElementPtr> links;

    if (sdf_model->HasElement("link")) {
        sdf::ElementPtr linkElem = sdf_model->GetElement("link");
        while (linkElem) {
            string linkName = linkElem->Get<string>("name");
            links.insert(make_pair(name_prefix + linkName, linkElem));
            linkElem = linkElem->GetNextElement("link");
        }
    }

    return links;
}

/**
 * create a structure to map parent and child
 * each parent has a list of children
 * this result is used to fill KDL::Tree
 */
static LinkNamesMap sdfBuildLinkNames(LinksMap links, JointsMap joints, string rootName)
{

    LinkNamesMap linkNames;
    LinksMap::iterator linksItr;
    JointsMap::iterator jointItr;

    //list each link and build and associate each child to its parent
    for (linksItr = links.begin(); linksItr != links.end(); linksItr++){
        string parent_name = rootName;
        string child_name  = linksItr->first;

        //check if child has a parent
        if ((jointItr = joints.find(child_name)) != joints.end()) {
            parent_name += "::" + jointItr->second->GetElement("parent")->Get<string>();
        }

        //insert child name in vector associated with parent name
        linkNames[parent_name].push_back(child_name);
        //make sure to register the child, even with an empty vector. There is a
        //consistency check in treeFromSdfString that requires it.
        linkNames[child_name];
    }

    return linkNames;

}

void kdl_parser::treeFromSdfModel(const sdf::ElementPtr& sdf_model, KDL::Tree& out)
{
    // model is the root segment
    string modelName = sdf_model->Get<string>("name");

    // map parents and children links
    LinksMap links = sdfLoadLinks(modelName + "::", sdf_model);

    // map links using link name
    JointsMap joints = sdfLoadJoints(modelName + "::", sdf_model);

    // map joints using child link names
    LinkNamesMap linkNames = sdfBuildLinkNames(links, joints, modelName);

    //build KDL::Tree using SDF information
    KDL::Tree tree(modelName);
    fillKdlTreeFromSDF(linkNames, joints, links, tree.getRootSegment()->first, modelName, tree);
    out = tree;
}

namespace kdl_parser {
/**
 * load kdl tree from sdf xml string
 */
bool treeFromSdfString(const string& xml, KDL::Tree& tree)
{
    sdf::SDFPtr sdf(new sdf::SDF);

    if (!sdf::init(sdf)){
        LOG_ERROR("unable to initialize sdf.");
        return false;
    }

    if (!sdf::readString(xml, sdf)){
        LOG_ERROR("unable to read xml string.");
        return false;
    }

    if (!sdf->Root()->HasElement("model")){
        LOG_ERROR("the <model> tag not exists");
        return false;
    }

    treeFromSdfModel(sdf->Root()->GetElement("model"), tree);
    return true;
}

/**
 * load kdl tree from sdf file
 */
bool treeFromSdfFile(const string& path, KDL::Tree& tree)
{
    ifstream file(path.c_str());
    string str((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    return treeFromSdfString(str, tree);
}

}
