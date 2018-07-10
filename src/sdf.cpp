#include <base-logging/Logging.hpp>
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
typedef map<string, vector<string> > LinksChildrenMap;

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
static KDL::Joint toKdl(string name, string type, KDL::Frame pose, KDL::Vector axis, double joint_upper, double joint_lower)
{
    if (type == "revolute"){
        return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::RotAxis, 1, 0, 0, 0, 0, joint_upper, joint_lower);
    }
    else if (type == "prismatic"){
        return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::TransAxis, 1, 0, 0, 0, 0, joint_upper, joint_lower);
    }
    else if (type == "fixed"){
        return KDL::Joint(name, KDL::Joint::None, 1, 0, 0, 0, 0, joint_upper, joint_lower);
    }
    else
        throw runtime_error("cannot handle joint type " + type);

}

static bool sdfIsModelStatic(const sdf::ElementPtr sdf_model)
{
    sdf::ElementPtr elementStatic = sdf_model->GetElement("static");
    if (elementStatic)
        return elementStatic->Get<bool>();
    else
        return false;
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
                         double& joint_upper,
                         double& joint_lower,
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

        if (sdf_axis->HasElement("limit")){
            sdf::ElementPtr limit_elem = sdf_axis->GetElement("limit");

            if (limit_elem->HasElement("upper"))
                joint_upper = limit_elem->Get<double>("upper");
            if (limit_elem->HasElement("lower"))
                joint_lower = limit_elem->Get<double>("lower");
        }

    }
}

static KDL::Frame getLinkPose(sdf::ElementPtr sdf_link)
{
    if (sdf_link->HasElement("pose"))
        return toKdl(sdf_link->GetElement("pose")->Get<Pose3d>());
    else
        return KDL::Frame();
}

/**
 * Fill KDL::Tree with SDF information
 */
static void convertSdfTree(
        LinksChildrenMap const& linksChildren,
        JointsMap const& joints,
        LinksMap const& links,
        sdf::ElementPtr sdf_root_link,
        string const& model_name,
        KDL::Tree& tree)
{
    string root_link_name = model_name + "::" + sdf_root_link->Get<string>("name");

    KDL::RigidBodyInertia I;
    if (sdf_root_link->HasElement("inertial")){
        I = sdfInertiaToKdl(sdf_root_link->GetElement("inertial"));
    }

    KDL::Segment segment(root_link_name,
            KDL::Joint(root_link_name, KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Identity(), KDL::Vector::Zero()), I);
    tree.addSegment(segment, model_name);

    KDL::Frame root2model = getLinkPose(sdf_root_link);

    LinksChildrenMap::const_iterator children_itr = linksChildren.find(root_link_name);
    if (children_itr == linksChildren.end())
        throw logic_error("cannot find parent link " + root_link_name);

    vector<string> const& children = children_itr->second;
    for (vector<string>::const_iterator child_itr = children.begin(); child_itr != children.end(); child_itr++) {
        string child_link_name = *child_itr;

        LinksMap::const_iterator child_link_itr = links.find(child_link_name);
        if (child_link_itr == links.end())
            throw logic_error("cannot find child link " + child_link_name + ", which was expected to be a child of " + root_link_name);
        sdf::ElementPtr sdf_child_link = child_link_itr->second;

        JointsMap::const_iterator joint_itr = joints.find(child_link_name);
        if (joint_itr == joints.end())
            throw logic_error("cannot find joint that attaches " + child_link_name + " to " + root_link_name);
        sdf::ElementPtr sdf_joint = joint_itr->second;

        KDL::Frame child2model;
        if (sdf_child_link->HasElement("pose")) {
            child2model = toKdl(sdf_child_link->GetElement("pose")->Get<Pose3d>());
        }

        KDL::RigidBodyInertia I;
        if (sdf_child_link->HasElement("inertial")){
            I = sdfInertiaToKdl(sdf_child_link->GetElement("inertial"));
        }

        string joint_type;
        string joint_name;
        KDL::Vector joint_axis;
        double joint_upper;
        double joint_lower;
        KDL::Frame joint2child;
        bool use_parent_model_frame = false;

        sdfExtractJointData(sdf_joint, joint_name, joint_type, joint2child, joint_axis, joint_upper, joint_lower, use_parent_model_frame);

        KDL::Frame child2parent = root2model.Inverse() * child2model;
        KDL::Frame joint2parent = child2parent * joint2child;

        if (use_parent_model_frame){
            KDL::Frame joint2model = joint2child * child2model;
            joint_axis = joint2model.M.Inverse() * joint_axis;
        }

        KDL::Joint joint = toKdl(model_name + "::" + joint_name, joint_type, joint2parent, joint_axis, joint_upper, joint_lower);
        KDL::Segment segment(child_link_name, joint, child2parent, I);
        if (! tree.addSegment(segment, root_link_name))
            throw std::logic_error("failed to add segment " + child_link_name + " as child of " + root_link_name);

        convertSdfTree(linksChildren, joints, links, sdf_child_link, model_name, tree);
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
static LinksChildrenMap sdfBuildLinksChildrenMap(LinksMap const& links, JointsMap const& joints, string rootName)
{

    LinksChildrenMap children;
    LinksMap::const_iterator linksItr;
    JointsMap::const_iterator jointItr;

    //list each link and build and associate each child to its parent
    for (linksItr = links.begin(); linksItr != links.end(); linksItr++){
        string parent_name = rootName;
        string child_name  = linksItr->first;

        //check if child has a parent
        if ((jointItr = joints.find(child_name)) != joints.end()) {
            parent_name += "::" + jointItr->second->GetElement("parent")->Get<string>();
        }

        //insert child name in vector associated with parent name
        children[parent_name].push_back(child_name);
        //make sure to register the child, even with an empty vector. There is a
        //consistency check in treeFromSdfString that requires it.
        children[child_name];
    }

    return children;

}

void kdl_parser::treeFromSdfModel(const sdf::ElementPtr& sdf_model, KDL::Tree& out)
{
    // model is the root segment
    string model_name = sdf_model->Get<string>("name");

    // map parents and children links
    LinksMap links = sdfLoadLinks(model_name + "::", sdf_model);

    // map links using link name
    JointsMap joints = sdfLoadJoints(model_name + "::", sdf_model);

    // map joints using child link names
    LinksChildrenMap children = sdfBuildLinksChildrenMap(links, joints, model_name);

    //build KDL::Tree using SDF information
    KDL::Tree tree(model_name);

    if (!sdf_model->HasElement("link"))
    {
        out = tree;
        return;
    }

    bool static_model = sdfIsModelStatic(sdf_model);
    sdf::ElementPtr sdf_root_link = sdf_model->GetElement("link");
    if (static_model) {
        while (sdf_root_link) {
            string root_name = model_name + "::" + sdf_root_link->Get<string>("name");
            if (tree.getSegment(root_name) == tree.getSegments().end())
                convertSdfTree(children, joints, links, sdf_root_link, model_name, tree);
            sdf_root_link = sdf_root_link->GetNextElement("link");
        }
    }
    else {
        string root_name = model_name + "::" + sdf_root_link->Get<string>("name");
        convertSdfTree(children, joints, links, sdf_root_link, model_name, tree);
    }

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
