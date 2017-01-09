#include <boost/test/unit_test.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace kdl_parser;
using namespace std;
using namespace KDL;

BOOST_AUTO_TEST_CASE(it_sets_the_root_segment_to_the_model_name_and_rigidly_links_the_root_link_to_it)
{
    string sdf = "<sdf version=\"1.6\"><model name=\"test\"><link name=\"root\" /></model></sdf>";
    Tree tree;
    BOOST_REQUIRE(treeFromString(sdf, tree, kdl_parser::ROBOT_MODEL_SDF));

    TreeElement const& treeRoot = tree.getRootSegment()->second;
    BOOST_REQUIRE_EQUAL("test", treeRoot.segment.getName());
    TreeElement const& rootLink = treeRoot.children[0]->second;
    BOOST_REQUIRE_EQUAL("test::root", rootLink.segment.getName());
}

BOOST_AUTO_TEST_CASE(it_discovers_the_children_of_the_root_link_and_links_them_to_it)
{
    string sdf = "\
        <sdf version=\"1.6\">\
        <model name=\"test\">\
            <link name=\"root\" />\
            <link name=\"child0\" />\
            <link name=\"child0_child\" />\
            <link name=\"child1\" />\
            \
            <joint name=\"j0\" type=\"fixed\">\
                <parent>root</parent>\
                <child>child0</child>\
            </joint>\
            <joint name=\"j1\" type=\"fixed\">\
                <parent>root</parent>\
                <child>child1</child>\
            </joint>\
            <joint name=\"j2\" type=\"fixed\">\
                <parent>child0</parent>\
                <child>child0_child</child>\
            </joint>\
        </model>\
        </sdf>";
    Tree tree;
    BOOST_REQUIRE(treeFromString(sdf, tree, kdl_parser::ROBOT_MODEL_SDF));
    TreeElement const& rootLink = tree.getRootSegment()->second.children[0]->second;
    BOOST_REQUIRE_EQUAL("test::root", rootLink.segment.getName());
    BOOST_REQUIRE_EQUAL(2, rootLink.children.size());

    TreeElement const& child0 = rootLink.children[0]->second;
    BOOST_REQUIRE_EQUAL("test::child0", child0.segment.getName());
    BOOST_REQUIRE_EQUAL("test::j0", child0.segment.getJoint().getName());
    TreeElement const& child1 = rootLink.children[1]->second;
    BOOST_REQUIRE_EQUAL("test::child1", child1.segment.getName());
    BOOST_REQUIRE_EQUAL("test::j1", child1.segment.getJoint().getName());

    BOOST_REQUIRE_EQUAL(1, child0.children.size());
    TreeElement const& child0_child = child0.children[0]->second;
    BOOST_REQUIRE_EQUAL("test::child0_child", child0_child.segment.getName());
    BOOST_REQUIRE_EQUAL("test::j2", child0_child.segment.getJoint().getName());
}

BOOST_AUTO_TEST_CASE(it_ignores_links_not_attached_to_the_first_link)
{
    string sdf = "\
        <sdf version=\"1.6\">\
        <model name=\"test\">\
            <link name=\"root\" />\
            <link name=\"other_root\" />\
            <link name=\"child\" />\
            \
            <joint name=\"j0\" type=\"fixed\">\
                <parent>other_root</parent>\
                <child>child</child>\
            </joint>\
        </model>\
        </sdf>";
    Tree tree;
    BOOST_REQUIRE(treeFromString(sdf, tree, kdl_parser::ROBOT_MODEL_SDF));
    BOOST_REQUIRE_EQUAL(1, tree.getNrOfSegments());
    BOOST_REQUIRE_EQUAL("test::root", tree.getRootSegment()->second.children[0]->second.segment.getName());
}

BOOST_AUTO_TEST_CASE(it_does_add_links_not_attached_to_the_first_link_if_the_model_is_static)
{
    string sdf = "\
        <sdf version=\"1.6\">\
        <model name=\"test\">\
            <link name=\"root\" />\
            <link name=\"other_root\" />\
            <link name=\"child\" />\
            \
            <joint name=\"j0\" type=\"fixed\">\
                <parent>other_root</parent>\
                <child>child</child>\
            </joint>\
            <static>true</static>\
        </model>\
        </sdf>";

    Tree tree;
    BOOST_REQUIRE(treeFromString(sdf, tree, kdl_parser::ROBOT_MODEL_SDF));
    BOOST_REQUIRE_EQUAL(3, tree.getNrOfSegments());
    BOOST_REQUIRE_EQUAL("test::root", tree.getRootSegment()->second.children[0]->second.segment.getName());
    BOOST_REQUIRE_EQUAL("test::other_root", tree.getRootSegment()->second.children[1]->second.segment.getName());
    BOOST_REQUIRE_EQUAL("test::child", tree.getRootSegment()->second.children[1]->second.children[0]->second.segment.getName());
}

