#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "Utils/String.h"
#include "Utils/Path.h"
#include "Utils/Matrix.h"
#include "Utils/Vector3d.h"
#include "Utils/RotoTrans.h"
#include "Utils/RotoTransNode.h"
#include "Utils/Rotation.h"
#include "Utils/Quaternion.h"

#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/NodeSegment.h"
static double requiredPrecision(1e-10);

TEST(ShallowCopy, DeepCopy)
{
    // DeepCopying a shallow copy, should also change the reference
    // Warning that may be surprising because one may be tend to DeepCopy
    // itself afterward, this doesn't release the shallowcopy referencing
    biorbd::utils::Vector3d MainNode(0, 0, 0, "NoName", "NoParent");
    biorbd::utils::Vector3d ShallowToDeep(MainNode);
    biorbd::utils::Vector3d NewNode(0, 0, 0, "MyName", "MyParent");
    EXPECT_STREQ(MainNode.biorbd::utils::Node::name().c_str(), "NoName");
    EXPECT_STREQ(ShallowToDeep.biorbd::utils::Node::name().c_str(), "NoName");
    EXPECT_STREQ(NewNode.biorbd::utils::Node::name().c_str(), "MyName");
    ShallowToDeep.setName("StillNoName");
    EXPECT_STREQ(MainNode.biorbd::utils::Node::name().c_str(), "StillNoName");
    EXPECT_STREQ(ShallowToDeep.biorbd::utils::Node::name().c_str(), "StillNoName");
    EXPECT_STREQ(NewNode.biorbd::utils::Node::name().c_str(), "MyName");
    ShallowToDeep.DeepCopy(NewNode);
    EXPECT_STREQ(MainNode.biorbd::utils::Node::name().c_str(), "MyName");
    EXPECT_STREQ(ShallowToDeep.biorbd::utils::Node::name().c_str(), "MyName");
    EXPECT_STREQ(NewNode.biorbd::utils::Node::name().c_str(), "MyName");
    ShallowToDeep.setName("BackToNoName");
    EXPECT_STREQ(MainNode.biorbd::utils::Node::name().c_str(), "BackToNoName");
    EXPECT_STREQ(ShallowToDeep.biorbd::utils::Node::name().c_str(), "BackToNoName");
    EXPECT_STREQ(NewNode.biorbd::utils::Node::name().c_str(), "MyName");
}

TEST(String, conversions)
{
    biorbd::utils::String str(biorbd::utils::String::to_string(M_PI));
    double pi = std::stod(str);
    EXPECT_EQ(pi, M_PI);
}

TEST(Path, Create)
{
    {
        biorbd::utils::Path emptyPath;
        EXPECT_STREQ(emptyPath.absolutePath().c_str(),
                     biorbd::utils::Path::currentDir().c_str());
        EXPECT_STREQ(emptyPath.absoluteFolder().c_str(),
                     biorbd::utils::Path::currentDir().c_str());
        EXPECT_STREQ(emptyPath.filename().c_str(), "");
        EXPECT_STREQ(emptyPath.extension().c_str(), "");
        EXPECT_STREQ(emptyPath.originalPath().c_str(), "");
    }

    {
        EXPECT_STREQ(
            biorbd::utils::Path::toUnixFormat(
                "MyUgly\\\\WindowsPath\\\\ToMyFile.biorbd").c_str(),
            biorbd::utils::String (
                "MyUgly/WindowsPath/ToMyFile.biorbd").c_str());

        EXPECT_STREQ(
            biorbd::utils::Path::toWindowsFormat(
                "MyCute/UnixPath/ToMyFile.biorbd").c_str(),
            biorbd::utils::String (
                "MyCute\\\\UnixPath\\\\ToMyFile.biorbd").c_str());
    }

    {
        biorbd::utils::String myPathInUglyWindowsStyleString(
            "MyUgly\\\\WindowsPath\\\\ToMyFile.biorbd");
        biorbd::utils::Path myPathInUglyWindowsStyle(
            myPathInUglyWindowsStyleString);
        EXPECT_STREQ(myPathInUglyWindowsStyle.relativePath().c_str(),
                     "./MyUgly/WindowsPath/ToMyFile.biorbd");
        EXPECT_STREQ(myPathInUglyWindowsStyle.originalPath().c_str(),
                     myPathInUglyWindowsStyleString.c_str());
    }

    {
#ifdef _WIN32
        biorbd::utils::String path(
            "C:\\MyLovely\\AbsolutePath\\ToMyLovelyFile.biorbd");
        biorbd::utils::String unixPath(
            "C:/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
        biorbd::utils::String absoluteUnixFolder("C:/MyLovely/AbsolutePath/");
#else
        biorbd::utils::String path(
            "/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
        biorbd::utils::String unixPath(
            "/MyLovely/AbsolutePath/ToMyLovelyFile.biorbd");
        biorbd::utils::String absoluteUnixFolder("/MyLovely/AbsolutePath/");
#endif

        biorbd::utils::Path absolutePath(path);
        EXPECT_STREQ(absolutePath.absolutePath().c_str(), unixPath.c_str());
        EXPECT_STREQ(absolutePath.absoluteFolder().c_str(), absoluteUnixFolder.c_str());
        EXPECT_STREQ(absolutePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(absolutePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(absolutePath.originalPath().c_str(), path.c_str());
    }

    {
        biorbd::utils::Path relativePath("MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(relativePath.absolutePath().c_str(),
                     (biorbd::utils::Path::currentDir() +
                      "MyLovely/RelativePath/ToMyLovelyFile.biorbd").c_str());
        EXPECT_STREQ(relativePath.absoluteFolder().c_str(),
                     (biorbd::utils::Path::currentDir() + "MyLovely/RelativePath/").c_str());
        EXPECT_STREQ(relativePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(relativePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(relativePath.relativePath().c_str(),
                     "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(relativePath.originalPath().c_str(),
                     "MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    }

    {
        biorbd::utils::Path
        weirdRelativePath("./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(weirdRelativePath.absolutePath().c_str(),
                     (biorbd::utils::Path::currentDir() +
                      "MyLovely/RelativePath/ToMyLovelyFile.biorbd").c_str());
        EXPECT_STREQ(weirdRelativePath.absoluteFolder().c_str(),
                     (biorbd::utils::Path::currentDir() + "MyLovely/RelativePath/").c_str());
        EXPECT_STREQ(weirdRelativePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(weirdRelativePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(weirdRelativePath.relativePath().c_str(),
                     "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(weirdRelativePath.originalPath().c_str(),
                     "./MyLovely/RelativePath/ToMyLovelyFile.biorbd");
    }

    {
        biorbd::utils::Path
        parentRelativePath("../MyLovely/ParentPath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(parentRelativePath.absolutePath().c_str(),
                     (biorbd::utils::Path::currentDir() +
                      "../MyLovely/ParentPath/ToMyLovelyFile.biorbd").c_str());
        EXPECT_STREQ(parentRelativePath.absoluteFolder().c_str(),
                     (biorbd::utils::Path::currentDir() + "../MyLovely/ParentPath/").c_str());
        EXPECT_STREQ(parentRelativePath.filename().c_str(), "ToMyLovelyFile");
        EXPECT_STREQ(parentRelativePath.extension().c_str(), "biorbd");
        EXPECT_STREQ(parentRelativePath.relativePath().c_str(),
                     "../MyLovely/ParentPath/ToMyLovelyFile.biorbd");
        EXPECT_STREQ(parentRelativePath.originalPath().c_str(),
                     "../MyLovely/ParentPath/ToMyLovelyFile.biorbd");
    }

    {
        biorbd::utils::Path noPath("MyLonelyFile.biorbd");
        EXPECT_STREQ(noPath.absolutePath().c_str(),
                     (biorbd::utils::Path::currentDir() + "MyLonelyFile.biorbd").c_str());
        EXPECT_STREQ(noPath.absoluteFolder().c_str(),
                     biorbd::utils::Path::currentDir().c_str());
        EXPECT_STREQ(noPath.filename().c_str(), "MyLonelyFile");
        EXPECT_STREQ(noPath.extension().c_str(), "biorbd");
        EXPECT_STREQ(noPath.relativePath().c_str(), "./MyLonelyFile.biorbd");
        EXPECT_STREQ(noPath.originalPath().c_str(), "MyLonelyFile.biorbd");
    }

    {
        biorbd::utils::Path almostNoPath("./MyKinbDofLonelyFile.biorbd");
        EXPECT_STREQ(almostNoPath.absolutePath().c_str(),
                     (biorbd::utils::Path::currentDir() + "MyKinbDofLonelyFile.biorbd").c_str());
        EXPECT_STREQ(almostNoPath.absoluteFolder().c_str(),
                     biorbd::utils::Path::currentDir().c_str());
        EXPECT_STREQ(almostNoPath.filename().c_str(), "MyKinbDofLonelyFile");
        EXPECT_STREQ(almostNoPath.extension().c_str(), "biorbd");
        EXPECT_STREQ(almostNoPath.relativePath().c_str(),
                     "./MyKinbDofLonelyFile.biorbd");
        EXPECT_STREQ(almostNoPath.originalPath().c_str(),
                     "./MyKinbDofLonelyFile.biorbd");
    }
}

TEST(Path, Copy)
{
    biorbd::utils::Path MainPath("MyLovelyPath.biorbd");
    biorbd::utils::Path ShallowCopy(MainPath);
    biorbd::utils::Path DeepCopyNow(MainPath.DeepCopy());
    biorbd::utils::Path DeepCopyLater;
    DeepCopyLater.DeepCopy(MainPath);

    EXPECT_STREQ(MainPath.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyNow.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(MainPath.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyNow.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.originalPath().c_str(), "MyLovelyPath.biorbd");

    // Changing the ShallowCopy should change the Main, but not the Deep
    ShallowCopy.setFilename("MySecondLovelyPath");
    ShallowCopy.setExtension("newExt");

    EXPECT_STREQ(MainPath.relativePath().c_str(), "./MySecondLovelyPath.newExt");
    EXPECT_STREQ(ShallowCopy.relativePath().c_str(), "./MySecondLovelyPath.newExt");
    EXPECT_STREQ(DeepCopyNow.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.relativePath().c_str(), "./MyLovelyPath.biorbd");
    EXPECT_STREQ(MainPath.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(ShallowCopy.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyNow.originalPath().c_str(), "MyLovelyPath.biorbd");
    EXPECT_STREQ(DeepCopyLater.originalPath().c_str(), "MyLovelyPath.biorbd");
}

TEST(Vector3d, rotate)
{
    biorbd::utils::Vector3d node(2, 3, 4);
    biorbd::utils::Vector3d nodeXrot(node.DeepCopy());
    SCALAR_TO_DOUBLE(x, node[0]);
    SCALAR_TO_DOUBLE(y, node[1]);
    SCALAR_TO_DOUBLE(z, node[2]);
    nodeXrot.applyRT( biorbd::utils::RotoTrans(biorbd::utils::Vector3d(M_PI, 0, 0),
                      biorbd::utils::Vector3d(0, 0, 0),
                      "xyz") );
    {
        SCALAR_TO_DOUBLE(nodeX, nodeXrot[0]);
        SCALAR_TO_DOUBLE(nodeY, nodeXrot[1]);
        SCALAR_TO_DOUBLE(nodeZ, nodeXrot[2]);
        EXPECT_NEAR(nodeX, x, requiredPrecision);
        EXPECT_NEAR(nodeY, -y, requiredPrecision);
        EXPECT_NEAR(nodeZ, -z, requiredPrecision);
    }

    biorbd::utils::Vector3d nodeYrot(node.DeepCopy());
    nodeYrot.applyRT( biorbd::utils::RotoTrans(biorbd::utils::Vector3d(0, M_PI, 0),
                      biorbd::utils::Vector3d(0, 0, 0),
                      "xyz") );
    {
        SCALAR_TO_DOUBLE(nodeX, nodeYrot[0]);
        SCALAR_TO_DOUBLE(nodeY, nodeYrot[1]);
        SCALAR_TO_DOUBLE(nodeZ, nodeYrot[2]);
        EXPECT_NEAR(nodeX, -x, requiredPrecision);
        EXPECT_NEAR(nodeY, y, requiredPrecision);
        EXPECT_NEAR(nodeZ, -z, requiredPrecision);
    }

    biorbd::utils::Vector3d nodeZrot(node.DeepCopy());
    nodeZrot.applyRT( biorbd::utils::RotoTrans(biorbd::utils::Vector3d(0, 0, M_PI),
                      biorbd::utils::Vector3d(0, 0, 0),
                      "xyz") );
    {
        SCALAR_TO_DOUBLE(nodeX, nodeZrot[0]);
        SCALAR_TO_DOUBLE(nodeY, nodeZrot[1]);
        SCALAR_TO_DOUBLE(nodeZ, nodeZrot[2]);
        EXPECT_NEAR(nodeX, -x, requiredPrecision);
        EXPECT_NEAR(nodeY, -y, requiredPrecision);
        EXPECT_NEAR(nodeZ, z, requiredPrecision);
    }

    biorbd::utils::Vector3d nodeZrot2(node.DeepCopy());
    nodeZrot2.applyRT( biorbd::utils::RotoTrans(biorbd::utils::Vector3d(M_PI, 0, 0),
                       biorbd::utils::Vector3d(0, 0, 0),
                       "zxy") );
    {
        SCALAR_TO_DOUBLE(nodeX, nodeZrot2[0]);
        SCALAR_TO_DOUBLE(nodeY, nodeZrot2[1]);
        SCALAR_TO_DOUBLE(nodeZ, nodeZrot2[2]);
        EXPECT_NEAR(nodeX, -x, requiredPrecision);
        EXPECT_NEAR(nodeY, -y, requiredPrecision);
        EXPECT_NEAR(nodeZ, z, requiredPrecision);
    }

    double trans(2);
    biorbd::utils::Vector3d nodeRot(node.DeepCopy());
    nodeRot.applyRT( biorbd::utils::RotoTrans(biorbd::utils::Vector3d(M_PI/6,
                     M_PI/6, M_PI/6),
                     biorbd::utils::Vector3d(trans, trans, trans), "xyz") );
    {
        SCALAR_TO_DOUBLE(nodeX, nodeRot[0]);
        SCALAR_TO_DOUBLE(nodeY, nodeRot[1]);
        SCALAR_TO_DOUBLE(nodeZ, nodeRot[2]);
        EXPECT_NEAR(nodeX, 4.200961894323342, requiredPrecision);
        EXPECT_NEAR(nodeY, 3.4419872981077808, requiredPrecision);
        EXPECT_NEAR(nodeZ, 6.698557158514987, requiredPrecision);
    }

    biorbd::utils::Vector3d nodeTrans(node.DeepCopy());
    nodeTrans.applyRT( biorbd::utils::RotoTrans(biorbd::utils::Vector3d(0, 0, 0),
                       biorbd::utils::Vector3d(2, 2, 2), "xyz") );
    {
        SCALAR_TO_DOUBLE(nodeX, nodeTrans[0]);
        SCALAR_TO_DOUBLE(nodeY, nodeTrans[1]);
        SCALAR_TO_DOUBLE(nodeZ, nodeTrans[2]);
        EXPECT_NEAR(nodeX, x + 2., requiredPrecision);
        EXPECT_NEAR(nodeY, y + 2., requiredPrecision);
        EXPECT_NEAR(nodeZ, z + 2., requiredPrecision);
    }
}

TEST(Vector3d, Copy)
{
    biorbd::utils::Vector3d MainNode(1, 2, 3, "MainNodeName", "NoParent");
    biorbd::utils::Vector3d ShallowCopy(MainNode);
    biorbd::utils::Vector3d DeepCopyNow(MainNode.DeepCopy());
    biorbd::utils::Vector3d DeepCopyLater;
    DeepCopyLater.DeepCopy(MainNode);

    EXPECT_STREQ(MainNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the values
    {
        SCALAR_TO_DOUBLE(MainNodeX, MainNode.x());
        SCALAR_TO_DOUBLE(MainNodeY, MainNode.y());
        SCALAR_TO_DOUBLE(MainNodeZ, MainNode.z());
        EXPECT_NEAR(MainNodeX, 1, requiredPrecision);
        EXPECT_NEAR(MainNodeY, 2, requiredPrecision);
        EXPECT_NEAR(MainNodeZ, 3, requiredPrecision);
        SCALAR_TO_DOUBLE(ShallowCopyX, ShallowCopy.x());
        SCALAR_TO_DOUBLE(ShallowCopyY, ShallowCopy.y());
        SCALAR_TO_DOUBLE(ShallowCopyZ, ShallowCopy.z());
        EXPECT_NEAR(ShallowCopyX, 1, requiredPrecision);
        EXPECT_NEAR(ShallowCopyY, 2, requiredPrecision);
        EXPECT_NEAR(ShallowCopyZ, 3, requiredPrecision);
        SCALAR_TO_DOUBLE(DeepCopyNowX, DeepCopyNow.x());
        SCALAR_TO_DOUBLE(DeepCopyNowY, DeepCopyNow.y());
        SCALAR_TO_DOUBLE(DeepCopyNowZ, DeepCopyNow.z());
        EXPECT_NEAR(DeepCopyNowX, 1, requiredPrecision);
        EXPECT_NEAR(DeepCopyNowY, 2, requiredPrecision);
        EXPECT_NEAR(DeepCopyNowZ, 3, requiredPrecision);
        SCALAR_TO_DOUBLE(DeepCopyLaterX, DeepCopyLater.x());
        SCALAR_TO_DOUBLE(DeepCopyLaterY, DeepCopyLater.y());
        SCALAR_TO_DOUBLE(DeepCopyLaterZ, DeepCopyLater.z());
        EXPECT_NEAR(DeepCopyLaterX, 1, requiredPrecision);
        EXPECT_NEAR(DeepCopyLaterY, 2, requiredPrecision);
        EXPECT_NEAR(DeepCopyLaterZ, 3, requiredPrecision);
    }
    // Change the values of ShallowCopy
    ShallowCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    {
        SCALAR_TO_DOUBLE(MainNodeX, MainNode.x());
        SCALAR_TO_DOUBLE(MainNodeY, MainNode.y());
        SCALAR_TO_DOUBLE(MainNodeZ, MainNode.z());
        EXPECT_NEAR(MainNodeX, 1, requiredPrecision);
        EXPECT_NEAR(MainNodeY, 2, requiredPrecision);
        EXPECT_NEAR(MainNodeZ, 3, requiredPrecision);
        SCALAR_TO_DOUBLE(ShallowCopyX, ShallowCopy.x());
        SCALAR_TO_DOUBLE(ShallowCopyY, ShallowCopy.y());
        SCALAR_TO_DOUBLE(ShallowCopyZ, ShallowCopy.z());
        EXPECT_NEAR(ShallowCopyX, 0, requiredPrecision);
        EXPECT_NEAR(ShallowCopyY, 0, requiredPrecision);
        EXPECT_NEAR(ShallowCopyZ, 0, requiredPrecision);
        SCALAR_TO_DOUBLE(DeepCopyNowX, DeepCopyNow.x());
        SCALAR_TO_DOUBLE(DeepCopyNowY, DeepCopyNow.y());
        SCALAR_TO_DOUBLE(DeepCopyNowZ, DeepCopyNow.z());
        EXPECT_NEAR(DeepCopyNowX, 1, requiredPrecision);
        EXPECT_NEAR(DeepCopyNowY, 2, requiredPrecision);
        EXPECT_NEAR(DeepCopyNowZ, 3, requiredPrecision);
        SCALAR_TO_DOUBLE(DeepCopyLaterX, DeepCopyLater.x());
        SCALAR_TO_DOUBLE(DeepCopyLaterY, DeepCopyLater.y());
        SCALAR_TO_DOUBLE(DeepCopyLaterZ, DeepCopyLater.z());
        EXPECT_NEAR(DeepCopyLaterX, 1, requiredPrecision);
        EXPECT_NEAR(DeepCopyLaterY, 2, requiredPrecision);
        EXPECT_NEAR(DeepCopyLaterZ, 3, requiredPrecision);
    }
}

TEST(Matrix, Copy)
{
    biorbd::utils::Matrix MainMatrix(4, 4);
    FILL_MATRIX(MainMatrix, std::vector<double>({0, 1, 2, 3,
                4, 5, 6, 7,
                8, 9, 10, 11,
                12, 13, 14, 15
                                                }));
    biorbd::utils::Matrix DeepCopy(MainMatrix);

    // Test for the values
    {
        SCALAR_TO_DOUBLE(mainMat, MainMatrix(0, 0));
        SCALAR_TO_DOUBLE(deepCopy, DeepCopy(0, 0));
        EXPECT_NEAR(mainMat, 0, requiredPrecision);
        EXPECT_NEAR(deepCopy, 0, requiredPrecision);
    }
    {
        SCALAR_TO_DOUBLE(mainMat, MainMatrix(1, 0));
        SCALAR_TO_DOUBLE(deepCopy, DeepCopy(1, 0));
        EXPECT_NEAR(mainMat, 4, requiredPrecision);
        EXPECT_NEAR(deepCopy, 4, requiredPrecision);
    }
    {
        SCALAR_TO_DOUBLE(mainMat, MainMatrix(0, 1));
        SCALAR_TO_DOUBLE(deepCopy, DeepCopy(0, 1));
        EXPECT_NEAR(mainMat, 1, requiredPrecision);
        EXPECT_NEAR(deepCopy, 1, requiredPrecision);
    }
    {
        SCALAR_TO_DOUBLE(mainMat, MainMatrix(1, 1));
        SCALAR_TO_DOUBLE(deepCopy, DeepCopy(1, 1));
        EXPECT_NEAR(mainMat, 5, requiredPrecision);
        EXPECT_NEAR(deepCopy, 5, requiredPrecision);
    }

    // Change the values of Copy
    DeepCopy.setZero();

    // Data are NOT shallow copy, therefore the parent should keep its values
    {
        SCALAR_TO_DOUBLE(mainMat, MainMatrix(2, 2));
        SCALAR_TO_DOUBLE(deepCopy, DeepCopy(2, 2));
        EXPECT_NEAR(mainMat, 10, requiredPrecision);
        EXPECT_NEAR(deepCopy, 0, requiredPrecision);
    }
}

TEST(Matrix, unitTest)
{
    biorbd::utils::Matrix mat1(3, 4);
    FILL_MATRIX(mat1, std::vector<double>({4.1, 5.1, 6.1, 7.1,
                                           8.1, 9.1, 10.1, 11.1,
                                           12.1, 13.1, 14.1, 15.1
                                          }));

    {
        biorbd::utils::Matrix mat2(3, 4);
        FILL_MATRIX(mat2, std::vector<double>({4.1, 5.1, 6.1, 7.1,
                                               8.1, 9.1, 10.1, 11.1,
                                               12.1, 13.1, 14.1, 15.1
                                              }));

        biorbd::utils::Matrix sum(mat1 + mat2);
        std::vector<double> expectedSum = {
            4.1*2, 5.1*2, 6.1*2, 7.1*2,
            8.1*2, 9.1*2, 10.1*2, 11.1*2,
            12.1*2, 13.1*2, 14.1*2, 15.1*2
        };

        EXPECT_EQ(sum.rows(), 3);
        EXPECT_EQ(sum.cols(), 4);
        int cmp(0);
        for (unsigned int i=0; i<sum.rows(); ++i) {
            for (unsigned int j=0; j<sum.cols(); ++j) {
                SCALAR_TO_DOUBLE(s, sum(i, j));
                EXPECT_NEAR(s, expectedSum[cmp++], requiredPrecision);
            }
        }
    }

    {
        biorbd::utils::Matrix mat2(4, 2);
        FILL_MATRIX(mat2, std::vector<double>({
            4.1, 5.1,
            6.1, 7.1,
            8.1, 9.1,
            10.1, 11.1
        }));

        biorbd::utils::Matrix mult(mat1 * mat2);
        std::vector<double> expectedMult = {
            169.04,  191.44,
            282.64,  321.04,
            396.24,  450.64
        };
        EXPECT_EQ(mult.rows(), 3);
        EXPECT_EQ(mult.cols(), 2);
        int cmp(0);
        for (unsigned int i=0; i<mult.rows(); ++i) {
            for (unsigned int j=0; j<mult.cols(); ++j) {
                SCALAR_TO_DOUBLE(m, mult(i, j));
                EXPECT_NEAR(m, expectedMult[cmp++], requiredPrecision);
            }
        }
    }

    {
        biorbd::utils::Vector vec(4);
        FILL_VECTOR(vec, std::vector<double>({4.1, 5.1, 6.1, 7.1}));
        biorbd::utils::Vector mult(mat1 * vec);

        std::vector<double> expectedMult({130.44, 220.04, 309.64});

        EXPECT_EQ(mult.rows(), 3);
        EXPECT_EQ(mult.cols(), 1);
        int cmp(0);
        for (unsigned int i=0; i<mult.rows(); ++i) {
            SCALAR_TO_DOUBLE(m, mult(i));
            EXPECT_NEAR(m, expectedMult[cmp++], requiredPrecision);
        }
    }
}

TEST(Rotation, unitTest)
{
    biorbd::utils::Rotation rot1(
        biorbd::utils::Vector3d(M_PI/3, M_PI/3, -M_PI/3), "xyz");
    biorbd::utils::Rotation rot2(
        biorbd::utils::Vector3d(M_PI/3, M_PI/3, M_PI/3), "xyz");

    {
        biorbd::utils::Rotation mult(rot1 * rot2);
        std::vector<double> expectedMult({
            0.87439881604791125, 0.4185095264191645, 0.24551270189221938,
            0.48131011839520887, -0.68413452641916439, -0.54799682452694543,
            -0.06137817547305488, 0.59733552217964769, -0.79963928962874664
        });
        int cmp(0);
        for (unsigned int i=0; i<3; ++i) {
            for (unsigned int j=0; j<3; ++j) {
                SCALAR_TO_DOUBLE(m, mult(i, j));
                EXPECT_NEAR(m, expectedMult[cmp++], requiredPrecision);
            }
        }
    }
}

TEST(RotoTrans, unitTest)
{
    {
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(1, 1, 1), biorbd::utils::Vector3d(1, 1, 1), "xyz");
        std::vector<double> rtExpected({
            0.29192658172642888, -0.45464871341284091,  0.84147098480789650, 1,
            0.83722241402998721, -0.30389665486452672, -0.45464871341284091, 1,
            0.46242567005663016,  0.83722241402998732,  0.29192658172642888, 1,
            0, 0, 0, 1
        });
        int cmp(0);
        for (unsigned int i = 0; i<4; ++i) {
            for (unsigned int j = 0; j<4; ++j) {
                SCALAR_TO_DOUBLE(val, rt(i, j));
                EXPECT_NEAR(val, rtExpected[cmp++], requiredPrecision);
            }
        }

        biorbd::utils::RotoTrans rt_t(rt.transpose());
        std::vector<double> rtTransposedExpected({
            0.29192658172642888,  0.83722241402998721, 0.46242567005663016, -1.5915746658130461,
            -0.45464871341284091, -0.30389665486452672, 0.83722241402998732, -0.07867704575261969,
            0.84147098480789650, -0.45464871341284091, 0.29192658172642888, -0.67874885312148447,
            0, 0, 0, 1
        });
        cmp = 0;
        for (unsigned int i = 0; i<4; ++i) {
            for (unsigned int j = 0; j<4; ++j) {
                SCALAR_TO_DOUBLE(val, rt_t(i, j));
                EXPECT_NEAR(val, rtTransposedExpected[cmp++],
                            requiredPrecision);
            }
        }

        biorbd::utils::Vector3d marker(1, 1, 1);
        marker.applyRT(rt_t);
        for (unsigned int i=0; i<3; ++i) {
            SCALAR_TO_DOUBLE(val, marker(i, 0));
            EXPECT_NEAR(val, 0, requiredPrecision);
        }
    }

    {
        biorbd::rigidbody::NodeSegment origin(1, 2, 3);
        biorbd::rigidbody::NodeSegment axis1(4, 2, 5);
        biorbd::rigidbody::NodeSegment axis2(3, -2, 1);
        {
            biorbd::utils::RotoTrans rt(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis1}, {origin, axis2}, {"x", "y"}, "x"));
            biorbd::utils::RotoTrans rt2(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin,  {origin, axis2}, {origin, axis1}, {"y", "x"}, "x"));

            std::vector<double> rtExpected({
                0.79091157883870022, 0.4082482904638631, 0.4558423058385518, 1,
                0.09304842103984709, -0.8164965809277261, 0.5698028822981898, 2,
                0.6048147367590061, -0.4082482904638631, -0.6837634587578276, 3,
                0, 0, 0, 1
            });
            int cmp(0);
            for (unsigned int i = 0; i<4; ++i) {
                for (unsigned int j = 0; j<4; ++j) {
                    SCALAR_TO_DOUBLE(val1, rt(i, j));
                    SCALAR_TO_DOUBLE(val2, rt2(i, j));
                    EXPECT_NEAR(val1, rtExpected[cmp], requiredPrecision);
                    EXPECT_NEAR(val2, rtExpected[cmp++], requiredPrecision);
                }
            }
        }

        {
            biorbd::utils::RotoTrans rt(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis1}, {origin, axis2}, {"x", "y"}, "y"));
            biorbd::utils::RotoTrans rt2(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis2}, {origin, axis1}, {"y", "x"}, "y"));
            std::vector<double> rtExpected({
                0.8320502943378437, 0.31606977062050695, 0.4558423058385518, 1,
                0,                  -0.8217814036133181, 0.5698028822981898, 2,
                0.5547001962252291, -0.47410465593076045, -0.6837634587578276, 3,
                0, 0, 0, 1
            });
            int cmp(0);
            for (unsigned int i = 0; i<4; ++i) {
                for (unsigned int j = 0; j<4; ++j) {
                    SCALAR_TO_DOUBLE(val1, rt(i, j));
                    SCALAR_TO_DOUBLE(val2, rt2(i, j));
                    EXPECT_NEAR(val1, rtExpected[cmp], requiredPrecision);
                    EXPECT_NEAR(val2, rtExpected[cmp++], requiredPrecision);
                }
            }
        }

        {
            biorbd::utils::RotoTrans rt(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis1}, {origin, axis2}, {"x", "z"}, "x"));
            biorbd::utils::RotoTrans rt2(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis2}, {origin, axis1}, {"z", "x"}, "x"));
            std::vector<double> rtExpected({
                0.7909115788387002, -0.4558423058385518, 0.4082482904638631, 1,
                0.09304842103984709, -0.5698028822981898, -0.8164965809277261, 2,
                0.6048147367590061, 0.6837634587578276, -0.4082482904638631, 3,
                0, 0, 0, 1
            });
            int cmp(0);
            for (unsigned int i = 0; i<4; ++i) {
                for (unsigned int j = 0; j<4; ++j) {
                    SCALAR_TO_DOUBLE(val1, rt(i, j));
                    SCALAR_TO_DOUBLE(val2, rt2(i, j));
                    EXPECT_NEAR(val1, rtExpected[cmp], requiredPrecision);
                    EXPECT_NEAR(val2, rtExpected[cmp++], requiredPrecision);
                }
            }
        }

        {
            biorbd::utils::RotoTrans rt(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis1}, {origin, axis2}, {"x", "z"}, "z"));
            biorbd::utils::RotoTrans rt2(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis2}, {origin, axis1}, {"z", "x"}, "z"));
            std::vector<double> rtExpected({
                0.8320502943378437, -0.4558423058385518, 0.31606977062050695, 1,
                0.0,                -0.5698028822981898, -0.8217814036133181, 2,
                0.5547001962252291, 0.6837634587578276, -0.47410465593076045, 3,
                0, 0, 0, 1
            });
            int cmp(0);
            for (unsigned int i = 0; i<4; ++i) {
                for (unsigned int j = 0; j<4; ++j) {
                    SCALAR_TO_DOUBLE(val1, rt(i, j));
                    SCALAR_TO_DOUBLE(val2, rt2(i, j));
                    EXPECT_NEAR(val1, rtExpected[cmp], requiredPrecision);
                    EXPECT_NEAR(val2, rtExpected[cmp++], requiredPrecision);
                }
            }
        }

        {
            biorbd::utils::RotoTrans rt(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis1}, {origin, axis2}, {"y", "z"}, "y"));
            biorbd::utils::RotoTrans rt2(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis2}, {origin, axis1}, {"z", "y"}, "y"));
            std::vector<double> rtExpected({
                0.4558423058385518, 0.7909115788387002, 0.4082482904638631, 1,
                0.5698028822981898, 0.09304842103984709, -0.8164965809277261, 2,
                -0.6837634587578276, 0.6048147367590061, -0.4082482904638631, 3,
                0, 0, 0, 1
            });
            int cmp(0);
            for (unsigned int i = 0; i<4; ++i) {
                for (unsigned int j = 0; j<4; ++j) {
                    SCALAR_TO_DOUBLE(val1, rt(i, j));
                    SCALAR_TO_DOUBLE(val2, rt2(i, j));
                    EXPECT_NEAR(val1, rtExpected[cmp], requiredPrecision);
                    EXPECT_NEAR(val2, rtExpected[cmp++], requiredPrecision);
                }
            }
        }

        {
            biorbd::utils::RotoTrans rt(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis1}, {origin, axis2}, {"y", "z"}, "z"));
            biorbd::utils::RotoTrans rt2(
                biorbd::utils::RotoTrans::fromMarkers(
                    origin, {origin, axis2}, {origin, axis1}, {"z", "y"}, "z"));
            std::vector<double> rtExpected({
                0.4558423058385518, 0.8320502943378437, 0.31606977062050695, 1,
                0.5698028822981898, 0.0,                -0.8217814036133181, 2,
                -0.6837634587578276, 0.5547001962252291, -0.47410465593076045, 3,
                0, 0, 0, 1
            });
            int cmp(0);
            for (unsigned int i = 0; i<4; ++i) {
                for (unsigned int j = 0; j<4; ++j) {
                    SCALAR_TO_DOUBLE(val1, rt(i, j));
                    SCALAR_TO_DOUBLE(val2, rt2(i, j));
                    EXPECT_NEAR(val1, rtExpected[cmp], requiredPrecision);
                    EXPECT_NEAR(val2, rtExpected[cmp++], requiredPrecision);
                }
            }
        }
    }
}

TEST(RotoTransNode, Copy)
{
    biorbd::utils::RotoTrans tp(
        biorbd::utils::Vector3d(1, 2, 3),
        biorbd::utils::Vector3d(1, 2, 3), "xyz");

    biorbd::utils::RotoTransNode MainRotoTransNode(tp, "NoName", "NoParent");
    biorbd::utils::RotoTransNode ShallowCopy(MainRotoTransNode);
    biorbd::utils::RotoTransNode DeepCopyNow(MainRotoTransNode.DeepCopy());
    biorbd::utils::RotoTransNode DeepCopyLater;
    DeepCopyLater.DeepCopy(MainRotoTransNode);

    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the names
    // Give a parent to the ShallowCopy
    ShallowCopy.setParent("NewParent");

    // Parent of MainNode should also change, but not of the DeepCopy
    EXPECT_STREQ(MainRotoTransNode.parent().c_str(), "NewParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NewParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");

    // Test for the values
    {
        SCALAR_TO_DOUBLE(MainRotoTransNode22, MainRotoTransNode(2, 2));
        SCALAR_TO_DOUBLE(ShallowCopy22, ShallowCopy(2, 2));
        SCALAR_TO_DOUBLE(DeepCopyNow22, DeepCopyNow(2, 2));
        SCALAR_TO_DOUBLE(DeepCopyLater22, DeepCopyLater(2, 2));
        EXPECT_NEAR(MainRotoTransNode22, -0.22484509536615291, requiredPrecision);
        EXPECT_NEAR(ShallowCopy22, -0.22484509536615291, requiredPrecision);
        EXPECT_NEAR(DeepCopyNow22, -0.22484509536615291, requiredPrecision);
        EXPECT_NEAR(DeepCopyLater22, -0.22484509536615291, requiredPrecision);
    }
    // Change the values of ShallowCopy
    ShallowCopy.setIdentity();

    // Data are NOT shallow copy, therefore the parent should keep its values
    {
        SCALAR_TO_DOUBLE(MainRotoTransNode22, MainRotoTransNode(2, 2));
        SCALAR_TO_DOUBLE(ShallowCopy22, ShallowCopy(2, 2));
        SCALAR_TO_DOUBLE(DeepCopyNow22, DeepCopyNow(2, 2));
        SCALAR_TO_DOUBLE(DeepCopyLater22, DeepCopyLater(2, 2));
        EXPECT_NEAR(MainRotoTransNode22, -0.22484509536615291, requiredPrecision);
        EXPECT_NEAR(ShallowCopy22, 1, requiredPrecision);
        EXPECT_NEAR(DeepCopyNow22, -0.22484509536615291, requiredPrecision);
        EXPECT_NEAR(DeepCopyLater22, -0.22484509536615291, requiredPrecision);
    }

}

TEST(ModelReading, equations)
{
    // The equation model was built so the x coordinates of the meshes should
    // be evaluated to the y coordinates.

    biorbd::Model m("models/equations.bioMod");
    biorbd::rigidbody::GeneralizedCoordinates Q(m);
    Q.setZero();
    std::vector<biorbd::utils::Vector3d> mesh(m.meshPoints(Q, 0, true));
    for (auto node : mesh) {
        SCALAR_TO_DOUBLE(nodeX, node.x());
        SCALAR_TO_DOUBLE(nodeY, node.y());
        EXPECT_DOUBLE_EQ(nodeX, nodeY);
    }
}

TEST(Quaternion, creation)
{
    {
        biorbd::utils::Quaternion quat;
        SCALAR_TO_DOUBLE(quatW, quat.w());
        SCALAR_TO_DOUBLE(quatX, quat.x());
        SCALAR_TO_DOUBLE(quatY, quat.y());
        SCALAR_TO_DOUBLE(quatZ, quat.z());
        EXPECT_NEAR(quatW, 1, requiredPrecision);
        EXPECT_NEAR(quatX, 0, requiredPrecision);
        EXPECT_NEAR(quatY, 0, requiredPrecision);
        EXPECT_NEAR(quatZ, 0, requiredPrecision);
        EXPECT_NEAR(quat.kStab(), 1, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion quat(1,2,3,4);
        SCALAR_TO_DOUBLE(quatW, quat.w());
        SCALAR_TO_DOUBLE(quatX, quat.x());
        SCALAR_TO_DOUBLE(quatY, quat.y());
        SCALAR_TO_DOUBLE(quatZ, quat.z());
        EXPECT_NEAR(quatW, 1, requiredPrecision);
        EXPECT_NEAR(quatX, 2, requiredPrecision);
        EXPECT_NEAR(quatY, 3, requiredPrecision);
        EXPECT_NEAR(quatZ, 4, requiredPrecision);

        SCALAR_TO_DOUBLE(quat0, quat[0]);
        SCALAR_TO_DOUBLE(quat1, quat[1]);
        SCALAR_TO_DOUBLE(quat2, quat[2]);
        SCALAR_TO_DOUBLE(quat3, quat[3]);
        EXPECT_NEAR(quat0, 1, requiredPrecision);
        EXPECT_NEAR(quat1, 2, requiredPrecision);
        EXPECT_NEAR(quat2, 3, requiredPrecision);
        EXPECT_NEAR(quat3, 4, requiredPrecision);
    }
    {
#ifdef BIORBD_USE_EIGEN3_MATH
        biorbd::utils::Quaternion quat(Eigen::Vector4d(1,2,3,4));
        EXPECT_NEAR(quat.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat.z(), 4, requiredPrecision);
#endif
    }
    {
        biorbd::utils::Quaternion quat(1, biorbd::utils::Vector3d(2,3,4));
        SCALAR_TO_DOUBLE(quatW, quat.w());
        SCALAR_TO_DOUBLE(quatX, quat.x());
        SCALAR_TO_DOUBLE(quatY, quat.y());
        SCALAR_TO_DOUBLE(quatZ, quat.z());
        EXPECT_NEAR(quatW, 1, requiredPrecision);
        EXPECT_NEAR(quatX, 2, requiredPrecision);
        EXPECT_NEAR(quatY, 3, requiredPrecision);
        EXPECT_NEAR(quatZ, 4, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion quat1(1,2,3,4,5);
        biorbd::utils::Quaternion quat2(quat1);
        biorbd::utils::Quaternion quat3 = quat1;

        SCALAR_TO_DOUBLE(quat2W, quat2.w());
        SCALAR_TO_DOUBLE(quat2X, quat2.x());
        SCALAR_TO_DOUBLE(quat2Y, quat2.y());
        SCALAR_TO_DOUBLE(quat2Z, quat2.z());
        EXPECT_NEAR(quat2W, 1, requiredPrecision);
        EXPECT_NEAR(quat2X, 2, requiredPrecision);
        EXPECT_NEAR(quat2Y, 3, requiredPrecision);
        EXPECT_NEAR(quat2Z, 4, requiredPrecision);
        EXPECT_NEAR(quat2.kStab(), 5, requiredPrecision);

        SCALAR_TO_DOUBLE(quat3W, quat3.w());
        SCALAR_TO_DOUBLE(quat3X, quat3.x());
        SCALAR_TO_DOUBLE(quat3Y, quat3.y());
        SCALAR_TO_DOUBLE(quat3Z, quat3.z());
        EXPECT_NEAR(quat3W, 1, requiredPrecision);
        EXPECT_NEAR(quat3X, 2, requiredPrecision);
        EXPECT_NEAR(quat3Y, 3, requiredPrecision);
        EXPECT_NEAR(quat3Z, 4, requiredPrecision);
        EXPECT_NEAR(quat3.kStab(), 5, requiredPrecision);
    }
    {
#ifdef BIORBD_USE_EIGEN3_MATH
        Eigen::Vector4d quat1(1,2,3,4);
        biorbd::utils::Quaternion quat2(quat1);
        biorbd::utils::Quaternion quat3;
        quat3 = quat1;

        EXPECT_NEAR(quat2.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat2.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat2.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat2.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat2.kStab(), 1, requiredPrecision);

        EXPECT_NEAR(quat3.w(), 1, requiredPrecision);
        EXPECT_NEAR(quat3.x(), 2, requiredPrecision);
        EXPECT_NEAR(quat3.y(), 3, requiredPrecision);
        EXPECT_NEAR(quat3.z(), 4, requiredPrecision);
        EXPECT_NEAR(quat3.kStab(), 1, requiredPrecision);
#endif
    }
}

TEST(Quaternion, addition)
{
    biorbd::utils::Quaternion q1(1,2,3,4,2);
    biorbd::utils::Quaternion q2(2,3,4,5,3);
    biorbd::utils::Quaternion q12 = q1+q2;

    SCALAR_TO_DOUBLE(q12W, q12.w());
    SCALAR_TO_DOUBLE(q12X, q12.x());
    SCALAR_TO_DOUBLE(q12Y, q12.y());
    SCALAR_TO_DOUBLE(q12Z, q12.z());
    EXPECT_NEAR(q12W, 3, requiredPrecision);
    EXPECT_NEAR(q12X, 5, requiredPrecision);
    EXPECT_NEAR(q12Y, 7, requiredPrecision);
    EXPECT_NEAR(q12Z, 9, requiredPrecision);
    EXPECT_NEAR(q12.kStab(), 2.5, requiredPrecision);
}

TEST(Quaternion, multiplication)
{
    biorbd::utils::Quaternion q1(1,2,3,4,2);
    biorbd::utils::Quaternion q2(2,3,4,5,3);
    double d(5);
    float f(5);

    {
        biorbd::utils::Quaternion q12 = q1*q2;
        SCALAR_TO_DOUBLE(q12W, q12.w());
        SCALAR_TO_DOUBLE(q12X, q12.x());
        SCALAR_TO_DOUBLE(q12Y, q12.y());
        SCALAR_TO_DOUBLE(q12Z, q12.z());
        EXPECT_NEAR(q12W, -36, requiredPrecision);
        EXPECT_NEAR(q12X, 6, requiredPrecision);
        EXPECT_NEAR(q12Y, 12, requiredPrecision);
        EXPECT_NEAR(q12Z, 12, requiredPrecision);
        EXPECT_NEAR(q12.kStab(), 2.5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q1d = q1*d;
        SCALAR_TO_DOUBLE(q1dW, q1d.w());
        SCALAR_TO_DOUBLE(q1dX, q1d.x());
        SCALAR_TO_DOUBLE(q1dY, q1d.y());
        SCALAR_TO_DOUBLE(q1dZ, q1d.z());
        EXPECT_NEAR(q1dW, 5, requiredPrecision);
        EXPECT_NEAR(q1dX, 10, requiredPrecision);
        EXPECT_NEAR(q1dY, 15, requiredPrecision);
        EXPECT_NEAR(q1dZ, 20, requiredPrecision);
        EXPECT_NEAR(q1d.kStab(), 2, requiredPrecision);
    }

    {
        biorbd::utils::Quaternion q1f = q1*f;
        SCALAR_TO_DOUBLE(q1fW, q1f.w());
        SCALAR_TO_DOUBLE(q1fX, q1f.x());
        SCALAR_TO_DOUBLE(q1fY, q1f.y());
        SCALAR_TO_DOUBLE(q1fZ, q1f.z());
        EXPECT_NEAR(q1fW, 5, requiredPrecision);
        EXPECT_NEAR(q1fX, 10, requiredPrecision);
        EXPECT_NEAR(q1fY, 15, requiredPrecision);
        EXPECT_NEAR(q1fZ, 20, requiredPrecision);
        EXPECT_NEAR(q1f.kStab(), 2, requiredPrecision);
    }
}

TEST(Quaternion, conversion)
{
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromGLRotate(1, 2, 3, 4, 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, 0.99996192306417131, requiredPrecision);
        EXPECT_NEAR(qX, 0.017453070996747869, requiredPrecision);
        EXPECT_NEAR(qY, 0.026179606495121806, requiredPrecision);
        EXPECT_NEAR(qZ, 0.034906141993495739, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromAxisAngle(
                1, biorbd::utils::Vector3d(2, 3, 4), 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, 0.87758256189037276, requiredPrecision);
        EXPECT_NEAR(qX, 0.17805417504364543, requiredPrecision);
        EXPECT_NEAR(qY, 0.26708126256546816, requiredPrecision);
        EXPECT_NEAR(qZ, 0.35610835008729086, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    biorbd::utils::RotoTrans rt(
        biorbd::utils::Vector3d(2, 3, 4), biorbd::utils::Vector3d(), "xyz");
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromMatrix(rt, 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, 0.77913560959923722, requiredPrecision);
        EXPECT_NEAR(qX, -0.46529436049374817, requiredPrecision);
        EXPECT_NEAR(qY, 0.27840624141687692, requiredPrecision);
        EXPECT_NEAR(qZ, 0.31454542547502823, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromMatrix(rt.rot(), 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, 0.77913560959923722, requiredPrecision);
        EXPECT_NEAR(qX, -0.46529436049374817, requiredPrecision);
        EXPECT_NEAR(qY, 0.27840624141687692, requiredPrecision);
        EXPECT_NEAR(qZ, 0.31454542547502823, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromZYXAngles(
                biorbd::utils::Vector3d(2, 3, 4), 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, 0.74732578388941839, requiredPrecision);
        EXPECT_NEAR(qX, -0.51483522877414645, requiredPrecision);
        EXPECT_NEAR(qY, -0.17015746936361908, requiredPrecision);
        EXPECT_NEAR(qZ, 0.38405116269438366, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromYXZAngles(
                biorbd::utils::Vector3d(2, 3, 4), 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, 0.74732578388941828, requiredPrecision);
        EXPECT_NEAR(qX, 0.46529436049374834, requiredPrecision);
        EXPECT_NEAR(qY, -0.27840624141687698, requiredPrecision);
        EXPECT_NEAR(qZ, 0.38405116269438366, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromXYZAngles(
                biorbd::utils::Vector3d(2, 3, 4), 5));
        SCALAR_TO_DOUBLE(qW, q.w());
        SCALAR_TO_DOUBLE(qX, q.x());
        SCALAR_TO_DOUBLE(qY, q.y());
        SCALAR_TO_DOUBLE(qZ, q.z());
        EXPECT_NEAR(qW, -0.77913560959923722, requiredPrecision);
        EXPECT_NEAR(qX, 0.46529436049374834, requiredPrecision);
        EXPECT_NEAR(qY, -0.27840624141687698, requiredPrecision);
        EXPECT_NEAR(qZ, -0.31454542547502828, requiredPrecision);
        EXPECT_NEAR(q.kStab(), 5, requiredPrecision);
    }
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
#ifndef BIORBD_USE_CASADI_MATH
        EXPECT_THROW(q.toMatrix(), std::runtime_error);
#endif
        biorbd::utils::Rotation mat(q.toMatrix(true));

        SCALAR_TO_DOUBLE(mat00, mat(0, 0));
        SCALAR_TO_DOUBLE(mat01, mat(0, 1));
        SCALAR_TO_DOUBLE(mat02, mat(0, 2));
        SCALAR_TO_DOUBLE(mat10, mat(1, 0));
        SCALAR_TO_DOUBLE(mat11, mat(1, 1));
        SCALAR_TO_DOUBLE(mat12, mat(1, 2));
        SCALAR_TO_DOUBLE(mat20, mat(2, 0));
        SCALAR_TO_DOUBLE(mat21, mat(2, 1));
        SCALAR_TO_DOUBLE(mat22, mat(2, 2));
        EXPECT_NEAR(mat00, -81, requiredPrecision);
        EXPECT_NEAR(mat01, 4, requiredPrecision);
        EXPECT_NEAR(mat02, 46, requiredPrecision);
        EXPECT_NEAR(mat10, 44, requiredPrecision);
        EXPECT_NEAR(mat11, -67, requiredPrecision);
        EXPECT_NEAR(mat12, 28, requiredPrecision);
        EXPECT_NEAR(mat20, 14, requiredPrecision);
        EXPECT_NEAR(mat21, 52, requiredPrecision);
        EXPECT_NEAR(mat22, -49, requiredPrecision);
    }
    {
        biorbd::utils::Vector3d rot (0.2, 0.3, 0.4);
        biorbd::utils::RotoTrans rt_from_euler;
        rt_from_euler = biorbd::utils::RotoTrans::fromEulerAngles(rot,
                        biorbd::utils::Vector3d(), "xyz");

        biorbd::utils::Quaternion q(
            biorbd::utils::Quaternion::fromXYZAngles(rot, 5));
        biorbd::utils::RotoTrans rt_from_quat(q.toMatrix());

        for (unsigned int i=0; i<4; ++i) {
            for (unsigned int j=0; j<4; ++j) {
                SCALAR_TO_DOUBLE(valEuler, rt_from_euler(i, j));
                SCALAR_TO_DOUBLE(valQuat, rt_from_quat(i, j));
                EXPECT_NEAR(valEuler, valQuat, requiredPrecision);
            }
        }

        biorbd::utils::Quaternion qFromRt(biorbd::utils::Quaternion::fromMatrix(
                                              rt_from_quat));
        for (unsigned int i=0; i<4; ++i) {
            SCALAR_TO_DOUBLE(valQ, q(i));
            SCALAR_TO_DOUBLE(valQRt, qFromRt(i));
            EXPECT_NEAR(valQ, valQRt, requiredPrecision);
        }
    }
}

TEST(Quaternion, otherOperations)
{
#ifndef BIORBD_USE_CASADI_MATH
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion q2(3, 4, 5, 6, 8);
        biorbd::utils::Quaternion qSlerp(q.slerp (7, q2));

        SCALAR_TO_DOUBLE(qSlerpW, qSlerp.w());
        SCALAR_TO_DOUBLE(qSlerpX, qSlerp.x());
        SCALAR_TO_DOUBLE(qSlerpY, qSlerp.y());
        SCALAR_TO_DOUBLE(qSlerpZ, qSlerp.z());
        EXPECT_NEAR(qSlerpW, 8.582415805503274, requiredPrecision);
        EXPECT_NEAR(qSlerpX, 9.493173329713267, requiredPrecision);
        EXPECT_NEAR(qSlerpY, 10.40393085392326, requiredPrecision);
        EXPECT_NEAR(qSlerpZ, 11.314688378133257, requiredPrecision);
        EXPECT_NEAR(qSlerp.kStab(), 7, requiredPrecision);
    }
#endif
    {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion qConj(q.conjugate());

        SCALAR_TO_DOUBLE(qConjW, qConj.w());
        SCALAR_TO_DOUBLE(qConjX, qConj.x());
        SCALAR_TO_DOUBLE(qConjY, qConj.y());
        SCALAR_TO_DOUBLE(qConjZ, qConj.z());
        EXPECT_NEAR(qConjW, 2, requiredPrecision);
        EXPECT_NEAR(qConjX, -3, requiredPrecision);
        EXPECT_NEAR(qConjY, -4, requiredPrecision);
        EXPECT_NEAR(qConjZ, -5, requiredPrecision);
        EXPECT_NEAR(qConj.kStab(), 6, requiredPrecision);
    } {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion qTime(
            q.timeStep(biorbd::utils::Vector3d(7, 8, 9), 0.1));

        SCALAR_TO_DOUBLE(qTimeW, qTime.w());
        SCALAR_TO_DOUBLE(qTimeX, qTime.x());
        SCALAR_TO_DOUBLE(qTimeY, qTime.y());
        SCALAR_TO_DOUBLE(qTimeZ, qTime.z());
        EXPECT_NEAR(qTimeW, -2.9791236033976602, requiredPrecision);
        EXPECT_NEAR(qTimeX, 3.1304258212109395, requiredPrecision);
        EXPECT_NEAR(qTimeY, 3.4370175820938655, requiredPrecision);
        EXPECT_NEAR(qTimeZ, 4.8489346122578709, requiredPrecision);
        EXPECT_NEAR(qTime.kStab(), 6, requiredPrecision);
    } {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Vector3d vec(q.rotate(biorbd::utils::Vector3d(7, 8, 9)));

        SCALAR_TO_DOUBLE(vecX, vec.x());
        SCALAR_TO_DOUBLE(vecY, vec.y());
        SCALAR_TO_DOUBLE(vecZ, vec.z());
        EXPECT_NEAR(vecX, 282, requiredPrecision);
        EXPECT_NEAR(vecY, 384, requiredPrecision);
        EXPECT_NEAR(vecZ, 582, requiredPrecision);
    } {
        biorbd::utils::Quaternion q(2, 3, 4, 5, 6);
        biorbd::utils::Quaternion qdot(q.omegaToQDot(biorbd::utils::Vector3d(7, 8, 9)));

        SCALAR_TO_DOUBLE(qdotW, qdot.w());
        SCALAR_TO_DOUBLE(qdotX, qdot.x());
        SCALAR_TO_DOUBLE(qdotY, qdot.y());
        SCALAR_TO_DOUBLE(qdotZ, qdot.z());
        EXPECT_NEAR(qdotW, -49, requiredPrecision);
        EXPECT_NEAR(qdotX, 5, requiredPrecision);
        EXPECT_NEAR(qdotY, 12, requiredPrecision);
        EXPECT_NEAR(qdotZ, 7, requiredPrecision);
        EXPECT_NEAR(qdot.kStab(), 6, requiredPrecision);
    } {
        biorbd::utils::Quaternion q1(1, 0, 0, 0, 5);
        biorbd::utils::Vector v(3);
        FILL_VECTOR(v, std::vector<double>({1,2,3}));
        q1.derivate(v);
        {
            SCALAR_TO_DOUBLE(q1W, q1.w());
            SCALAR_TO_DOUBLE(q1X, q1.x());
            SCALAR_TO_DOUBLE(q1Y, q1.y());
            SCALAR_TO_DOUBLE(q1Z, q1.z());
            EXPECT_NEAR(q1W, 0, requiredPrecision);
            EXPECT_NEAR(q1X, 0.5, requiredPrecision);
            EXPECT_NEAR(q1Y, 1, requiredPrecision);
            EXPECT_NEAR(q1Z, 1.5, requiredPrecision);
            EXPECT_NEAR(q1.kStab(), 5, requiredPrecision);
        }
        double w(0.07035975447302918);
        double x(0.7035975447302919);
        double y(0.7035975447302919);
        double z(0.07035975447302918);
        biorbd::utils::Quaternion q2(w,x,y,z, 5);
        q2.derivate(v);
        {
            SCALAR_TO_DOUBLE(q2W, q2.w());
            SCALAR_TO_DOUBLE(q2X, q2.x());
            SCALAR_TO_DOUBLE(q2Y, q2.y());
            SCALAR_TO_DOUBLE(q2Z, q2.z());
            EXPECT_NEAR(q2W,-1.1609359488049815, requiredPrecision);
            EXPECT_NEAR(q2X, 1.0202164398589233, requiredPrecision);
            EXPECT_NEAR(q2Y,-0.9498566853858941, requiredPrecision);
            EXPECT_NEAR(q2Z, 0.45733840407468973, requiredPrecision);
            EXPECT_NEAR(q2.kStab(), 5, requiredPrecision);
        }
    }
}

TEST(Quaternion, velocities)
{
    {
        biorbd::utils::Vector3d e(0.1,0.2,0.3);
        biorbd::utils::Vector3d eR(0.4,0.5,0.6);
        biorbd::utils::Quaternion q;
        biorbd::utils::Vector3d w(q.eulerDotToOmega(eR,e,"xyz"));

        SCALAR_TO_DOUBLE(w0, w[0]);
        SCALAR_TO_DOUBLE(w1, w[1]);
        SCALAR_TO_DOUBLE(w2, w[2]);
        EXPECT_NEAR(w0, 0.52227744876434945, requiredPrecision);
        EXPECT_NEAR(w1, 0.36181645351259678, requiredPrecision);
        EXPECT_NEAR(w2, 0.67946773231802449, requiredPrecision);
    }

}

TEST(Quaternion, normalization)
{
    {
        biorbd::utils::Quaternion q(1,1,1,1);
        q.normalize();
        SCALAR_TO_DOUBLE(q0, q[0]);
        SCALAR_TO_DOUBLE(q1, q[1]);
        SCALAR_TO_DOUBLE(q2, q[2]);
        SCALAR_TO_DOUBLE(q3, q[3]);
        EXPECT_NEAR(q0, 0.5, requiredPrecision);
        EXPECT_NEAR(q1, 0.5, requiredPrecision);
        EXPECT_NEAR(q2, 0.5, requiredPrecision);
        EXPECT_NEAR(q3, 0.5, requiredPrecision);
    }

}

TEST(Vector, operation)
{

    //TODO: Addition Scalar + Vector is undefined
    //{
    //    biorbd::utils::Vector v(4);
    //    v[0] = 1.1;
    //    v[1] = 1.2;
    //    v[2] = 1.3;
    //    v[3] = 1.4;
    //    biorbd::utils::Scalar s(5);
    //    biorbd::utils::Vector multVectAndScalar(v + s);

    //    biorbd::utils::Vector expectedMultVectAndScalar(4);
    //    expectedMultVectAndScalar << 6.1, 6.2, 6.3, 6.4;

    //    for (unsigned int i = 0; i < 4; ++i) {
    //        EXPECT_NEAR(multVectAndScalar[i], expectedMultVectAndScalar[i], requiredPrecision);
    //    }
    //}

    {
        biorbd::utils::Vector v(4);
        v[0] = 1.1;
        v[1] = 1.2;
        v[2] = 1.3;
        v[3] = 1.4;
        biorbd::utils::Scalar s(5);
        biorbd::utils::Vector multVectAndScalar(s * v);

        std::vector<double> expectedMultVectAndScalar({5.5, 6.0, 6.5, 7.0});

        for (unsigned int i = 0; i < 4; ++i) {
            SCALAR_TO_DOUBLE(val, multVectAndScalar(i));
            EXPECT_NEAR(val, expectedMultVectAndScalar[i], requiredPrecision);
        }
    }
}

TEST(Vector, norm)
{
    {
        biorbd::utils::Vector v(4);
        FILL_VECTOR(v, std::vector<double>({1.1, 1.2, 1.3, 1.4}));

        SCALAR_TO_DOUBLE(val2false, v.norm(2, false));
        SCALAR_TO_DOUBLE(val2true, v.norm(2, true));
        EXPECT_NEAR(val2false, 2.5099800796022267, requiredPrecision);
        EXPECT_NEAR(val2true, 6.2999999999999998, requiredPrecision);
    }
}

TEST(Vector, normGradient)
{
    {
        biorbd::utils::Vector v(4);
        FILL_VECTOR(v, std::vector<double>({1.1, 1.2, 1.3, 1.4}));

        biorbd::utils::Vector nG(v.normGradient(2, false));
        std::vector<double> expectednG({
            0.43825049008927769, 0.47809144373375745, 0.51793239737823726, 0.55777335102271697
        });

        for (unsigned int i = 0; i < 4; ++i) {
            SCALAR_TO_DOUBLE(val, nG(i));
            EXPECT_NEAR(val, expectednG[i], requiredPrecision);
        }
    }
}
