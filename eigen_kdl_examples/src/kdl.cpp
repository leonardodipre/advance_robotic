#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>
#include <iomanip>  // Include this for std::setprecision

int main(int argc, char **argv)
{
    // Step 1: Create a KDL Chain object starting from the base block
    KDL::Chain kdlChain = KDL::Chain();

    // Step 2: Define the first segment (0.5m) extending vertically from the base to the first joint
    KDL::Joint joint1(KDL::Joint::None);  // Fixed joint
    KDL::Frame frame1 = KDL::Frame(KDL::Vector(0.0, 0.5, 0.0));  // 0.5m along the y-axis
    kdlChain.addSegment(KDL::Segment(joint1, frame1));

    // Step 3: Define the second segment (0.3m) extending vertically from the first to the second joint
    KDL::Joint joint2(KDL::Joint::RotZ);  // Second movable joint
    KDL::Frame frame2 = KDL::Frame(KDL::Vector(0.0, 0.3, 0.0));  // 0.3m along the y-axis
    kdlChain.addSegment(KDL::Segment(joint2, frame2));

    // Step 4: Define the third segment (0.2m) extending vertically to the next block
    KDL::Joint joint3(KDL::Joint::RotZ);  // Fixed block
    KDL::Frame frame3 = KDL::Frame(KDL::Vector(0.0, 0.2, 0.0));  // 0.2m along the y-axis
    kdlChain.addSegment(KDL::Segment(joint3, frame3));

    // Step 5: Define the fourth segment (0.2m) extending horizontally after the fixed block
    KDL::Joint joint4(KDL::Joint::RotZ);  // Third movable joint
    KDL::Frame frame4 = KDL::Frame(KDL::Vector(0.2, 0.0, 0.0));  // 0.2m along the x-axis
    kdlChain.addSegment(KDL::Segment(joint4, frame4));

    // Step 6: Define the fifth segment (0.1m) representing the final movable link
    KDL::Joint joint5(KDL::Joint::RotZ);  // Fourth movable joint
    KDL::Frame frame5 = KDL::Frame(KDL::Vector(0.1, 0.0, 0.0));  // 0.1m along the x-axis
    kdlChain.addSegment(KDL::Segment(joint5, frame5));

    // Step 7: Define the joint angles for the movable joints in RADIANDS
    KDL::JntArray jointAngles = KDL::JntArray(4);
    
    jointAngles(0) = 0.3 ;       // Joint 1
    jointAngles(1) = -0.2;        // Joint 2
    jointAngles(2) = 0.5; 
    jointAngles(3) = -0.1;          // Joint 3
            

    // Step 8: Initialize the forward kinematics solver
    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);

    // Step 9: Compute the end-effector frame
    KDL::Frame eeFrame;
    FKSolver.JntToCart(jointAngles, eeFrame);

    // Step 10: Print the translation (position) and rotation matrices
    // Translation (position)
    KDL::Vector position = eeFrame.p;
    std::cout << "End Effector Position:" << std::endl;
    std::cout << "X: " << position.x() << std::endl;
    std::cout << "Y: " << position.y() << std::endl;
    std::cout << "Z: " << position.z() << std::endl;

    // Rotation matrix
    std::cout << "End Effector Rotation Matrix:" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << std::fixed << std::setprecision(4) << eeFrame.M(i, j) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
