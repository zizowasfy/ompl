#define BOOST_TEST_MODULE "FactoredMotionPlanningLifts"
#include <boost/test/unit_test.hpp>

#include "factorization_common.h"

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_InclusionMaps)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(4);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    const auto indicesB = std::vector<size_t>({0,1});
    const auto indicesC = std::vector<size_t>({2,3});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));

    ////Create states to lift
    auto stateA = A->allocState();
    auto stateB = B->allocState();
    auto stateC = C->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});

    A->lift(baseStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[1], stateB_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA_RN->values[2], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[3], stateC_RN->values[1]);

    A->freeState(stateA);
    B->freeState(stateB);
    C->freeState(stateC);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_InclusionMapsShifted)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(4);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    const auto indicesB = std::vector<size_t>({0,3});
    const auto indicesC = std::vector<size_t>({1,2});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));

    ////Create states to lift
    auto stateA = A->allocState();
    auto stateB = B->allocState();
    auto stateC = C->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});

    A->lift(baseStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[1], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[2], stateC_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA_RN->values[3], stateB_RN->values[1]);

    A->freeState(stateA);
    B->freeState(stateB);
    C->freeState(stateC);
}

BOOST_AUTO_TEST_CASE(FactoredSpaceInformation_InclusionMapsMultiConnected)
{
    ompl::base::StateSpacePtr space_A = CreateCubeStateSpace(9);
    space_A->setName("SpaceA");
    auto A = std::make_shared<FactoredSpaceInformation>(space_A);

    ompl::base::StateSpacePtr space_B = CreateCubeStateSpace(2);
    space_B->setName("SpaceB");
    auto B = std::make_shared<FactoredSpaceInformation>(space_B);

    ompl::base::StateSpacePtr space_C = CreateCubeStateSpace(2);
    space_C->setName("SpaceC");
    auto C = std::make_shared<FactoredSpaceInformation>(space_C);

    ompl::base::StateSpacePtr space_D = CreateCubeStateSpace(5);
    space_D->setName("SpaceD");
    auto D = std::make_shared<FactoredSpaceInformation>(space_D);

    const auto indicesB = std::vector<size_t>({0,3});
    const auto indicesC = std::vector<size_t>({1,8});
    const auto indicesD = std::vector<size_t>({2,4,5,6,7});

    auto projAB = std::make_shared<Projection_RN_RM>(space_A, space_B, indicesB);
    BOOST_CHECK(A->addChild(B, projAB));
    auto projAC = std::make_shared<Projection_RN_RM>(space_A, space_C, indicesC);
    BOOST_CHECK(A->addChild(C, projAC));
    auto projAD = std::make_shared<Projection_RN_RM>(space_A, space_D, indicesD);
    BOOST_CHECK(A->addChild(D, projAD));

    ////Create states to lift
    auto stateA = A->allocState();
    auto stateB = B->allocState();
    auto stateC = C->allocState();
    auto stateD = D->allocState();

    auto samplerB = B->allocStateSampler();
    auto samplerC = C->allocStateSampler();
    auto samplerD = D->allocStateSampler();

    samplerB->sampleUniform(stateB);
    samplerC->sampleUniform(stateC);
    samplerD->sampleUniform(stateD);

    std::unordered_map<std::string, State*> baseStates;
    baseStates.insert({B->getName(), stateB});
    baseStates.insert({C->getName(), stateC});
    baseStates.insert({D->getName(), stateD});

    A->lift(baseStates, stateA);

    const auto *stateA_RN = stateA->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateB_RN = stateB->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateC_RN = stateC->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *stateD_RN = stateD->as<ompl::base::RealVectorStateSpace::StateType>();

    BOOST_CHECK_EQUAL(stateA_RN->values[0], stateB_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[3], stateB_RN->values[1]);

    BOOST_CHECK_EQUAL(stateA_RN->values[1], stateC_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[8], stateC_RN->values[1]);

    BOOST_CHECK_EQUAL(stateA_RN->values[2], stateD_RN->values[0]);
    BOOST_CHECK_EQUAL(stateA_RN->values[4], stateD_RN->values[1]);
    BOOST_CHECK_EQUAL(stateA_RN->values[5], stateD_RN->values[2]);
    BOOST_CHECK_EQUAL(stateA_RN->values[6], stateD_RN->values[3]);
    BOOST_CHECK_EQUAL(stateA_RN->values[7], stateD_RN->values[4]);

    A->freeState(stateA);
    B->freeState(stateB);
    C->freeState(stateC);
    D->freeState(stateD);
}
