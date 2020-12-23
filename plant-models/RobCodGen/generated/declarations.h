#ifndef IIT_ROBOT_KUKA_DECLARATIONS_H_
#define IIT_ROBOT_KUKA_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace Kuka {

static const int JointSpaceDimension = 7;
static const int jointsCount = 7;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 8;

namespace tpl {
template <typename SCALAR>
using Column7d = iit::rbd::PlainMatrix<SCALAR, 7, 1>;

template <typename SCALAR>
using JointState = Column7d<SCALAR>;
}

using Column7d = tpl::Column7d<double>;
typedef Column7d JointState;

enum JointIdentifiers {
    JA = 0
    , JB
    , JC
    , JD
    , JE
    , JF
    , JG
};

enum LinkIdentifiers {
    BASE0 = 0
    , LINK1
    , LINK2
    , LINK3
    , LINK4
    , LINK5
    , LINK6
    , LINK7
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JA,JB,JC,JD,JE,JF,JG};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE0,LINK1,LINK2,LINK3,LINK4,LINK5,LINK6,LINK7};

}
}
#endif
