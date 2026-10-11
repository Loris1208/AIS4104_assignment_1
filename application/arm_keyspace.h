#ifndef HPP_GUARD_AIS4104_APPLICATION_ARM_KEYSPACE_H
#define HPP_GUARD_AIS4104_APPLICATION_ARM_KEYSPACE_H

#include <praxis/config/declaration.h>

namespace ais4104 {

// praxis's arm keyspace, with the fallback of every key the four mode-carrying windows write set to
// what they write when opened in Preview: a document stating no mode opens in Preview, and a
// document stating one keeps it. The keyspace also declares hand_derived/tool, the name of the
// function in modules/modeling/chains.cpp a document's tool center point comes from,
// tool_window/opens, false where a document's preset opens no Tool window, pose_window/frames,
// the frames a forward kinematics or tooling preset's Pose window reads: flange, tool or both, the
// default, and pose_target_window/opens, true where a tooling document's preset opens a Pose target
// window beside Tool frame motion.
praxis::config::declaration assignment_arm_keyspace();

}

#endif
