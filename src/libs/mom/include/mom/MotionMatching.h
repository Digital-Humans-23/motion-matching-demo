#pragma once

#include <deque>

#include "crl-basic/gui/plots.h"
#include "crl-basic/utils/trajectory.h"
#include "mom/MotionDatabase.h"

namespace crl::mocap {

/**
 * inertialization parameters.
 */
struct InertializationInfo {
    V3D x0v = V3D(0, 0, 0);
    double x0 = 0, v0 = 0, a0 = 0;
    double t1 = 0;               // when inertialization finishes
    double A = 0, B = 0, C = 0;  // coefficients
};

/**
 * Implementation of motion matching and inertialization blending.
 *
 * References:
 * [1] D. Holden, et al., "Learned Motion Matching", 2020.
 * [2] D. Bollo, "Inertialization: High-Performance Animation Transitions in’Gears of War’.", 2016.
 *
 * TODO:
 * - check query trajectory index 20/40/60 (should it be 21/41/61?)
 *
 * Done:
 * - double check frame index after motion matching and stitching. remember, we overlap very last frame of old frame with very first frame of new frame.
 */
class MotionMatching {
public:
    // commands
    double speedForward = 0;
    double speedSideways = 0;
    double turningSpeed = 0;

    // threshold for swing/contact
    double feetHeightThreshold = 0.055;
    double feetSpeedThreshold = 0.8;

    // transition time t1
    // note. actual transition time is min(t1, -5 * x0 / v0)
    double transitionTime = 0.4;

    // option
    bool inertialization = true;
    // if queueSize = 0, we just inertialize motion as post-process
    // if queueSize > 0, we save 0, 1, ..., queueSize (so in total queueSize+1) frames
    uint queueSize = 60;

    // extracted speed profile
    Trajectory3D speedProfile;
    Trajectory1D heightProfile;
    // if this is empty, we extract height and speed of root
    std::string referenceJointName = "";  // Spine1

private:
    MocapSkeleton *skeleton_ = nullptr;
    MotionDatabase *database_ = nullptr;

    // time related
    double motionTime_ = 0;
    double dt_ = 1.0 / 60;  // this is timestep of one frame (updated from DB)

    // inertialization info
    InertializationInfo rootPosInertializationInfo_;
    InertializationInfo rootQInertializationInfo_;
    std::vector<InertializationInfo> jointInertializationInfos_;

    // pose of character (updated when motion matching happens)
    P3D t_wc = P3D(0, 0, 0);
    Quaternion Q_wc = Quaternion::Identity();

    // pose of new motion at t=0
    P3D t_wt0 = P3D(0, 0, 0);
    Quaternion Q_wt0 = Quaternion::Identity();

    // pose of new motion at t=0 after stitching
    P3D t_wt0prime = P3D(0, 0, 0);
    Quaternion Q_wt0prime = Quaternion::Identity();

    // motion idx
    MotionIndex currMotionIdx_ = {-1, -1};
    MotionIndex lastMotionIdx_ = {-1, -1};

    // queue
    // we want to compute initialized motion at once (when motion matching happens) and extracting velocity and foot contact profile.
    std::deque<MocapSkeletonState> queue_;

    // plot
    crl::gui::RealTimeLinePlot2D<crl::V3D> characterSpeedPlots_;
    crl::gui::RealTimeLinePlot2D<crl::V3D> speedProfilePlots_;
    crl::gui::RealTimeLinePlot2D<crl::dVector> eeSpeedPlots_;

public:
    MotionMatching(MocapSkeleton *skeleton, MotionDatabase *database)
        : characterSpeedPlots_("Character Speed", "[sec]", "[m/s] or [rad/s]"),
          speedProfilePlots_("Speed Profile", "[sec]", "[m/s] or [rad/s]"),
          eeSpeedPlots_("Feet Speed", "[sec]", "[m/s]") {
        this->skeleton_ = skeleton;
        this->database_ = database;
        jointInertializationInfos_.resize(skeleton->getMarkerCount());

        // set initial motion
        resetMotion();

        // plots
        characterSpeedPlots_.addLineSpec({"forward", [](const auto &d) { return (float)d.x(); }});
        characterSpeedPlots_.addLineSpec({"sideways", [](const auto &d) { return (float)d.y(); }});
        characterSpeedPlots_.addLineSpec({"turning", [](const auto &d) { return (float)d.z(); }});
        speedProfilePlots_.addLineSpec({"forward", [](const auto &d) { return (float)d.x(); }});
        speedProfilePlots_.addLineSpec({"sideways", [](const auto &d) { return (float)d.y(); }});
        speedProfilePlots_.addLineSpec({"turning", [](const auto &d) { return (float)d.z(); }});
        eeSpeedPlots_.addLineSpec({"fl", [](const auto &d) { return (float)d[0]; }});
        eeSpeedPlots_.addLineSpec({"hl", [](const auto &d) { return (float)d[1]; }});
        eeSpeedPlots_.addLineSpec({"fr", [](const auto &d) { return (float)d[2]; }});
        eeSpeedPlots_.addLineSpec({"hr", [](const auto &d) { return (float)d[3]; }});
    }

    ~MotionMatching() = default;

    double getFrameTimeStep() {
        return dt_;
    }

    /**
     * reset motion (restart from the origin)
     */
    void resetMotion(std::string initDataset = "D1_ex01_KAN01_001.bvh", uint initFrame = 0) {
        t_wc = P3D(0, 0, 0);
        Q_wc = Quaternion::Identity();

        // pose of new motion at t=0
        t_wt0 = P3D(0, 0, 0);
        Q_wt0 = Quaternion::Identity();

        // pose of new motion at t=0 after stitching
        t_wt0prime = P3D(0, 0, 0);
        Q_wt0prime = Quaternion::Identity();

        // set initial motion
        currMotionIdx_ = lastMotionIdx_ = {0, 0};
        for (uint i = 0; i < this->database_->getClipCount(); i++) {
            if (database_->getClipByClipIndex(i)->getName() == initDataset && database_->getClipByClipIndex(i)->getFrameCount() - 60 > initFrame) {
                currMotionIdx_ = lastMotionIdx_ = {i, initFrame};
            }
        }
        skeleton_->setState(&database_->getMotionByMotionIndex(currMotionIdx_));

        // and reset inertialization infos
        rootPosInertializationInfo_ = InertializationInfo();
        rootQInertializationInfo_ = InertializationInfo();
        for (uint i = 0; i < jointInertializationInfos_.size(); i++) {
            jointInertializationInfos_[i] = InertializationInfo();
        }

        // reset queue
        queue_.clear();

        // reset foot fall pattern
        motionTime_ = 0;

        // reset plots
        characterSpeedPlots_.clearData();
        speedProfilePlots_.clearData();
        eeSpeedPlots_.clearData();

        // now fill up queue from starting
        if (queueSize > 0) {
            saveFutureMotionIntoQueue();
        }
    }

    /**
     * compute inertialization and update. here, old motion is current motion
     * sequence and new motion is the best match searched with a query vector.
     */
    void matchMotion() {
        dVector xq = createQueryVector();
        MotionIndex bestMatch = database_->searchBestMatchByQuery(xq);

        Logger::consolePrint("Best match: (%d, %d: %s) -> (%d, %d: %s)",  //
                             currMotionIdx_.first, currMotionIdx_.second - 1, database_->getClipByClipIndex(currMotionIdx_.first)->getName().c_str(),
                             bestMatch.first, bestMatch.second, database_->getClipByClipIndex(bestMatch.first)->getName().c_str());

        // here we need to match with previous frame!
        matchMotion({currMotionIdx_.first, currMotionIdx_.second - 1}, bestMatch);
    }

    /**
     * compute inertialization and update (for given motions).
     */
    void matchMotion(MotionIndex oldMotionIdx, MotionIndex newMotionIdx) {
        // old motion stitched
        MocapSkeletonState oldMotionMinus1 = computeStitchedMotion(database_->getMotionByMotionIndex({oldMotionIdx.first, oldMotionIdx.second - 1}));
        MocapSkeletonState oldMotion = computeStitchedMotion(database_->getMotionByMotionIndex(oldMotionIdx));

        // update Q_wc and t_wc
        t_wc = P3D(skeleton_->root->state.pos.x, 0, skeleton_->root->state.pos.z);
        Q_wc = computeCharacterQ(skeleton_->root->state.orientation, skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis));

        // update Q_wt0 and t_wt0
        MocapSkeletonState newMotion = database_->getMotionByMotionIndex(newMotionIdx);
        Q_wt0 = newMotion.getRootOrientation();
        t_wt0 = newMotion.getRootPosition();

        // update Q_wt0prime and t_wt0prime
        double roll, pitch, yaw;
        computeEulerAnglesFromQuaternion(Q_wt0,  //
                                         skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis), skeleton_->upAxis, roll, pitch, yaw);

        Q_wt0prime = Q_wc * getRotationQuaternion(pitch, skeleton_->forwardAxis.cross(skeleton_->upAxis)) * getRotationQuaternion(roll, skeleton_->forwardAxis);
        t_wt0prime = t_wc + V3D(0, t_wt0.y, 0);

        // compute inertialization
        newMotion.setRootOrientation(Q_wt0prime);
        newMotion.setRootPosition(t_wt0prime);
        computeInertialization(oldMotionMinus1, oldMotion, newMotion, transitionTime, dt_);

        // update motion idx
        lastMotionIdx_ = newMotionIdx;
        currMotionIdx_ = newMotionIdx;
        currMotionIdx_.second++;  // we play from next frame

        // clear queue first
        queue_.clear();

        // save future motions into queue if queueSize > 0
        if (queueSize > 0) {
            saveFutureMotionIntoQueue();
        }
    }

    /**
     * advance one frame
     */
    void advance() {
        if (queue_.size() > 0) {
            // if queue is not empty, play saved motions
            // remember! queue.front() is next frame!
            skeleton_->setState(&queue_.front());
            queue_.pop_front();
        } else {
            // if not, we need to compute new motion
            MocapSkeletonState newMotion_t = computeStitchedMotion(database_->getMotionByMotionIndex(currMotionIdx_));

            if (inertialization) {
                if (currMotionIdx_.first != lastMotionIdx_.first)
                    // well this will never happen but just in case...
                    throw std::runtime_error(
                        "MotionMatcher::advance() error: something wrong in "
                        "index. (current motion database = " +
                        std::to_string(currMotionIdx_.first) + " != " + "matched motion database = " + std::to_string(lastMotionIdx_.first) + ")");

                double t = (currMotionIdx_.second - lastMotionIdx_.second) * dt_;

                // here tMinus1 motion is current state of skeleton
                MocapSkeletonState newMotion_tMinus1(skeleton_);
                auto inertializedMotion = inertializeState(newMotion_t, newMotion_tMinus1, t, dt_);
                skeleton_->setState(&inertializedMotion);
            } else {
                skeleton_->setState(&newMotion_t);
            }
        }

        // plot
        {
            Quaternion characterQ =
                computeCharacterQ(skeleton_->root->state.orientation, skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis));

            V3D characterVel = skeleton_->root->state.velocity;
            characterVel = characterQ.inverse() * characterVel;
            V3D characterAngularVel = skeleton_->root->state.angularVelocity;
            characterAngularVel = characterQ.inverse() * characterAngularVel;

            V3D speed;
            speed.x() = characterVel.dot(skeleton_->forwardAxis);
            speed.y() = characterVel.dot(skeleton_->forwardAxis.cross(skeleton_->upAxis));
            speed.z() = characterAngularVel.dot(skeleton_->upAxis);
            characterSpeedPlots_.addData(motionTime_, speed);
        }

        // increase index
        currMotionIdx_.second++;
        motionTime_ += dt_;
    }

    void drawDebugInfo(const gui::Shader &shader) {
        // draw query info
        {
            // future trajectory
            dVector xq = createQueryVector();
            P3D characterPos = P3D(skeleton_->root->state.pos.x, 0, skeleton_->root->state.pos.z);
            Quaternion characterQ =
                computeCharacterQ(skeleton_->root->state.orientation, skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis));

            V3D tt1(xq[0], 0, xq[1]);
            tt1 = characterQ * tt1;
            V3D tt2(xq[2], 0, xq[3]);
            tt2 = characterQ * tt2;
            V3D tt3(xq[4], 0, xq[5]);
            tt3 = characterQ * tt3;
            drawSphere(characterPos + tt1, 0.02, shader, V3D(0, 0, 0), 0.5);
            drawSphere(characterPos + tt2, 0.02, shader, V3D(0, 0, 0), 0.5);
            drawSphere(characterPos + tt3, 0.02, shader, V3D(0, 0, 0), 0.5);

            // future heading
            V3D td1(xq[6], 0, xq[7]);
            td1 = characterQ * td1;
            V3D td2(xq[8], 0, xq[9]);
            td2 = characterQ * td2;
            V3D td3(xq[10], 0, xq[11]);
            td3 = characterQ * td3;
            drawArrow3d(characterPos + tt1, td1 * 0.15, 0.01, shader, V3D(0, 0, 0), 0.5);
            drawArrow3d(characterPos + tt2, td2 * 0.15, 0.01, shader, V3D(0, 0, 0), 0.5);
            drawArrow3d(characterPos + tt3, td3 * 0.15, 0.01, shader, V3D(0, 0, 0), 0.5);

            // foot position
            V3D ft1(xq[12], xq[13], xq[14]);
            ft1 = characterQ * ft1;
            V3D ft2(xq[15], xq[16], xq[17]);
            ft2 = characterQ * ft2;
            V3D ft3(xq[18], xq[19], xq[20]);
            ft3 = characterQ * ft3;
            V3D ft4(xq[21], xq[22], xq[23]);
            ft4 = characterQ * ft4;

            // foot velocity
            V3D ft1dot(xq[24], xq[25], xq[26]);
            ft1dot = characterQ * ft1dot;
            V3D ft2dot(xq[27], xq[28], xq[29]);
            ft2dot = characterQ * ft2dot;
            V3D ft3dot(xq[30], xq[31], xq[32]);
            ft3dot = characterQ * ft3dot;
            V3D ft4dot(xq[33], xq[34], xq[35]);
            ft4dot = characterQ * ft4dot;

            // contact
            V3D f1color(1, 1, 0);
            V3D f2color(1, 1, 0);
            V3D f3color(1, 1, 0);
            V3D f4color(1, 1, 0);
            if (ft1.y() < feetHeightThreshold && ft1dot.norm() < feetSpeedThreshold)
                f1color = V3D(0, 1, 1);
            if (ft2.y() < feetHeightThreshold && ft2dot.norm() < feetSpeedThreshold)
                f2color = V3D(0, 1, 1);
            if (ft3.y() < feetHeightThreshold && ft3dot.norm() < feetSpeedThreshold)
                f3color = V3D(0, 1, 1);
            if (ft4.y() < feetHeightThreshold && ft4dot.norm() < feetSpeedThreshold)
                f4color = V3D(0, 1, 1);

            drawSphere(characterPos + ft1, 0.02, shader, f1color, 0.5);
            drawSphere(characterPos + ft2, 0.02, shader, f2color, 0.5);
            drawSphere(characterPos + ft3, 0.02, shader, f3color, 0.5);
            drawSphere(characterPos + ft4, 0.02, shader, f4color, 0.5);

            drawArrow3d(characterPos + ft1, ft1dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);
            drawArrow3d(characterPos + ft2, ft2dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);
            drawArrow3d(characterPos + ft3, ft3dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);
            drawArrow3d(characterPos + ft4, ft4dot * 0.15, 0.01, shader, V3D(1, 0.55, 0), 0.5);

            // root velocity
            // let's just draw from character pos (for projecting to ground)
            V3D htdot(xq[36], xq[37], xq[38]);
            htdot = characterQ * htdot;
            drawSphere(characterPos, 0.02, shader, V3D(1, 0, 1), 0.5);
            drawArrow3d(characterPos, htdot * 0.15, 0.01, shader, V3D(1, 0, 1), 0.5);
        }

        // draw best match info (currently playing sequence)
        {
            const auto &currentClip = database_->getClipByClipIndex(currMotionIdx_.first);
            for (uint i = currMotionIdx_.second + 20; i <= currMotionIdx_.second + 60; i += 20) {
                if (i >= currentClip->getFrameCount())
                    break;

                // future motion
                auto motionAfter_i = computeStitchedMotion(currentClip->getState(i));

                // pos
                P3D pos = motionAfter_i.getRootPosition();
                pos.y = 0;
                drawSphere(pos, 0.02, shader, V3D(1, 0, 0), 0.5);

                // heading
                Quaternion q = motionAfter_i.getRootOrientation();
                q = computeCharacterQ(q, skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis));

                V3D heading = q * skeleton_->forwardAxis;
                heading.y() = 0;
                heading.normalize();

                drawArrow3d(pos, heading * 0.15, 0.01, shader, V3D(1, 0, 0), 0.5);
            }
        }
    }

    void plotDebugInfo() {
        characterSpeedPlots_.draw();
        speedProfilePlots_.draw();
        eeSpeedPlots_.draw();
    }

private:
    /**
     * Compute inertialization infos (coefficients, x0, v0, a0, t1, etc.)
     * 
     * t1 is user's desired transition time (t1 = 0.5 is desirable...)
     * dt is one frame timestep
     */
    void computeInertialization(const MocapSkeletonState &oldMinus1Motion, const MocapSkeletonState &oldMotion, const MocapSkeletonState &newMotion, double t1,
                                double dt) {
        // root
        {
            // position
            V3D oldMinus1Pos = V3D(oldMinus1Motion.getRootPosition());
            V3D oldPos = V3D(oldMotion.getRootPosition());
            V3D newPos = V3D(newMotion.getRootPosition());

            rootPosInertializationInfo_ = computeInertialization(oldMinus1Pos, oldPos, newPos, dt, t1);

            // orientation
            Quaternion oldMinus1Q = oldMinus1Motion.getRootOrientation();
            Quaternion oldQ = oldMotion.getRootOrientation();
            Quaternion newQ = newMotion.getRootOrientation();

            rootQInertializationInfo_ = computeInertialization(oldMinus1Q, oldQ, newQ, dt, t1);
        }

        // joints
        for (uint i = 0; i < skeleton_->getMarkerCount(); i++) {
            Quaternion oldMinus1Q = oldMinus1Motion.getJointRelativeOrientation(i);
            Quaternion oldQ = oldMotion.getJointRelativeOrientation(i);
            Quaternion newQ = newMotion.getJointRelativeOrientation(i);

            jointInertializationInfos_[i] = computeInertialization(oldMinus1Q, oldQ, newQ, dt, t1);
        }
    }

    /**
     * initialize state at time t based on computed inertialization infos.
     * 
     * motion_t is a motion from new sequence at time t
     * t is a time since the last motion matching happened
     * dt is a frame timestep (for computing velocity by finite difference)
     */
    MocapSkeletonState inertializeState(const MocapSkeletonState &motion_t, const MocapSkeletonState &motion_tMinus1, double t, double dt) {
        // we will return inertializedMotion_t
        MocapSkeletonState inertializedMotion_t = motion_tMinus1;

        // root
        {
            // position
            V3D rootPosV = V3D(motion_t.getRootPosition());
            P3D pos_tminus1 = inertializedMotion_t.getRootPosition();

            V3D inertializedPosV = inertializeVector(rootPosInertializationInfo_, rootPosV, t);
            inertializedMotion_t.setRootPosition(P3D() + inertializedPosV);
            inertializedMotion_t.setRootVelocity((inertializedPosV - V3D(pos_tminus1)) / dt);

            // orientation
            Quaternion rootQ = motion_t.getRootOrientation();
            Quaternion q_tminus1 = inertializedMotion_t.getRootOrientation();

            Quaternion inertializedQ = inertializeQuaternion(rootQInertializationInfo_, rootQ, t);
            inertializedMotion_t.setRootOrientation(inertializedQ);
            inertializedMotion_t.setRootAngularVelocity(estimateAngularVelocity(q_tminus1, inertializedQ, dt));
        }

        // joints
        for (uint i = 0; i < skeleton_->getMarkerCount(); i++) {
            Quaternion jointRelQ = motion_t.getJointRelativeOrientation(i);
            Quaternion q_tminus1 = inertializedMotion_t.getJointRelativeOrientation(i);
            auto &inertializeInfo = jointInertializationInfos_[i];

            Quaternion inertializedQ = inertializeQuaternion(inertializeInfo, jointRelQ, t);
            inertializedMotion_t.setJointRelativeOrientation(inertializedQ, i);
            inertializedMotion_t.setJointRelativeAngVelocity(estimateAngularVelocity(q_tminus1, inertializedQ, dt), i);
        }

        return inertializedMotion_t;
    }

    InertializationInfo computeInertialization(Quaternion &oldPrevQ, Quaternion &oldQ, Quaternion &newQ, double dt, double t1) {
        Quaternion q0 = oldQ * newQ.inverse();
        Quaternion q_minus1 = oldPrevQ * newQ.inverse();

        // compute x0
        if (q0.vec().norm() < 1e-10) {
            // then we can just say q0 is identity...
            // just play coming sequences as it is
            InertializationInfo info;
            return info;
        }

        // if q0 is not identity
        V3D x0v = q0.vec().normalized();
        double x0 = getRotationAngle(q0, x0v);

        // compute x_minus1
        double x_minus1 = 2 * atan2(q_minus1.vec().dot(x0v), q_minus1.w());

        // compute v0 and a0
        // note.
        // if x0 >= 0 then v0 should be < 0 and a0 should be > 0
        // if x0 < 0 then v0 should be > 0 and a0 should be < 0
        // if this is not the case, just clamp v0 or a0

        // v0 = (x0 - x_minus1) / dt
        double v0 = (x0 - x_minus1) / dt;

        if ((x0 > 0 && v0 < 0) || (x0 < 0 && v0 > 0))
            // t1 = min(t1, -5 * x0/v0)
            t1 = std::min(t1, -5 * x0 / v0);

        // a0 = (-8 * v0 * t1 - 20 * x0) / (t1^2)
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        if (x0 > 0) {
            if (v0 > 0)
                v0 = 0;
            if (a0 < 0)
                a0 = 0;
        } else {
            if (v0 < 0)
                v0 = 0;
            if (a0 > 0)
                a0 = 0;
        }

        // A = - (a0 * t1^2 + 6 * v0 * t1 + 12 * x0) / (2 * t1^5)
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        // B = (3 * a0 * t1^2 + 16 * v0 * t1 + 30 * x0) / (2 * t1^4)
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        // C = - (3 * a0 * t1^2 + 12 * v0 * t1 + 20 * x0) / (2 * t1^3)
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.x0v = x0v;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    InertializationInfo computeInertialization(V3D &oldPrevV3D, V3D &oldV3D, V3D &newV3D, double dt, double t1) {
        V3D x0v = oldV3D - newV3D;
        V3D x_minus1v = oldPrevV3D - newV3D;

        double x0 = x0v.norm();
        if (x0 < 1e-10) {
            // can be consider there's no change at all
            InertializationInfo info;
            return info;
        }

        // compute x_minus1
        double x_minus1 = x_minus1v.dot(x0v.normalized());

        // compute v0 and a0
        // note.
        // if x0 >= 0 then v0 should be < 0 and a0 should be > 0
        // if x0 < 0 then v0 should be > 0 and a0 should be < 0
        // if this is not the case, just clamp v0 or a0

        // v0 = (x0 - x_minus1) / dt
        double v0 = (x0 - x_minus1) / dt;

        if ((x0 > 0 && v0 < 0) || (x0 < 0 && v0 > 0))
            // t1 = min(t1, -5 * x0/v0)
            t1 = std::min(t1, -5 * x0 / v0);

        // a0 = (-8 * v0 * t1 - 20 * x0) / (t1^2)
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        if (x0 > 0) {
            if (v0 > 0)
                v0 = 0;
            if (a0 < 0)
                a0 = 0;
        } else {
            if (v0 < 0)
                v0 = 0;
            if (a0 > 0)
                a0 = 0;
        }

        // A = - (a0 * t1^2 + 6 * v0 * t1 + 12 * x0) / (2 * t1^5)
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        // B = (3 * a0 * t1^2 + 16 * v0 * t1 + 30 * x0) / (2 * t1^4)
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        // C = - (3 * a0 * t1^2 + 12 * v0 * t1 + 20 * x0) / (2 * t1^3)
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.x0v = x0v;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    /**
     * inertialize quaternion newQ_t at time t.
     */
    Quaternion inertializeQuaternion(InertializationInfo &info, Quaternion &newQ_t, double t) {
        if (t >= info.t1)
            // inertialization is done just return newQ
            return newQ_t;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double x = info.A * t * t * t * t * t  //
                   + info.B * t * t * t * t    //
                   + info.C * (t * t * t)      //
                   + 0.5 * info.a0 * t * t     //
                   + info.v0 * t               //
                   + info.x0;                  //

        Quaternion dq = Quaternion(cos(0.5 * x),                 //
                                   sin(0.5 * x) * info.x0v.x(),  //
                                   sin(0.5 * x) * info.x0v.y(),  //
                                   sin(0.5 * x) * info.x0v.z())
                            .normalized();

        return dq * newQ_t;
    }

    /**
     * inertialize vector newV_t at time t.
     */
    V3D inertializeVector(InertializationInfo &info, V3D &newV_t, double t) {
        if (t >= info.t1)
            // inertialization done just return newV
            return newV_t;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double x = info.A * t * t * t * t * t  //
                   + info.B * t * t * t * t    //
                   + info.C * (t * t * t)      //
                   + 0.5 * info.a0 * t * t     //
                   + info.v0 * t               //
                   + info.x0;                  //

        V3D dv = V3D(info.x0v.normalized()) * x;

        return newV_t + dv;
    }

    /**
     * create query vector from current skeleton's state and user command.
     */
    dVector createQueryVector() {
        Quaternion rootQ = skeleton_->root->state.orientation;
        P3D rootPos = skeleton_->root->state.pos;

        // becareful! skeleton's forward direction is x axis not z axis!
        double roll, pitch, yaw;
        computeEulerAnglesFromQuaternion(rootQ,  //
                                         skeleton_->forwardAxis, skeleton_->forwardAxis.cross(skeleton_->upAxis), skeleton_->upAxis, roll, pitch, yaw);

        // current character frame
        Quaternion characterQ = getRotationQuaternion(yaw, skeleton_->upAxis);
        P3D characterPos(rootPos.x, 0, rootPos.z);
        double characterYaw = yaw;

        // generate trajectory (of character)
        Trajectory3D queryPosTrajectory;
        Trajectory1D queryHeadingTrajectory;
        Trajectory3D querySpeedTrajectory;

        {
            P3D pos = characterPos;
            double headingAngle = characterYaw;
            double vForward = this->speedForward;
            double vSideways = this->speedSideways;
            double vTurning = this->turningSpeed;

            double dtTraj = 1.0 / 60;  // trajectory dt
            double t = 0;
            while (t <= 1.0) {
                Quaternion headingQ = getRotationQuaternion(headingAngle, skeleton_->upAxis);

                queryPosTrajectory.addKnot(t, V3D(pos));
                queryHeadingTrajectory.addKnot(t, headingAngle);
                querySpeedTrajectory.addKnot(t, V3D(vForward, vSideways, vTurning));

                vForward = speedForward;
                vSideways = speedSideways;
                vTurning = turningSpeed;

                // update the position and heading of the body frame based on target
                // speeds...
                pos = pos + dtTraj * (headingQ * (skeleton_->forwardAxis * vForward + skeleton_->forwardAxis.cross(skeleton_->upAxis) * vSideways));
                headingAngle += dtTraj * vTurning;

                t += dtTraj;
            }
        }

        // build query vector
        dVector xq(39);

        // 1/2: 2D projected future trajectory
        // TODO: check if 20/40/60 * dt is correct
        // after 20 frames
        P3D characterPosAfter20 = P3D() + queryPosTrajectory.evaluate_linear(20 * dt_);
        V3D tt1(characterPos, characterPosAfter20);
        tt1 = characterQ.inverse() * tt1;

        Quaternion characterQAfter20 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(20 * dt_), skeleton_->upAxis);
        V3D td1 = characterQAfter20 * skeleton_->forwardAxis;
        td1 = characterQ.inverse() * td1;
        td1.y() = 0;
        td1.normalize();

        // after 40 frames
        P3D characterPosAfter40 = P3D() + queryPosTrajectory.evaluate_linear(40.0 * dt_);
        V3D tt2(characterPos, characterPosAfter40);
        tt2 = characterQ.inverse() * tt2;

        Quaternion characterQAfter40 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(40 * dt_), skeleton_->upAxis);
        V3D td2 = characterQAfter40 * skeleton_->forwardAxis;
        td2 = characterQ.inverse() * td2;
        td2.y() = 0;
        td2.normalize();

        // after 60 frames
        P3D characterPosAfter60 = P3D() + queryPosTrajectory.evaluate_linear(60 * dt_);
        V3D tt3(characterPos, characterPosAfter60);
        tt3 = characterQ.inverse() * tt3;

        Quaternion characterQAfter60 = getRotationQuaternion(queryHeadingTrajectory.evaluate_linear(60 * dt_), skeleton_->upAxis);
        V3D td3 = characterQAfter60 * skeleton_->forwardAxis;
        td3 = characterQ.inverse() * td3;
        td3.y() = 0;
        td3.normalize();

        // 3/4: feet positions and velocities w.r.t character frame (in R^12)
        auto *flJoint = skeleton_->getMarkerByName("LeftHand");
        auto *hlJoint = skeleton_->getMarkerByName("LeftFoot");
        auto *frJoint = skeleton_->getMarkerByName("RightHand");
        auto *hrJoint = skeleton_->getMarkerByName("RightFoot");

        P3D flFeetPos = flJoint->state.getWorldCoordinates(flJoint->endSites[0].endSiteOffset);
        P3D hlFeetPos = hlJoint->state.getWorldCoordinates(hlJoint->endSites[0].endSiteOffset);
        P3D frFeetPos = frJoint->state.getWorldCoordinates(frJoint->endSites[0].endSiteOffset);
        P3D hrFeetPos = hrJoint->state.getWorldCoordinates(hrJoint->endSites[0].endSiteOffset);

        V3D ft1 = characterQ.inverse() * V3D(characterPos, flFeetPos);
        V3D ft2 = characterQ.inverse() * V3D(characterPos, hlFeetPos);
        V3D ft3 = characterQ.inverse() * V3D(characterPos, frFeetPos);
        V3D ft4 = characterQ.inverse() * V3D(characterPos, hrFeetPos);

        V3D ft1dot = flJoint->state.getVelocityForPoint_local(flJoint->endSites[0].endSiteOffset);
        V3D ft2dot = hlJoint->state.getVelocityForPoint_local(hlJoint->endSites[0].endSiteOffset);
        V3D ft3dot = frJoint->state.getVelocityForPoint_local(frJoint->endSites[0].endSiteOffset);
        V3D ft4dot = hrJoint->state.getVelocityForPoint_local(hrJoint->endSites[0].endSiteOffset);

        ft1dot = characterQ.inverse() * ft1dot;
        ft2dot = characterQ.inverse() * ft2dot;
        ft3dot = characterQ.inverse() * ft3dot;
        ft4dot = characterQ.inverse() * ft4dot;

        // 5: hip (root) joint velocity w.r.t character frame (in R^3)
        V3D htdot = skeleton_->root->state.velocity;
        htdot = characterQ.inverse() * htdot;

        // save to feature vector
        xq[0] = tt1.x();
        xq[1] = tt1.z();
        xq[2] = tt2.x();
        xq[3] = tt2.z();
        xq[4] = tt3.x();
        xq[5] = tt3.z();

        xq[6] = td1.x();
        xq[7] = td1.z();
        xq[8] = td2.x();
        xq[9] = td2.z();
        xq[10] = td3.x();
        xq[11] = td3.z();

        xq[12] = ft1.x();
        xq[13] = ft1.y();
        xq[14] = ft1.z();
        xq[15] = ft2.x();
        xq[16] = ft2.y();
        xq[17] = ft2.z();
        xq[18] = ft3.x();
        xq[19] = ft3.y();
        xq[20] = ft3.z();
        xq[21] = ft4.x();
        xq[22] = ft4.y();
        xq[23] = ft4.z();

        xq[24] = ft1dot.x();
        xq[25] = ft1dot.y();
        xq[26] = ft1dot.z();
        xq[27] = ft2dot.x();
        xq[28] = ft2dot.y();
        xq[29] = ft2dot.z();
        xq[30] = ft3dot.x();
        xq[31] = ft3dot.y();
        xq[32] = ft3dot.z();
        xq[33] = ft4dot.x();
        xq[34] = ft4dot.y();
        xq[35] = ft4dot.z();

        xq[36] = htdot.x();
        xq[37] = htdot.y();
        xq[38] = htdot.z();

        return xq;
    }

    Quaternion computeCharacterQ(Quaternion &rootQ, V3D xAxis, V3D zAxis) {
        double roll, pitch, yaw;
        computeEulerAnglesFromQuaternion(rootQ,  //
                                         xAxis, zAxis, zAxis.cross(xAxis), roll, pitch, yaw);
        return getRotationQuaternion(yaw, zAxis.cross(xAxis));
    }

    /**
     * motion stitched to current skeleton pose
     */
    MocapSkeletonState computeStitchedMotion(const MocapSkeletonState &state) {
        MocapSkeletonState stitchedMotion = state;
        Quaternion Q_wtt = stitchedMotion.getRootOrientation();
        P3D t_wtt = stitchedMotion.getRootPosition();

        Quaternion Q_wttprime = Q_wt0prime * Q_wt0.inverse() * Q_wtt;
        P3D t_wttprime = t_wt0prime + Q_wt0prime * Q_wt0.inverse() * (V3D(t_wt0, t_wtt));
        stitchedMotion.setRootOrientation(Q_wttprime);
        stitchedMotion.setRootPosition(t_wttprime);
        return stitchedMotion;
    }

    /**
     * save future motions from next frame which will be played by advance() 
     * function. 
     */
    void saveFutureMotionIntoQueue() {
        // back up for t-1
        MocapSkeletonState newMotion_tMinus1(skeleton_);

        // save inertialized motion to queue (from next frame)
        for (uint i = 0; i < queueSize; i++) {
            if (currMotionIdx_.second + i >= database_->getClipByClipIndex(currMotionIdx_.first)->getFrameCount())
                throw std::runtime_error(
                    "MotionMatcher::matchMotion() error: something "
                    "wrong in index (current index = " +
                    std::to_string(currMotionIdx_.second) + " + " + std::to_string(i) +
                    " >= data size = " + std::to_string(database_->getClipByClipIndex(currMotionIdx_.first)->getFrameCount()) + ")");

            MocapSkeletonState newMotion_t = computeStitchedMotion(database_->getMotionByMotionIndex({currMotionIdx_.first, currMotionIdx_.second + i}));

            double t = i * dt_;

            if (inertialization) {
                auto inertializedMotion = inertializeState(newMotion_t, newMotion_tMinus1, t, dt_);

                // save inertialized motion to queue
                queue_.push_back(inertializedMotion);
                newMotion_tMinus1 = inertializedMotion;
            } else {
                MocapSkeletonState newMotion_t = computeStitchedMotion(database_->getMotionByMotionIndex({currMotionIdx_.first, currMotionIdx_.second + i}));

                queue_.push_back(newMotion_t);
            }
        }
    }
};

}  // namespace crl::mocap