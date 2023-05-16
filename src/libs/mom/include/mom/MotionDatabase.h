//
// Created by Dongho Kang on 07.12.21.
//

#ifndef CRL_MOCAP_MOTIONDATABASE_H
#define CRL_MOCAP_MOTIONDATABASE_H

#include "mocap/MocapClip.h"

namespace crl::mocap {

/**
 * motion index in database.
 */
typedef std::pair<int, int> MotionIndex;

/**
 * feature vector x and its corresponding idx.
 */
struct MocapFeature {
    // feature vector
    dVector x;
    // motion idx in MocapDataset
    uint motionIdx = -1;
    uint datasetIdx = -1;
};

/**
 * we build motion database X.
 */
class MotionDatabase {
public:
    explicit MotionDatabase(std::string &dataDirectoryPath) {
        // import clips
        importFilesFromDirectory(dataDirectoryPath);

        // index dataset
        indexDataset();
    }

    /**
     * search best match by query vector xq
     */
    MotionIndex searchBestMatchByQuery(const dVector &xq) {
        double minLoss = INFINITY;
        MotionIndex minIdx = {-1, -1};

        // normalized query
        dVector xqNormalized = (xq - mu_).array() / sigma_.array();

        for (const auto &f : features_) {
            dVector xNormalized = (f.x - mu_).array() / sigma_.array();

            double loss = (xNormalized - xqNormalized).norm();

            if (loss < minLoss) {
                minLoss = loss;
                minIdx = {f.datasetIdx, f.motionIdx};
            }
        }

        return minIdx;
    }

    uint getClipCount() {
        return clips_.size();
    }

    uint getTotalFrameCount() {
        uint cnt = 0;
        for (auto &c : clips_) {
            cnt += c->getFrameCount();
        }
        return cnt;
    }

    const std::unique_ptr<BVHClip> &getClipByClipIndex(uint clipIdx) const {
        if (clipIdx >= clips_.size())
            throw std::runtime_error("getMotionByMotionIndex error: wrong index");
        return clips_[clipIdx];
    }

    const MocapSkeletonState &getMotionByMotionIndex(MotionIndex idx) {
        if (idx.first >= clips_.size())
            throw std::runtime_error("getMotionByMotionIndex error: wrong index");
        if (idx.second >= clips_[idx.first]->getFrameCount())
            throw std::runtime_error("getMotionByMotionIndex error: wrong index");
        return clips_[idx.first]->getState(idx.second);
    }

private:
    void importFilesFromDirectory(std::string &dataDirectoryPath) {
        // load mocap clips from data directory.
        for (const auto &entry : fs::directory_iterator(dataDirectoryPath)) {
            try {
                if (entry.path().extension() == ".bvh") {
                    clips_.push_back(std::make_unique<crl::mocap::BVHClip>(entry.path()));
                }
            } catch (...) {
                crl::Logger::consolePrint("Failed to load %s file.\n", entry.path().c_str());
            }
        }

        // sort by name.
        std::sort(clips_.begin(), clips_.end(),                                                                     //
                  [](const std::unique_ptr<crl::mocap::BVHClip> &a, const std::unique_ptr<crl::mocap::BVHClip> &b)  //
                  { return a->getName() < b->getName(); });
    }

    /**
     * index the whole dataset into features and normalize
     */
    void indexDataset() {
        features_.clear();
        mu_ = dVector(featureDim_);
        mu_.setZero();
        sigma_ = dVector(featureDim_);
        sigma_.setConstant(1);

        // reserve features
        features_.reserve(getTotalFrameCount());

        // store feature vector of each frame of dataset
        //
        // 1. 2D projected future trajectory (after 20/40/60 frames) w.r.t. character frame (in R^6)
        // 2. future heading vectors (after 20/40/60 frames) w.r.t character frame (in R^6)
        // 3. feet positions w.r.t character frame (in R^12)
        // 4. feet velocities w.r.t character frame (in R^12)
        // 5. hip (root) joint velocity w.r.t character frame (in R^3)
        //
        // character frame is a frame of the root projected to xz plane i.e.
        // the coordinate frame has orientation psi (yaw of root) and origin at (x, 0, z)
        //
        // note. once we finish indexing, we need to normalize this vectors by
        // std. of whole dataset

        for (uint i = 0; i < clips_.size(); i++) {
            auto &c = clips_[i];
            auto *skeleton = c->getModel();
            // frame
            const V3D worldUp = skeleton->upAxis;
            const V3D worldForward = skeleton->forwardAxis;

            // becareful! we save 61 frames into one feature.
            for (uint j = 0; j < c->getFrameCount() - 60; j++) {
                auto &m = c->getState(j);

                // feature vector
                dVector x(featureDim_);

                // temporarily set state first (for forward kinematic)
                skeleton->setState(&m);

                Quaternion rootQ = m.getRootOrientation();
                P3D rootPos = m.getRootPosition();

                // becareful! skeleton's forward direction is x axis not z axis!
                double roll, pitch, yaw;
                computeEulerAnglesFromQuaternion(rootQ,  //
                                                 worldForward, worldForward.cross(worldUp), worldUp, roll, pitch, yaw);

                // current character frame
                Quaternion characterQ = getRotationQuaternion(yaw, worldUp);
                P3D characterPos(rootPos.x, 0, rootPos.z);

                // 1/2: 2D projected future trajectory and heading
                // after 20 frames
                P3D rootPosAfter20 = c->getState(j + 20).getRootPosition();
                V3D tt1(characterPos, rootPosAfter20);
                tt1 = characterQ.inverse() * tt1;

                Quaternion rootQAfter20 = c->getState(j + 20).getRootOrientation();
                V3D td1 = rootQAfter20 * worldForward;
                td1 = characterQ.inverse() * td1;
                td1.y() = 0;
                td1.normalize();

                // after 40 frames
                P3D rootPosAfter40 = c->getState(j + 40).getRootPosition();
                V3D tt2(characterPos, rootPosAfter40);
                tt2 = characterQ.inverse() * tt1;

                Quaternion rootQAfter40 = c->getState(j + 40).getRootOrientation();
                V3D td2 = rootQAfter40 * worldForward;
                td2 = characterQ.inverse() * td2;
                td2.y() = 0;
                td2.normalize();

                // after 60 frames
                P3D rootPosAfter60 = c->getState(j + 60).getRootPosition();
                V3D tt3(characterPos, rootPosAfter60);
                tt3 = characterQ.inverse() * tt3;

                Quaternion rootQAfter60 = c->getState(j + 60).getRootOrientation();
                V3D td3 = rootQAfter60 * worldForward;
                td3 = characterQ.inverse() * td3;
                td3.y() = 0;
                td3.normalize();

                // 3/4: feet positions and velocities w.r.t character frame (in R^12)
                auto *flJoint = skeleton->getMarkerByName("LeftHand");
                auto *hlJoint = skeleton->getMarkerByName("LeftFoot");
                auto *frJoint = skeleton->getMarkerByName("RightHand");
                auto *hrJoint = skeleton->getMarkerByName("RightFoot");

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
                V3D htdot = m.getRootVelocity();
                htdot = characterQ.inverse() * htdot;

                // save to feature vector
                x[0] = tt1.x();
                x[1] = tt1.z();
                x[2] = tt2.x();
                x[3] = tt2.z();
                x[4] = tt3.x();
                x[5] = tt3.z();

                x[6] = td1.x();
                x[7] = td1.z();
                x[8] = td2.x();
                x[9] = td2.z();
                x[10] = td3.x();
                x[11] = td3.z();

                x[12] = ft1.x();
                x[13] = ft1.y();
                x[14] = ft1.z();
                x[15] = ft2.x();
                x[16] = ft2.y();
                x[17] = ft2.z();
                x[18] = ft3.x();
                x[19] = ft3.y();
                x[20] = ft3.z();
                x[21] = ft4.x();
                x[22] = ft4.y();
                x[23] = ft4.z();

                x[24] = ft1dot.x();
                x[25] = ft1dot.y();
                x[26] = ft1dot.z();
                x[27] = ft2dot.x();
                x[28] = ft2dot.y();
                x[29] = ft2dot.z();
                x[30] = ft3dot.x();
                x[31] = ft3dot.y();
                x[32] = ft3dot.z();
                x[33] = ft4dot.x();
                x[34] = ft4dot.y();
                x[35] = ft4dot.z();

                x[36] = htdot.x();
                x[37] = htdot.y();
                x[38] = htdot.z();

                features_.emplace_back();
                features_.back().x = x;
                features_.back().motionIdx = j;
                features_.back().datasetIdx = i;
            }
        }

        // normalize features
        computeFeatureMeanAndVar();
    }

    void computeFeatureMeanAndVar() {
        uint N = features_.size();

        // mean of x and x^2
        dVector E_x(featureDim_);
        dVector E_xsquare(featureDim_);
        E_x.setZero();
        E_xsquare.setZero();

        for (uint i = 0; i < N; i++) {
            E_x += features_[i].x;
            E_xsquare += dVector(features_[i].x.array() * features_[i].x.array());
        }

        // mean
        mu_ = E_x = E_x / N;

        // std
        E_xsquare = E_xsquare / N;
        sigma_ = E_xsquare - dVector(mu_.array() * mu_.array());
        sigma_ = sigma_.cwiseSqrt();

        if (printDebugInfo) {
            std::cout << "mu = " << std::endl  //
                      << mu_ << std::endl
                      << std::endl;
            std::cout << "sigma = " << std::endl << sigma_ << std::endl << std::endl;
        }
    }

public:
    // options
    bool printDebugInfo = true;

private:
    std::vector<std::unique_ptr<BVHClip>> clips_;
    std::vector<MocapFeature> features_;
    const uint featureDim_ = 39;

    // mean and std of features
    dVector mu_, sigma_;
};

}  // namespace crl::mocap

#endif  //CRL_MOCAP_MOTIONDATABASE_H
