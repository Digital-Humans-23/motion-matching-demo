#pragma once

#include <crl-basic/gui/application.h>
#include <imgui_widgets/imfilebrowser.h>
#include <crl-basic/utils/logger.h>

#include "mom/MotionMatching.h"

namespace momApp {

class App : public crl::gui::ShadowApplication {
public:
    App() : crl::gui::ShadowApplication("MotionMatching App") {
        // load mann dataset
        std::string mocapPath = dataPath_ + "D1_ex01_KAN01_001.bvh";
        mocapSkeleton = new crl::mocap::MocapSkeleton(mocapPath.c_str());
        motionDatabase = new crl::mocap::MotionDatabase(dataPath_);
        motionMatching = new crl::mocap::MotionMatching(mocapSkeleton, motionDatabase);
        motionMatching->queueSize = 60;
    }

    ~App() override {
        delete mocapSkeleton;
        delete motionDatabase;
    }

    void process() override {
        static uint frame = 0;
        if (frame >= 30) {
            crl::Logger::consolePrint("transition happens!");
            motionMatching->matchMotion();
            frame = 0;
        }
        motionMatching->advance();
        frame++;

        camera.target.x = mocapSkeleton->root->state.pos.x;
        camera.target.z = mocapSkeleton->root->state.pos.z;
        light.target.x() = mocapSkeleton->root->state.pos.x;
        light.target.z() = mocapSkeleton->root->state.pos.z;
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        mocapSkeleton->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        mocapSkeleton->draw(shader);
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        if (ImGui::CollapsingHeader("Motion Control options", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::InputDouble("Speed forward", &motionMatching->speedForward, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
            ImGui::InputDouble("Speed turning", &motionMatching->turningSpeed, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
        }
        ImGui::End();
    }

    virtual bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            processIsRunning = !processIsRunning;
        }
        if (key == GLFW_KEY_ENTER) {
            if (!processIsRunning)
                process();
        }
        if (key == GLFW_KEY_BACKSPACE) {
            screenIsRecording = !screenIsRecording;
        }
        if (key == GLFW_KEY_UP) {
            motionMatching->speedForward += 0.3;
        }
        if (key == GLFW_KEY_DOWN) {
            motionMatching->speedForward -= 0.3;
        }
        if (key == GLFW_KEY_LEFT) {
            motionMatching->turningSpeed += 0.3;
        }
        if (key == GLFW_KEY_RIGHT) {
            motionMatching->turningSpeed -= 0.3;
        }
        return false;
    }

    bool mouseMove(double xpos, double ypos) override {
        if (mouseState.dragging == false) {
            crl::P3D rayOrigin;
            crl::V3D rayDirection;
            camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);
        }
        return crl::gui::ShadowApplication::mouseMove(xpos, ypos);
    }

    bool mouseButtonPressed(int button, int mods) override {
        return true;
    }

    bool drop(int count, const char **fileNames) override {
        return true;
    }

public:
    std::string dataPath_ = MOTION_MATCHING_DEMO_DATA_FOLDER "/mocap/mann/";
    crl::mocap::MocapSkeleton *mocapSkeleton = nullptr;
    crl::mocap::MotionDatabase *motionDatabase = nullptr;
    crl::mocap::MotionMatching *motionMatching = nullptr;
};

}  // namespace momApp