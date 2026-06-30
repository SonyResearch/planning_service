/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file test_types.cc

#include <gtest/gtest.h>

#include "planning_service/common/file_utils.h"
#include "planning_service/common/string_utils.h"
#include "planning_service/service/utils/utils.h"

namespace service {
namespace utils {

class UtilsTest : public ::testing::Test {
 protected:
  /**
   * @brief Copy the `iiwa` data into a temp dir to load into the TestManager,
   * and create a new manager instance.
   */
  void SetUp() override {
    const fs::path data_dir {"planning_service/test_data/wallflower/"};
    temp_dir_ = common::utils::temp_dir();
    temp_data_dir_ = temp_dir_ / "wallflower";
    fs::create_directories(temp_data_dir_);
    fs::copy(data_dir, temp_data_dir_,
             fs::copy_options::recursive | fs::copy_options::update_existing);

    sys_conf_0 = {{"robot", Eigen::Vector2d(0.0, 0.2)}};
    sys_conf_1 = {{"robot", Eigen::Vector2d(2.0, 0.2)}};
    dmd = drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
        temp_data_dir_ / "dmd.yaml");
    const auto flower_raw {
        common::utils::FileToString(temp_data_dir_ / "wallflower_flower.urdf")};
    const auto wall_raw {
        common::utils::FileToString(temp_data_dir_ / "wallflower_wall.urdf")};
    ModelFile flower_modelfile {"wallflower_flower", ModelFormat::URDF,
                                "test_models", "", flower_raw};
    ModelFile wall_modelfile {"wallflower_wall", ModelFormat::URDF,
                              "test_models", "", wall_raw};
    Model wall_model, robot_model;
    wall_model.name = "wall";
    wall_model.file = wall_modelfile;
    robot_model.name = "robot";
    robot_model.file = flower_modelfile;
    std::vector<Model> models {wall_model, robot_model};
    ctx = std::make_shared<PlanContext>("wallflower", dmd, models,
                                        motion::ConstraintsAdapter());
  }

  void TearDown() override {
    fs::remove_all(temp_data_dir_.parent_path());
  }

  fs::path temp_data_dir_;
  fs::path temp_dir_;
  system_conf_t sys_conf_0, sys_conf_1;
  drake::multibody::parsing::ModelDirectives dmd;
  std::shared_ptr<PlanContext> ctx;
};

TEST_F(UtilsTest, SaveModelFile) {
  const auto package_name {"test_models"};
  const auto urdf_src_path {temp_data_dir_ / "wallflower_flower.urdf"};
  service::ModelFile modelfile;
  modelfile.name = "wallflower_flower_duplicate";
  modelfile.format = service::ModelFormat::URDF;
  modelfile.package_name = package_name;
  modelfile.content = common::utils::FileToString(urdf_src_path);
  EXPECT_TRUE(SaveModelFile(modelfile, temp_data_dir_));
  EXPECT_TRUE(fs::exists(temp_data_dir_ / "wallflower_flower_duplicate.urdf"))
      << "URDF was not created";
  EXPECT_EQ(modelfile.content,
            common::utils::FileToString(temp_data_dir_
                                        / "wallflower_flower_duplicate.urdf"))
      << "URDF contents are not consistent";
}

TEST_F(UtilsTest, SavePackageToXml) {
  const auto package_name {"test_tmp_models"};
  const auto urdf_dir {temp_dir_};
  fs::remove(urdf_dir / "package.xml");
  EXPECT_TRUE(SavePackageToXml(package_name, urdf_dir));
  EXPECT_TRUE(fs::exists(urdf_dir / "package.xml"))
      << "package.xml file was not created";
}

TEST_F(UtilsTest, SaveMeshFile) {
  const fs::path mesh_path {
      "planning_service/test_data/alfred/urdf/meshes/visual/"
      "bowl_cg_pulp_48oz.obj"};
  service::MeshFile meshfile {
      .name = "bowl",
      .extension = ".obj",
      .package_name = "test_models",
      .parent_path = "",
      .collision = true,
      .content = common::utils::FileToString(mesh_path)};
  EXPECT_TRUE(SaveMeshFile(meshfile, temp_data_dir_));
  EXPECT_TRUE(fs::exists(temp_data_dir_ / "bowl.obj"))
      << "mesh was not created";
  EXPECT_EQ(meshfile.content,
            common::utils::FileToString(temp_data_dir_ / "bowl.obj"))
      << "mesh contents are not consistent";
}

TEST_F(UtilsTest, ContextIsValid) {
  std::vector<Model> models;
  motion::ConstraintsAdapter constraints_adapter;
  const PlanContext no_models_ctx {"wallflower", dmd, models,
                                   constraints_adapter};

  EXPECT_FALSE(ContextIsValid(no_models_ctx))
      << "context with no models should not be valid!";
  const auto flower_raw {
      common::utils::FileToString(temp_data_dir_ / "wallflower_flower.urdf")};
  const auto wall_raw {
      common::utils::FileToString(temp_data_dir_ / "wallflower_wall.urdf")};
  ModelFile flower_modelfile {"wallflower_flower", ModelFormat::URDF,
                              "test_models", "", flower_raw};
  ModelFile wall_modelfile {"wallflower_wall", ModelFormat::URDF, "test_models",
                            "", wall_raw};
  Model wall_model, robot_model;
  wall_model.name = "wall";
  wall_model.file = wall_modelfile;
  robot_model.name = "robot";
  models.push_back(wall_model);
  models.push_back(robot_model);
  const PlanContext model_no_geo_ctx {"wallflower", dmd, models,
                                      constraints_adapter};
  EXPECT_FALSE(ContextIsValid(model_no_geo_ctx))
      << "a model with no geometry is not valid!";

  models.erase(models.end());
  const PlanContext missing_model_ctx {"wallflower", dmd, models,
                                       constraints_adapter};
  EXPECT_FALSE(ContextIsValid(missing_model_ctx))
      << "context missing one model is not valid!";

  robot_model.name = "not_the_right_robot";
  robot_model.file = flower_modelfile;
  models.push_back(robot_model);
  const PlanContext misnamed_model_ctx {"wallflower", dmd, models,
                                        constraints_adapter};
  EXPECT_FALSE(ContextIsValid(misnamed_model_ctx))
      << "model with incorrect name is not valid!";

  models.erase(models.end());
  robot_model.name = "robot";
  robot_model.file.package_name = "not_test_models";
  models.push_back(robot_model);
  const PlanContext package_mismatch_ctx {"wallflower", dmd, models,
                                          constraints_adapter};
  EXPECT_FALSE(ContextIsValid(package_mismatch_ctx))
      << "models from multiple packages are not valid!";

  robot_model.file.package_name = "test_models";
  models.erase(models.end());
  models.push_back(robot_model);
  const PlanContext no_collision_check_ctx {"wallflower", dmd, models,
                                            constraints_adapter};
  EXPECT_FALSE(ContextIsValid(no_collision_check_ctx))
      << "model with no collision checker is not valid!";

  models.push_back(robot_model);
  const PlanContext extra_model_ctx {"wallflower", dmd, models,
                                     constraints_adapter};
  EXPECT_FALSE(ContextIsValid(extra_model_ctx))
      << "context with more models than in DMD is not valid!";

  models.erase(models.end());
  constraints_adapter.collision_checker = motion::CollisionCheckerAdapter();
  const PlanContext valid_ctx {"wallflower", dmd, models, constraints_adapter};
  EXPECT_TRUE(ContextIsValid(valid_ctx))
      << "context with all correct data should be valid!";
}

TEST_F(UtilsTest, SaveContext) {
  json j;
  std::cout << "json: " << j << std::endl;
  std::cout << "ctx json: " << ctx->metadata << std::endl;
  EXPECT_FALSE(SaveContext(*ctx, temp_data_dir_))
      << "context without an ID cannot be saved!";
  ctx->id = 1000;
  EXPECT_TRUE(SaveContext(*ctx, temp_data_dir_))
      << "valid context was not saved successfully!";
  EXPECT_TRUE(fs::is_directory(temp_data_dir_ / "1000"))
      << "context directory was not created!";
  EXPECT_TRUE(fs::is_regular_file(temp_data_dir_ / "1000" / "dmd.yaml"));
  EXPECT_TRUE(
      fs::is_regular_file(temp_data_dir_ / "1000" / "constraints.yaml"));
  EXPECT_FALSE(fs::is_regular_file(temp_data_dir_ / "1000" / "metadata.json"));
  ctx->metadata["scene_name"] = "wallflower";
  ctx->metadata["description"] = "a test context";
  EXPECT_TRUE(SaveContext(*ctx, temp_data_dir_));
  EXPECT_TRUE(fs::is_regular_file(temp_data_dir_ / "1000" / "metadata.json"));
}

TEST_F(UtilsTest, LoadContext) {
  ctx->id = 1000;
  EXPECT_TRUE(SaveContext(*ctx, temp_data_dir_));
  PlanContext loaded_no_metadata_ctx {1000};
  EXPECT_TRUE(LoadContext(loaded_no_metadata_ctx, temp_data_dir_));
  EXPECT_FALSE(loaded_no_metadata_ctx.metadata.empty())
      << "context lacking metadata should have had a new one created!";
  EXPECT_EQ(loaded_no_metadata_ctx.metadata["scene_name"],
            "unnamed_context-1000");

  ctx->id = 1001;
  ctx->metadata["scene_name"] = "wallflower";
  ctx->metadata["description"] = "a test context";
  EXPECT_TRUE(SaveContext(*ctx, temp_data_dir_));
  PlanContext loaded_metadata_ctx {1001};
  EXPECT_TRUE(LoadContext(loaded_metadata_ctx, temp_data_dir_));
  EXPECT_FALSE(loaded_metadata_ctx.metadata.empty())
      << "context with metadata should have loaded it!";
  EXPECT_EQ(loaded_metadata_ctx.metadata["scene_name"], "wallflower");
  EXPECT_EQ(loaded_metadata_ctx.metadata["description"], "a test context");
}

TEST_F(UtilsTest, RegisterPlanContext) {
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          temp_data_dir_ / "dmd.yaml")};

  std::vector<Model> models;
  motion::ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = motion::CollisionCheckerAdapter();
  ModelFile flower_file {};
  Model wall_model, robot_model;
  robot_model.name = "robot";
  robot_model.file = ModelFile(
      "wallflower_flower", ModelFormat::URDF, "test_models", "",
      common::utils::FileToString(temp_data_dir_ / "wallflower_flower.urdf"));
  models.push_back(robot_model);
  wall_model.name = "wall";
  wall_model.file = ModelFile(
      "wallflower_wall", ModelFormat::URDF, "test_models", "",
      common::utils::FileToString(temp_data_dir_ / "wallflower_wall.urdf"));
  models.push_back(wall_model);

  const PlanContext wallflower_ctx {"wallflower", dmd, models,
                                    constraints_adapter};
  const auto wallflower_id {RegisterPlanContext(wallflower_ctx, temp_dir_)};
  EXPECT_TRUE(wallflower_id.has_value())
      << "valid context did not return an ID!";
  EXPECT_EQ(wallflower_id->value, 10370042980164148007U)
      << "computed ID did not match expected for wallflower!";
}

}  // namespace utils
}  // namespace service
