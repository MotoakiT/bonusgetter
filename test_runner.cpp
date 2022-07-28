#include "test_runner.h"

const int kTestRunParamsNum = 2;
const DrivingParam kTestRunParams[kTestRunParamsNum] = {
  { kTraceRightEdge, 50, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 1000, false},
  { kStopWheels, 0, { 0, 0, 0 }, kInvalidEnd, kInvalidColor, 0, false},
};


TestRunner::TestRunner(DrivingManager* driving_manager)
    : driving_manager_(driving_manager) {
  SetTestRunParam();
}

void TestRunner::SetTestRunParam() {
  for (int i=0; i<kTestRunParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kTestRunParams[i]);
    }
}

void TestRunner::Update() {
  driving_manager_->Update();
  if (driving_manager_->DrivingParamsEmpty()) {
    //state_ = kBlockBingo;
  }
}
