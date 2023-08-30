#!/bin/bash
catkin_make tests --build ./build-tests/ --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Wno-unused -Og --coverage -fno-inline -fno-inline-small-functions -fno-default-inline" -DCMAKE_C_FLAGS="-Wall -Wno-unused --coverage -fno-inline -fno-inline-small-functions -fno-default-inline" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXE_LINKER_FLAGS="-lgcov"
echo "====== Running tests ========"
roscore & ./devel/lib/ects/ects-test
killall -9 roscore
echo "====== Running coverage ========"
rm -rf ./code_coverage.info
lcov --directory ./build-tests/ects/ --capture --output-file ./code_coverage.info -rc lcov_branch_coverage=1 -rc "lcov_excl_br_line=LCOV_EXCL_BR_LINE|ROS_[IWEF]"
lcov --remove ./code_coverage.info '/usr/*' 'boost/*' '*/test/*' '*/c++/*' '*/opt/*' '*/devel/include/*' '*nlohmann*' -o ./code_coverage.info -rc lcov_branch_coverage=1
rm -rf ./code_coverage_report
genhtml code_coverage.info --branch-coverage --output-directory ./code_coverage_report/
