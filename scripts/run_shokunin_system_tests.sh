#!/usr/bin/env bash
set -euo pipefail  # good safety defaults (explained below)

run_test() {
    local TEST_NAME="$1"
    local OPTIONS_FILE="shokunin/config/${TEST_NAME}_test_options.yaml"
    local RESULTS_FILE="logs/shokunin/benchmark/${TEST_NAME}_test_results.yaml"
    local PLOT_FILE="logs/shokunin/benchmark/${TEST_NAME}_test_results.png"
    local CONTEXT_ID=12395773221843663529

    # Erase the results and plot files if they exist
    rm -f "$RESULTS_FILE"
    rm -f "$PLOT_FILE"

    # Run the planning service script
    ./bazelw run shokunin/go_to_linear_move -- \
        -c "$CONTEXT_ID" \
        -o "$OPTIONS_FILE" \
        -r "/$RESULTS_FILE" \
        --wait_for_viz true \
        -v 0
    TEST_EXIT_CODE=$?
    if [ "$TEST_EXIT_CODE" -eq 0 ]; then
        echo "✅ ------ Test $TEST_NAME passed ------"
    else
        echo "❌ ------ Test $TEST_NAME failed ------"
    fi
    return ${TEST_EXIT_CODE}
}

# List of test commands
tests=(
    go_to_plate_west
    go_to_plate_east
)

for test in "${tests[@]}"; do
    run_test "$test" || overall_success=1
    # return immediately if a test fails
    if [ "${overall_success:-0}" -eq 1 ]; then
        echo "❌ Some tests failed. Exiting."
        exit 1
    fi
done

./bazelw run shokunin/solve_saved_problems -- -d /data/shokunin/system_tests/problems_dir/
TEST_EXIT_CODE=$?
if [ "$TEST_EXIT_CODE" -eq 0 ]; then
    echo "✅ ------ All saved problem successfully solved ------"
else
    echo "❌ ------ Solving some or all saved problems failed ------"
    exit 1
fi

echo "🎉🎉🎉 All tests passed! 🎉🎉🎉"
exit 0
