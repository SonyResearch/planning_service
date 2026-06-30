# Python client tests
set -eufo pipefail

bazel test --test_output=errors --verbose_failures //client/... "$@"
